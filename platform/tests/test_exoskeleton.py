import asyncio
import copy
import math
import time

import httpx
import pytest

from hc_platform.exoskeleton import ExoskeletonNormalizer, ExoskeletonInputError
from hc_platform.exoskeleton.gateway import ExoskeletonRelay


@pytest.fixture
def exo_config(tmp_path):
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        """<robot name="exo"><link name="root"/><link name="arm"/><link name="tip"/><joint name="rotation" type="revolute"><parent link="root"/><child link="arm"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="1" effort="10"/></joint><joint name="slide" type="prismatic"><parent link="arm"/><child link="tip"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" velocity="1" effort="10"/></joint></robot>"""
    )
    return {
        "urdf": str(urdf),
        "joint_names": ["rotation", "slide"],
        "lower": [-3, -1],
        "upper": [3, 1],
        "exoskeleton": {
            "enabled": True,
            "joint_mapping": {
                "angle": {
                    "name": "rotation",
                    "unit": "deg",
                    "scale": -1,
                    "offset": 0.1,
                },
                "distance": {"name": "slide", "unit": "mm"},
            },
        },
    }


def test_name_units_and_original_sample_time(exo_config):
    normalizer = ExoskeletonNormalizer(exo_config)
    result = normalizer.normalize(
        ["angle", "distance"], [90, 200], 1_000_000_000, 1_010_000_000, 2_000_000_000
    )
    assert result["joint_names"] == ["rotation", "slide"]
    assert result["positions"] == pytest.approx([0.1 - math.pi / 2, 0.2])
    assert result["sample_mono_ns"] == 1_990_000_000
    assert result["deadline_mono_ns"] == 2_090_000_000
    with pytest.raises(ExoskeletonInputError, match="NONINCREASING"):
        normalizer.normalize(
            ["angle"], [0], 1_000_000_000, 1_020_000_000, 2_020_000_000
        )
    with pytest.raises(ExoskeletonInputError, match="STALE"):
        normalizer.normalize(
            ["angle"], [0], 1_000_000_001, 2_000_000_000, 3_000_000_000
        )
    with pytest.raises(ExoskeletonInputError, match="FUTURE"):
        normalizer.normalize(
            ["angle"], [0], 3_000_000_000, 2_000_000_000, 3_000_000_000
        )


def test_invalid_alias_dimension_and_bounds(exo_config):
    normalizer = ExoskeletonNormalizer(exo_config)
    with pytest.raises(ExoskeletonInputError, match="JOINT_LIMIT"):
        normalizer.normalize(["angle"], [2000], 1, 1, 1)
    with pytest.raises(ExoskeletonInputError, match="UNKNOWN_JOINT"):
        normalizer.normalize(["unknown"], [0], 1, 1, 1)
    with pytest.raises(ExoskeletonInputError, match="NONFINITE"):
        normalizer.normalize(["angle"], [float("nan")], 1, 1, 1)
    exo_config["exoskeleton"]["joint_mapping"]["wrong"] = {
        "name": "slide",
        "unit": "deg",
    }
    with pytest.raises(ExoskeletonInputError, match="DIMENSION"):
        normalizer.normalize(["wrong"], [0], 1, 1, 1)
    exo_config["exoskeleton"]["joint_mapping"]["alias"] = "rotation"
    with pytest.raises(ExoskeletonInputError, match="DUPLICATE"):
        normalizer.normalize(["angle", "alias"], [0, 0], 1, 1, 1)


def test_relay_only_new_samples_after_selected_epoch(exo_config):
    async def run():
        state = {
            "boot_id": "test",
            "epoch": 1,
            "lease_id": 1,
            "reference_epoch": 1,
            "safety": "ready",
            "owner": "none",
            "mode": "exo",
            "transition_pending": False,
            "last_command_seq": 0,
        }
        sent = []

        def handler(request):
            if request.url.path == "/api/control/state":
                return httpx.Response(200, json=copy.deepcopy(state))
            assert request.url.path == "/api/control/command"
            import json

            sent.append(json.loads(request.content))
            return httpx.Response(200, json={"ok": True})

        async with httpx.AsyncClient(
            transport=httpx.MockTransport(handler), base_url="http://robot"
        ) as http:
            relay = ExoskeletonRelay(exo_config, http)
            normalizer = ExoskeletonNormalizer(exo_config)
            old = normalizer.normalize(["angle"], [0], 1, 1, time.monotonic_ns())
            assert not await relay.poll_state()
            assert not await relay.forward(old)
            state["owner"] = "exo"
            assert await relay.poll_state()
            assert not await relay.forward(old)
            fresh = normalizer.normalize(["angle"], [1], 2, 2, time.monotonic_ns())
            assert await relay.forward(fresh)
            assert not await relay.forward(fresh)
            assert sent[-1]["sample_mono_ns"] == fresh["sample_mono_ns"]
            assert sent[-1]["deadline_mono_ns"] == fresh["deadline_mono_ns"]
            state.update(epoch=2, lease_id=2, reference_epoch=2)
            await relay.poll_state()
            assert not await relay.forward(fresh)
            state["owner"] = "human"
            assert not await relay.poll_state()
            newer = normalizer.normalize(["angle"], [2], 3, 3, time.monotonic_ns())
            assert not await relay.forward(newer)
            assert len(sent) == 1 and sent[0]["source"] == "exo"

    asyncio.run(run())
