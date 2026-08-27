import json
from pathlib import Path

import pytest

from hc_dataset.catalog import DatasetCatalog, DatasetError


def test_catalog_round_trip_and_atomic_finalize(tmp_path: Path) -> None:
    catalog = DatasetCatalog(tmp_path)
    created = catalog.create(
        "OpenArmX Trial", ["/robots/openarmx/input/vr_frame"], {"operator": "test"}
    )
    assert created.manifest["state"] == "recording"
    completed = catalog.finalize(
        created.dataset_id, state="complete", message_count=42, bytes_written=100
    )
    assert completed.manifest["message_count"] == 42
    assert catalog.list()[0].dataset_id == created.dataset_id
    assert json.loads(completed.manifest_path.read_text())["schema"] == "hc-dataset/v1"


@pytest.mark.parametrize("dataset_id", ["../escape", "/tmp/escape", "bad id", ""])
def test_catalog_rejects_unsafe_ids(tmp_path: Path, dataset_id: str) -> None:
    with pytest.raises(DatasetError):
        DatasetCatalog(tmp_path).get(dataset_id)


def test_catalog_requires_explicit_absolute_unique_topics(tmp_path: Path) -> None:
    catalog = DatasetCatalog(tmp_path)
    for topics in ([], ["relative"], ["/same", "/same"]):
        with pytest.raises(DatasetError):
            catalog.create("bad", topics)
