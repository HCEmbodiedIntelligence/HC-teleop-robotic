import asyncio
import tempfile
from unittest.mock import patch
import unittest
from pathlib import Path

try:
    import aiohttp
    from aiohttp.test_utils import TestClient, TestServer
except ImportError:  # The base ROS Python lacks web dependencies before install.sh.
    aiohttp = None

from middleware.core.config import ConfigStore
from tests.test_robot_profiles import URDF, io_yaml


@unittest.skipIf(aiohttp is None, "aiohttp is installed by install.sh")
class RobotProfileApiTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        from middleware.core.app import create_app, MiddlewareRuntime
        original_start = MiddlewareRuntime.start
        async def start(instance):
            self.runtime = instance
            await original_start(instance)
        self.start_patch = patch.object(MiddlewareRuntime, "start", start)
        self.start_patch.start()

        self.directory = tempfile.TemporaryDirectory()
        root = Path(self.directory.name)
        self.store = ConfigStore(root / "middleware.yaml")
        value = self.store.load()
        value["robot_profiles"] = {"root": str(root / "profiles"), "active": ""}
        value["ros"]["enabled"] = False
        value["ros"]["subscriptions"] = [{"topic": "/observation/joint_states", "type": "sensor_msgs/msg/JointState", "outputs": ["record"]}]
        value["vr"]["enabled"] = False
        value["camera"]["enabled"] = False
        value["safety"]["enabled"] = False
        value["safety"]["stop_on_startup"] = False
        value["ros"]["recording"].update(directory=str(root/"data"), job_state_dir=str(root/"jobs"), artifact_state_dir=str(root/"ready"))
        self.store.save(value)
        self.client = TestClient(TestServer(create_app(self.store)))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()
        self.directory.cleanup()
        self.start_patch.stop()

    async def test_import_list_and_activate_profile(self):
        form = aiohttp.FormData()
        form.add_field("id", "api_robot")
        form.add_field("display_name", "API Robot")
        form.add_field(
            "urdf", URDF, filename="robot.urdf", content_type="application/xml"
        )
        form.add_field(
            "config",
            io_yaml(),
            filename="vr_configs.yml",
            content_type="application/yaml",
        )
        response = await self.client.post("/api/robot-profiles/import", data=form)
        self.assertEqual(response.status, 201, await response.text())
        imported = await response.json()
        self.assertEqual(imported["profile"]["schema"], "hc-robot-config-v1")

        response = await self.client.get("/api/robot-profiles")
        listed = await response.json()
        self.assertEqual([item["id"] for item in listed["profiles"]], ["api_robot"])

        response = await self.client.post(
            "/api/robot-profiles/api_robot/activate"
        )
        self.assertEqual(response.status, 200, await response.text())
        activated = await response.json()
        self.assertTrue(activated["restart_simulation_required"])
        self.assertEqual(
            ConfigStore(self.store.path).load()["robot_profiles"]["active"],
            "api_robot",
        )

    async def test_import_zip_archive_api(self):
        import io
        import zipfile

        zip_buffer = io.BytesIO()
        with zipfile.ZipFile(zip_buffer, "w") as zf:
            zf.writestr("archive_bot/urdf/robot.urdf", URDF)
            zf.writestr("archive_bot/vr_configs.yml", io_yaml())
            zf.writestr("archive_bot/mesh/base.stl", b"mesh_data")

        form = aiohttp.FormData()
        form.add_field("id", "archive_bot")
        form.add_field("display_name", "Archive Bot")
        form.add_field(
            "archive",
            zip_buffer.getvalue(),
            filename="archive_bot.zip",
            content_type="application/zip",
        )
        response = await self.client.post("/api/robot-profiles/import", data=form)
        self.assertEqual(response.status, 201, await response.text())
        imported = await response.json()
        self.assertEqual(imported["profile"]["id"], "archive_bot")
        self.assertEqual(imported["profile"]["schema"], "hc-robot-config-v1")

    async def test_service_and_vr_settings_are_written_to_yaml(self):
        response = await self.client.get("/api/config")
        value = await response.json()
        value["server"]["host"] = "127.0.0.1"
        value["vr"]["pose_port"] = 5105
        value["vr"]["data_topic"] = "/vrdata"
        response = await self.client.put("/api/config", json=value)
        self.assertEqual(response.status, 200, await response.text())

        saved = ConfigStore(self.store.path).load()
        self.assertEqual(saved["server"]["host"], "127.0.0.1")
        self.assertEqual(saved["vr"]["pose_port"], 5105)
        self.assertEqual(saved["vr"]["data_topic"], "/vrdata")

    async def test_delete_robot_profile_api(self):
        form = aiohttp.FormData()
        form.add_field("id", "temp_robot")
        form.add_field("display_name", "Temp Robot")
        form.add_field("urdf", URDF, filename="robot.urdf", content_type="application/xml")
        form.add_field("config", io_yaml(), filename="vr_configs.yml", content_type="application/yaml")
        response = await self.client.post("/api/robot-profiles/import", data=form)
        self.assertEqual(response.status, 201)

        # Delete active profile clears active in config
        await self.client.post("/api/robot-profiles/temp_robot/activate")
        response = await self.client.delete("/api/robot-profiles/temp_robot")
        self.assertEqual(response.status, 200)
        json_data = await response.json()
        self.assertEqual(json_data["deleted"], "temp_robot")
        self.assertTrue(json_data["cleared_active"])
        self.assertEqual(
            ConfigStore(self.store.path).load()["robot_profiles"]["active"], ""
        )

    async def test_recording_precheck_api(self):
        # Precheck endpoint
        pre_res = await self.client.post("/api/recording/precheck")
        self.assertEqual(pre_res.status, 200)
        pre_data = await pre_res.json()
        self.assertTrue(pre_data["ok"])

        # Without force, start should be rejected if topics not ready or none configured
        start_fail = await self.client.post("/api/recording/start", json={"filename": "fail.mcap", "force": False})
        self.assertEqual(start_fail.status, 400)
        fail_data = await start_fail.json()
        self.assertFalse(fail_data["ok"])
        forced = await self.client.post("/api/recording/start", json={"force": True})
        self.assertEqual(forced.status, 400)

    async def test_recording_api_start_stop(self):
        self.runtime.recorder.health = lambda: {item["topic"]: {"state": "ok"} for item in self.runtime.config["ros"]["subscriptions"]}
        res = await self.client.post("/api/recording/start", json={"filename": "api_test.mcap", "force": True})
        self.assertEqual(res.status, 200)
        data = await res.json()
        self.assertTrue(data["ok"])
        self.assertTrue(data["recording"])
        self.assertTrue(data["path"].endswith("api_test.mcap"))

        # Check status
        status_res = await self.client.get("/api/recording/status")
        self.assertEqual(status_res.status, 200)
        self.assertTrue((await status_res.json())["recording"])

        # Stop recording
        stop_res = await self.client.post("/api/recording/stop")
        self.assertEqual(stop_res.status, 200)
        stop_data = await stop_res.json()
        self.assertTrue(stop_data["ok"])
        self.assertFalse(stop_data["recording"])

    async def completed_recording(self, name):
        self.runtime.recorder.health = lambda: {item["topic"]: {"state": "ok"} for item in self.runtime.config["ros"]["subscriptions"]}
        response = await self.client.post("/api/recording/start", json={"filename": name})
        self.assertEqual(response.status, 200, await response.text())
        filename = Path((await response.json())["path"]).name
        self.runtime.recorder.record({"kind": "event", "payload": {"synthetic": True}})
        await self.client.post("/api/recording/stop")
        for _ in range(500):
            if all(j['artifact_state'] == 'ready' for j in self.runtime.recorder.jobs.values()):
                return filename
            await asyncio.sleep(.02)
        self.fail(str(self.runtime.recorder.jobs))

    async def test_dataset_management_apis(self):
        filename = await self.completed_recording("ds_test.mcap")

        # List recordings
        list_res = await self.client.get("/api/recordings")
        self.assertEqual(list_res.status, 200)
        list_data = await list_res.json()
        self.assertTrue(any(f["filename"] == filename for f in list_data["files"]))

        # Download recording
        down_res = await self.client.get(f"/api/recordings/{filename}/download")
        self.assertEqual(down_res.status, 200)
        content = await down_res.read()
        self.assertGreater(len(content), 0)

        # Batch delete
        del_res = await self.client.post(
            "/api/recordings/batch-delete",
            json={"filenames": [filename]},
        )
        self.assertEqual(del_res.status, 200)
        del_data = await del_res.json()
        self.assertIn(filename, del_data["deleted"], del_data)

    async def test_replay_apis(self):
        filename = await self.completed_recording("replay_test.mcap")

        # Start replay
        res = await self.client.post(
            "/api/replay/start",
            json={"filename": filename, "speed": 2.0, "loop": False},
        )
        self.assertEqual(res.status, 200)
        data = await res.json()
        self.assertTrue(data["ok"])
        self.assertEqual(data["replay"]["filename"], filename)

        # Check status
        st_res = await self.client.get("/api/replay/status")
        self.assertEqual(st_res.status, 200)

        # Pause replay
        p_res = await self.client.post("/api/replay/pause")
        self.assertEqual(p_res.status, 200)

        # Resume replay
        r_res = await self.client.post("/api/replay/resume")
        self.assertEqual(r_res.status, 200)

        # Stop replay
        stop_res = await self.client.post("/api/replay/stop")
        self.assertEqual(stop_res.status, 200)
        stop_data = await stop_res.json()
        self.assertEqual(stop_data["replay"]["state"], "stopped")
        self.assertTrue(stop_data["replay"]["requires_reset"])


if __name__ == "__main__":
    unittest.main()
