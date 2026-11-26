import unittest
from unittest.mock import AsyncMock, Mock, patch
from uuid import uuid4

from middleware.platform_device.agent import Agent
from middleware.platform_device.operator import operator_peer
from middleware.platform_device.sink import Observe
from middleware.platform_sessions import (
    OperatorPlatform,
    PlatformHTTPError,
    RobotPlatform,
)


class DeviceAssignmentTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.sink = Mock(spec=Observe)
        self.sink.capabilities.return_value = {}
        self.sink.receipt.return_value = None
        with patch.dict("os.environ", HC_TELEOP_ROBOT_TOKEN="fixture"):
            self.agent = Agent(
                {"name": "bench", "base_url": "http://localhost"}, None, self.sink
            )
        self.agent.platform = AsyncMock(spec=RobotPlatform)

    async def test_release_closes_capture_even_before_peer_negotiation(self):
        sid = str(uuid4())
        command = {"session_id": sid}
        self.agent.platform.sync.side_effect = [
            {"session_id": sid, "capture_command": command},
            {},
        ]
        await self.agent.poll()
        self.sink.command.assert_called_once_with(command)
        await self.agent.poll()
        self.sink.end.assert_awaited_once_with(sid)
        self.assertIsNone(self.agent.session_id)

    async def test_instance_conflict_does_not_ack_local_receipt(self):
        self.sink.receipt.return_value = (str(uuid4()), {"request_id": str(uuid4())})
        self.agent.platform.sync.side_effect = PlatformHTTPError(409)
        with self.assertRaises(PlatformHTTPError):
            await self.agent.poll()
        self.sink.ack.assert_not_awaited()

    async def test_superseded_receipt_does_not_block_next_session(self):
        receipt = (str(uuid4()), {"request_id": str(uuid4())})
        self.sink.receipt.return_value = receipt
        sid = str(uuid4())
        self.agent.platform.sync.side_effect = [
            PlatformHTTPError(409),
            {"session_id": sid},
        ]
        await self.agent.poll()
        self.sink.ack.assert_awaited_once_with(receipt)
        self.assertEqual(self.agent.session_id, sid)


class OperatorLeaseTests(unittest.IsolatedAsyncioTestCase):
    async def test_media_setup_failure_releases_reservation(self):
        platform = AsyncMock(spec=OperatorPlatform)
        platform.reserve.return_value = {"session_id": "sid", "ice_servers": []}
        configure = Mock(side_effect=ValueError("unsupported media"))
        with self.assertRaisesRegex(ValueError, "unsupported media"):
            async with operator_peer(platform, "robot", configure):
                self.fail("must not yield an unusable peer")
        peer, _ = configure.call_args.args
        self.assertEqual(peer.connectionState, "closed")
        platform.release.assert_awaited_once_with("robot", "sid")
