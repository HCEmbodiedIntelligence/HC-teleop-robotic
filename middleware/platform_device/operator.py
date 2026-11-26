"""WebRTC session lifecycle shared by non-browser operators and policy clients."""

import asyncio
from contextlib import asynccontextmanager, suppress

from aiortc import (
    RTCConfiguration,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
)

from middleware.platform_sessions import OperatorPlatform, PlatformHTTPError


@asynccontextmanager
async def operator_peer(platform: OperatorPlatform, robot_id: str, configure):
    """Reserve a robot and yield (peer, session_id) after exchanging SDP.

    configure(peer, session_id) adds the consumer's media and channels before the
    offer. The lease is renewed while the context is open. Losing the lease
    closes the peer and propagates the sync error on context exit.
    """
    assignment = await platform.reserve(robot_id)
    sid = assignment["session_id"]
    peer = None
    renewer = None
    try:
        ice = [RTCIceServer(**item) for item in assignment.get("ice_servers", [])]
        peer = RTCPeerConnection(RTCConfiguration(ice))
        configure(peer, sid)

        async def negotiate():
            await peer.setLocalDescription(await peer.createOffer())
            offer = peer.localDescription.sdp
            while True:
                state = await platform.sync(robot_id, sid, offer)
                if state.get("answer"):
                    await peer.setRemoteDescription(
                        RTCSessionDescription(state["answer"], "answer")
                    )
                    return
                offer = None
                await asyncio.sleep(0.5)

        async def renew():
            try:
                while True:
                    await asyncio.sleep(2)
                    await platform.sync(robot_id, sid)
            except Exception:
                await peer.close()
                raise

        await asyncio.wait_for(negotiate(), timeout=30)
        renewer = asyncio.create_task(renew())
        yield peer, sid
    finally:
        try:
            if renewer is not None:
                renewer.cancel()
                with suppress(asyncio.CancelledError):
                    await renewer
        finally:
            try:
                if peer is not None:
                    await peer.close()
            finally:
                try:
                    await platform.release(robot_id, sid)
                except PlatformHTTPError as exc:
                    if (
                        exc.status != 409
                    ):  # The platform may already have reclaimed the lease.
                        raise
