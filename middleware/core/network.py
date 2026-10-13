"""IPv4 interface-aware discovery helpers for the Linux robot host."""
import ipaddress
import json
import socket
import struct
import subprocess
import sys

IP_PKTINFO = getattr(socket, 'IP_PKTINFO', 8)  # Linux uapi/linux/in.h


def enable_packet_info(sock):
    if not sys.platform.startswith('linux'):
        return False
    sock.setsockopt(socket.IPPROTO_IP, IP_PKTINFO, 1)
    return True


def receive_packet(sock, enabled):
    if not enabled:
        packet, peer = sock.recvfrom(65535)
        return packet, peer, None
    packet, ancillary, flags, peer = sock.recvmsg(65535, socket.CMSG_SPACE(12))
    for level, kind, data in ancillary:
        if level == socket.IPPROTO_IP and kind == IP_PKTINFO and len(data) >= 12:
            index, local, destination = struct.unpack('=I4s4s', data[:12])
            # ipi_spec_dst is the local interface address; ipi_addr may be broadcast.
            return packet, peer, struct.pack('=I4s4s', index, local, b'\0' * 4)
    return packet, peer, None


def reply_packet(sock, payload, peer, packet_info):
    if packet_info is None:
        return sock.sendto(payload, peer)
    return sock.sendmsg([payload], [(socket.IPPROTO_IP, IP_PKTINFO, packet_info)], 0, peer)


def local_addresses():
    """List actual interface addresses instead of guessing from an Internet route."""
    try:
        result = subprocess.run(['ip', '-j', '-4', 'addr', 'show', 'up'], capture_output=True,
                                text=True, timeout=2, check=True)
        addresses = []
        for interface in json.loads(result.stdout):
            for address in interface.get('addr_info', []):
                host = address.get('local', '')
                if address.get('scope') == 'global' and not ipaddress.ip_address(host).is_loopback:
                    addresses.append((interface['ifname'], host))
        return addresses or [('lo', '127.0.0.1')]
    except (OSError, ValueError, subprocess.SubprocessError):
        return [('lo', '127.0.0.1')]
