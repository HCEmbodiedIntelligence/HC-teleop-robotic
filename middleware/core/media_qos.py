"""Best-effort DSCP on this peer's actual aioice UDP sockets.

Private aiortc/aioice integration is isolated here and covered by a real peer
connection test. A successful mark is not proof of AP/WMM prioritization.
"""
import socket


def mark_media_transport(sender, dscp=None):
    result = {'requested_dscp': dscp, 'marked_sockets': 0, 'errors': [],
              'wmm_verified': False, 'selected_path_verified': False, 'unmarked_protocols': 0, 'scope': 'ICE UDP socket (includes multiplexed traffic)'}
    if dscp is None:
        result['state'] = 'disabled'
        return result
    if isinstance(dscp, bool) or not isinstance(dscp, int) or not 0 <= dscp <= 63:
        raise ValueError('DSCP must be an integer from 0 to 63 or null')
    try:
        connection = sender.transport.transport._connection
        protocols = connection._protocols
    except AttributeError:
        result.update(state='unsupported', errors=['aiortc/aioice transport layout unsupported'])
        return result
    seen = set()
    for protocol in protocols:
        transport = getattr(protocol, 'transport', None)
        sock = transport.get_extra_info('socket') if transport else None
        if sock is None:
            result['unmarked_protocols'] += 1
            result['errors'].append('candidate transport exposes no socket (e.g. TURN); selected path unverified')
            continue
        if id(sock) in seen:
            continue
        seen.add(id(sock))
        try:
            if sock.family == socket.AF_INET:
                level, option = socket.IPPROTO_IP, socket.IP_TOS
            elif sock.family == socket.AF_INET6:
                level, option = socket.IPPROTO_IPV6, socket.IPV6_TCLASS
            else:
                continue
            # Preserve ECN bits rather than unconditionally clearing them.
            tos = (dscp << 2) | (sock.getsockopt(level, option) & 3)
            sock.setsockopt(level, option, tos)
            if sock.getsockopt(level, option) >> 2 != dscp:
                raise OSError('DSCP read-back differs from requested value')
            result['marked_sockets'] += 1
        except (OSError, AttributeError) as exc:
            result['errors'].append(str(exc))
    result['state'] = ('marked' if result['marked_sockets'] and not result['errors']
                       else 'partial' if result['marked_sockets'] else 'unavailable')
    return result
