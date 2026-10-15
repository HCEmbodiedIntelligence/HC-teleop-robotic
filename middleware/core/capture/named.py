"""Per-axis association for partial driver messages. Never invent a target.

Validity evidence is supplied by the command producer, on the same ordered
channel as commands and invalidations. Bare JointState has no such evidence.
"""
import math


def action_at(documents, axes, target, epoch, observed=None):
    values, valid, stamps, evidence = [], [], [], []
    documents = [d for d in documents if d['clock_epoch'] == epoch
                 and (observed is None or d['receive_time_ns'] <= observed)]
    for axis in axes:
        name = axis['name']
        relevant = [d for d in documents if name in d['joint_names']
                    and d['capture_time_ns'] <= target]
        events = [d for d in relevant if d.get('validity_event')]
        event = max(events, key=lambda d: (d['capture_time_ns'], d['source_seq'])) if events else None
        boundary = event['capture_time_ns'] if event else -1
        commands = [d for d in relevant if not d.get('validity_event')
                    and d['capture_time_ns'] > boundary]
        command = max(commands, key=lambda d: (d['capture_time_ns'], d['source_seq'])) if commands else None
        good, value, stamp = False, None, None
        if command:
            stamp = command['capture_time_ns']
            value = command['payload']['position'][command['joint_names'].index(name)]
            good = (command.get('valid', True) and command.get('command_valid', True)
                    and command.get('validity_known') is True and command.get('source_epoch') is not None
                    and (event is None or command['source_epoch'] == event['source_epoch'])
                    and target - stamp <= axis['max_age_ms'] * 1_000_000
                    and isinstance(value, (int, float)) and math.isfinite(value))
            if axis.get('mode', 'periodic') == 'on_change':
                good = good and any(lease['command_sequence'] == command['source_seq']
                                    and lease['source_epoch'] == command['source_epoch']
                                    and lease['issued_ns'] <= target <= lease['valid_until_ns']
                                    for lease in command.get('hold_leases', {}).get(name, []))
        values.append(value if good else None)
        valid.append(bool(good))
        stamps.append(stamp)
        evidence.append({key: command.get(key) if command else None
                         for key in ('source_seq', 'source_epoch', 'receive_time_ns', 'validity_known')})
    return {'values': values, 'valid': valid, 'source_ns': stamps, 'evidence': evidence}


def state_at(documents, axes, target, epoch, max_gap_ns):
    values, valid, brackets, received = [], [], [], []
    evidence = []
    for axis in axes:
        name = axis['name']
        data = [d for d in documents if d['clock_epoch'] == epoch and name in d['joint_names']]
        before = [d for d in data if d['capture_time_ns'] <= target]
        after = [d for d in data if d['capture_time_ns'] >= target]
        left = max(before, key=lambda d: (d['capture_time_ns'], d['source_seq'])) if before else None
        right = min(after, key=lambda d: (d['capture_time_ns'], -d['source_seq'])) if after else None
        good, value, bracket = False, None, None
        if left and right:
            bracket = [left['capture_time_ns'], right['capture_time_ns']]
            gap = bracket[1] - bracket[0]
            a = left['payload']['position'][left['joint_names'].index(name)]
            b = right['payload']['position'][right['joint_names'].index(name)]
            good = (gap <= max_gap_ns and left.get('valid', True) and right.get('valid', True)
                    and left.get('clock_id') == right.get('clock_id')
                    and left.get('source_epoch') == right.get('source_epoch')
                    and left.get('continuity_id') == right.get('continuity_id')
                    and all(isinstance(x, (int, float)) and math.isfinite(x) for x in (a, b)))
            if good:
                # Joint positions are continuous coordinates, not wrapped Euler angles.
                alpha = (target - bracket[0]) / gap if gap else 0
                value = a + alpha * (b - a)
        values.append(value)
        valid.append(bool(good))
        brackets.append(bracket)
        received.append(max(left['receive_time_ns'], right['receive_time_ns']) if left and right else None)
        evidence.append({'target_ns': target,
                         'left_seq': left['source_seq'] if left else None,
                         'right_seq': right['source_seq'] if right else None,
                         'alpha': alpha if good else None,
                         'clock_id': left.get('clock_id') if left else None,
                         'clock_epoch': left.get('clock_epoch') if left else None,
                         'source_epoch': left.get('source_epoch') if left else None,
                         'continuity_id': left.get('continuity_id') if left else None,
                         'timestamp_semantics': [d.get('timestamp_semantics') if d else None for d in (left,right)]})
    return {'values': values, 'valid': valid, 'brackets_ns': brackets, 'receive_ns': received,
            'interpolation': evidence}


def aligned(samples, target, source, alignment, epoch):
    documents = [s.data() for s in samples]
    if alignment['strict']:
        from .alignment import clock_reasons
        for document in documents:
            if clock_reasons(document):
                document['valid'] = False
    if source['kind'] == 'action':
        result = action_at(documents, source['named_axes'], target, epoch)
        value = {'payload': {'position': result['values']}, 'components': result}
    else:
        result = state_at(documents, source['named_axes'], target, epoch,
                          round(alignment['joint_max_bracket_gap_ms'] * 1e6))
        value = {'position': {'value': result['values'], 'bracket_gap_ns': max(
            (b[1] - b[0] for b in result['brackets_ns'] if b), default=0)}, 'components': result}
    errors = [axis['name'] + ':' + source['kind'] + '_missing_invalid_or_expired'
              for axis, good in zip(source['named_axes'], result['valid']) if not good]
    return value, errors
