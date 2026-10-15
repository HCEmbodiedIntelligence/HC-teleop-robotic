"""Record next-frame position targets, separately from actual driver commands.

Only numeric samples and primary camera metadata are cached. Candidates are
written during capture; the complete recording's real-action inventory decides
whether they may be used. Nothing in this module publishes to a robot.
"""
from collections import Counter, OrderedDict
import math

from .alignment import target_count, target_time
from .buffers import TimeCache
from .exposure_association import numeric_target
from .named import aligned
from .primary_timeline import (PRIMARY_ACTION_POLICY, uses_primary_timeline,
                               frame_identity, next_observation)

POLICY = 'recorder-next-state/v1'


class NextStateActions:
    def __init__(self, cfg):
        self.cfg = cfg
        self.cache = TimeCache(cfg['buffers'])
        self.actions = {k: s for k, s in cfg['sources'].items()
                        if s['kind'] == 'action' and s.get('named_axes')}
        self.states = {k: s for k, s in cfg['sources'].items()
                       if s['kind'] == 'state' and s.get('named_axes')}
        self.primary = cfg['alignment']['primary_camera']
        self.real_sources = set()
        self.counts = Counter()
        self.k = 0
        self.previous = None
        self.finished = False
        self.primary_mode = uses_primary_timeline(cfg['alignment'])
        self.pending_primary = OrderedDict()
        self.last_primary_time = None

    def observe(self, sample):
        ident = sample.source_id
        if ident in self.actions:
            # Even an invalid command or termination event prevents substitution.
            self.real_sources.add(ident)
        elif ident in self.states or ident == self.primary:
            self.cache.add(sample, sample.capture_time_ns)
            if self.primary_mode and ident == self.primary:
                data = sample.data()
                identity = frame_identity(ident, data)
                key = (identity['clock_id'], identity['clock_epoch'], identity['rgb_source_seq'])
                if self.last_primary_time is not None and sample.capture_time_ns <= self.last_primary_time:
                    raise ValueError('primary_camera_late_or_repeated_frame')
                if key not in self.pending_primary:
                    if len(self.pending_primary) >= self.cfg['buffers']['cache_items_per_source']:
                        raise ValueError('next_state_primary_queue_capacity_exceeded')
                    self.pending_primary[key] = {'identity': identity, 'source_seq': data['source_seq'],
                        'rgb': {key: data['rgb'].get(key) for key in ('capture_time_ns',
                            'exposure_midpoint_ns', 'timestamp_semantics', 'timestamp_evidence')}}

    def frame(self, k, start, epoch, primary=None):
        target = primary['identity']['capture_time_ns'] if primary else target_time(start, k, self.cfg['dataset']['fps'])
        row = {'target_frame_index': k, 'target_time_ns': target, 'images': {}}
        samples = self.cache.eligible(self.primary, 2**63-1)
        if self.cfg['alignment']['causal']:
            samples = [s for s in samples if s.capture_time_ns <= target]
        if primary:
            row['images'][self.primary] = primary
        elif samples:
            selected = min(samples, key=lambda s: (abs(s.capture_time_ns-target),
                                                  s.capture_time_ns, s.source_seq))
            row['images'][self.primary] = selected.data()
        stamp = numeric_target(row, self.cfg)
        # A wall-clock delay alone does not prove camera delivery is complete.
        # Wait for a primary sample on/after the grid before choosing its nearest
        # frame, then for right state brackets. Never reuse an old image merely
        # because the next frame is still in transport.
        ready = stamp is not None
        if not primary and self.cfg['alignment'].get('state_reference') == 'primary_exposure_midpoint':
            ready = ready and any(s.capture_time_ns >= target for s in samples)
        axes = {}
        if stamp is not None:
            for ident, source in self.states.items():
                state, _ = aligned(self.cache.eligible(ident, 2**63-1), stamp, source,
                                   self.cfg['alignment'], epoch)
                parts = state['components']
                for i, axis in enumerate(source['named_axes']):
                    axes[axis['name']] = {'source_id': ident, 'unit': axis['unit'],
                        **{key: values[i] for key, values in parts.items()}}
                    ready = ready and parts['brackets_ns'][i] is not None
        return {'index': k, 'grid_ns': target, 'time_ns': stamp,
                'clock_epoch': epoch, 'axes': axes, 'ready': bool(ready),
                **({'primary_frame': primary['identity']} if primary else {})}

    def targets(self, current, following):
        for ident, source in self.actions.items():
            if ident in self.real_sources:
                continue
            values, validity, evidence, reasons = [], [], [], []
            for axis in source['named_axes']:
                before = current['axes'].get(axis['name'], {})
                after = following['axes'].get(axis['name'], {}) if following else {}
                same = all(before.get('interpolation', {}).get(key) ==
                           after.get('interpolation', {}).get(key)
                           for key in ('clock_id', 'clock_epoch', 'source_epoch', 'continuity_id'))
                good = bool(following and current['time_ns'] is not None and
                    following['time_ns'] is not None and following['time_ns'] > current['time_ns']
                    and before.get('valid') and after.get('valid') and same
                    and before.get('unit') == after.get('unit') == axis['unit']
                    and isinstance(after.get('values'), (int, float))
                    and math.isfinite(after['values']))
                if self.primary_mode:
                    good = good and next_observation(current.get('primary_frame'),
                        following.get('primary_frame') if following else None, self.cfg['alignment'])
                values.append(after['values'] if good else None)
                validity.append(good)
                evidence.append({'state_source_id': after.get('source_id'),
                                 'state': after, 'previous_state': before})
                if not good:
                    reasons.append(axis['name'] + (':next_state_unavailable' if following
                                                    else ':next_frame_missing'))
            self.counts[ident] += 1
            yield {'schema_version': PRIMARY_ACTION_POLICY if self.primary_mode else POLICY, 'source_id': ident,
                **({'primary_frame': current['primary_frame'],
                    'next_primary_frame': following['primary_frame'] if following else None}
                   if self.primary_mode else {}),
                'source_seq': current['index'], 'target_frame_index': current['index'],
                'target_time_ns': current['grid_ns'], 'capture_time_ns': current['time_ns'],
                'clock_epoch': current['clock_epoch'], 'action_origin': 'next_state',
                'action_stage': 'derived', 'selection': 'only_if_no_real_source_in_recording',
                'next_frame_index': following['index'] if following else None,
                'next_target_time_ns': following['grid_ns'] if following else None,
                'next_state_time_ns': following['time_ns'] if following else None,
                'action_dt_ns': (following['time_ns']-current['time_ns']
                                 if following and following['time_ns'] is not None and current['time_ns'] is not None else None),
                'joint_names': [a['name'] for a in source['named_axes']],
                'joint_units': [a['unit'] for a in source['named_axes']],
                'payload': {'position': values}, 'valid': all(validity),
                'components': {'values': values, 'valid': validity,
                    'source_ns': [following['time_ns'] if following else None] * len(values),
                    'evidence': evidence}, 'invalid_reasons': reasons}

    def advance(self, start, now, *, end=None, epoch=0, final=False):
        if self.finished:
            return
        if self.primary_mode:
            yield from self.advance_primary(start, now, end=end, epoch=epoch, final=final)
            return
        delay = round(self.cfg['alignment']['delay_ms'] * 1e6)
        until = end if final else min(now-delay+1, end if end is not None else now+1)
        due = target_count(start, until, self.cfg['dataset']['fps'])
        # The session owner treats this as explicit failure, never skips targets.
        if due-self.k > self.cfg['alignment']['max_catchup_rows']:
            raise ValueError('next_state_action_catchup_budget_exceeded')
        while self.k < due:
            current = self.frame(self.k, start, epoch)
            if not current['ready']:
                wait_limit = round(self.cfg['buffers']['retention_ms'] * 1e6)
                if not final and now-current['grid_ns'] < wait_limit:
                    break
                # Bounded wait expired or recording stopped: persist explicit
                # invalidity instead of silently choosing a stale state/frame.
                current['axes'] = {}
                current['time_ns'] = None
            if self.previous is not None:
                yield from self.targets(self.previous, current)
            self.previous = current
            self.k += 1
        if final:
            if self.previous is not None:
                yield from self.targets(self.previous, None)
            self.previous = None
            self.finished = True

    def advance_primary(self, start, now, *, end=None, epoch=0, final=False):
        delay = round(self.cfg['alignment']['delay_ms'] * 1e6)
        pending = sorted(self.pending_primary.items(), key=lambda item: item[1]['identity']['capture_time_ns'])
        processed = 0
        for key, primary in pending:
            stamp = primary['identity']['capture_time_ns']
            if stamp < start or (end is not None and stamp >= end):
                del self.pending_primary[key]
                continue
            if not final and stamp > now-delay:
                break
            current = self.frame(self.k, start, epoch, primary)
            if not current['ready']:
                if not final and now-stamp < round(self.cfg['buffers']['retention_ms'] * 1e6):
                    break
                current['axes'] = {}
            if self.previous is not None:
                yield from self.targets(self.previous, current)
            self.previous = current
            self.last_primary_time = stamp
            del self.pending_primary[key]
            self.k += 1
            processed += 1
            if not final and processed >= self.cfg['alignment']['max_catchup_rows']:
                break
        if final:
            if self.previous is not None:
                yield from self.targets(self.previous, None)
            self.previous = None
            self.finished = True

    def status(self):
        return {'policy': PRIMARY_ACTION_POLICY if self.primary_mode else POLICY,
                'location': 'recorder', 'formula': 'action[t]=state[t+1]',
                'real_action_sources': sorted(self.real_sources), 'candidate_counts': dict(self.counts),
                'selected_sources': sorted(set(self.counts)-self.real_sources),
                'selection': 'only_if_no_real_source_in_recording',
                'tail_policy': 'invalid_without_next_frame', 'cache_peak_bytes': self.cache.peak_bytes}
