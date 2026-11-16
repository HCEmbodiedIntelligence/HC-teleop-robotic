"""Bounded same-host monotonic reorder window for confirmed DAgger cuts."""

import bisect
import heapq

from .artifacts import canonical
from .errors import DataError


class DaggerWindow:
    def __init__(
        self,
        origin_ns,
        *,
        delay_ns=500_000_000,
        max_bytes=64 * 1024 * 1024,
        max_messages=4096,
        corrections=True,
    ):
        self.origin_ns, self.delay_ns = origin_ns, delay_ns
        self.max_bytes, self.max_messages, self.corrections = (
            max_bytes,
            max_messages,
            corrections,
        )
        self.samples, self.events, self.identities = [], [], {}
        self.bytes = self.counter = self.admitted = self.selected = self.excluded = 0
        self.event_bytes = 0
        self.watermark_ns = origin_ns - 1
        self.exclusion_reasons = {}

    def exclude(self, reason):
        self.excluded += 1
        self.exclusion_reasons[reason] = self.exclusion_reasons.get(reason, 0) + 1

    def add_event(self, active, event):
        value = {**event, "active": bool(active)}
        encoded = canonical(value)
        if len(encoded) > 16 * 1024:
            raise DataError("INTERVENTION_EVENT_TOO_LARGE")
        identity = value["event_id"]
        if identity in self.identities:
            if self.identities[identity] != encoded:
                raise DataError("INTERVENTION_EVENT_CONFLICT")
            return False
        if value["monotonic_ns"] <= self.watermark_ns:
            raise DataError(
                "DAGGER_TRANSITION_TOO_LATE",
                "Ownership event arrived behind the committed sample watermark",
            )
        if (
            len(self.events) >= 65536
            or self.event_bytes + len(encoded) > 4 * 1024 * 1024
        ):
            raise DataError("DAGGER_EVENT_BUDGET_EXCEEDED")
        self.identities[identity] = encoded
        self.event_bytes += len(encoded)
        self.events.append(value)
        self.events.sort(key=lambda item: (item["monotonic_ns"], item["control_epoch"]))
        return True

    def add_sample(self, item, sample_ns, metadata):
        self.admitted += 1
        if sample_ns is None:
            self.exclude("sample_clock_unknown")
            return False
        if sample_ns < self.origin_ns:
            self.exclude("before_session")
            return False
        size = len(item[1]) + len(item[5].encode()) + 512
        if len(self.samples) >= self.max_messages or self.bytes + size > self.max_bytes:
            raise DataError("DAGGER_REORDER_OVERFLOW")
        self.counter += 1
        heapq.heappush(self.samples, (sample_ns, self.counter, size, item, metadata))
        self.bytes += size
        return True

    def _validate_events(self, through_ns):
        active, previous_epoch, boot = False, -1, None
        for event in self.events:
            if event["monotonic_ns"] > through_ns:
                break
            if event["active"] == active or event["control_epoch"] <= previous_epoch:
                raise DataError(
                    "INTERVENTION_TRANSITION_INVALID",
                    "Unresolved ownership sequence after the reorder window",
                )
            if boot is not None and event.get("boot_id", boot) != boot:
                raise DataError("DAGGER_CORE_RESTART")
            boot = event.get("boot_id", boot)
            active, previous_epoch = event["active"], event["control_epoch"]

    def drain(self, now_ns, *, final=False, stop_ns=None):
        through = now_ns if final else now_ns - self.delay_ns
        self._validate_events(through)
        times = [event["monotonic_ns"] for event in self.events]
        ready = []
        while self.samples and self.samples[0][0] <= through:
            stamp, _, size, item, metadata = heapq.heappop(self.samples)
            self.bytes -= size
            self.watermark_ns = max(self.watermark_ns, stamp)
            if stop_ns is not None and stamp > stop_ns:
                self.exclude("after_session_stop")
                continue
            index = bisect.bisect_right(times, stamp) - 1
            event = self.events[index] if index >= 0 else None
            human = bool(event and event["active"])
            reason = None
            if self.corrections and not human:
                reason = "outside_confirmed_intervention"
            elif self.corrections:
                epoch, owner = (
                    metadata.get("control_epoch"),
                    metadata.get("control_owner"),
                )
                next_epoch = (
                    self.events[index + 1]["control_epoch"]
                    if index + 1 < len(self.events)
                    else None
                )
                if epoch is not None and (
                    epoch < event["control_epoch"]
                    or (next_epoch is not None and epoch >= next_epoch)
                ):
                    reason = "command_epoch_outside_intervention"
                elif owner is not None and owner not in {"human", "vr"}:
                    reason = "command_source_not_human"
                elif (
                    metadata.get("control_boot_id")
                    and event.get("boot_id")
                    and metadata["control_boot_id"] != event["boot_id"]
                ):
                    reason = "command_boot_mismatch"
            if reason:
                self.exclude(reason)
                continue
            self.selected += 1
            classification = {
                **metadata,
                "sample_monotonic_ns": stamp,
                "human_intervention": human,
                "intervention_event_id": event["event_id"] if human else None,
                "intervention_epoch": event["control_epoch"] if human else None,
                "classification": "within_confirmed_intervention"
                if human
                else "automatic_or_paused",
            }
            ready.append((item, classification))
        return ready

    def active_event(self, through_ns):
        previous = None
        for event in self.events:
            if event["monotonic_ns"] > through_ns:
                break
            previous = event
        return previous if previous and previous["active"] else None

    def intervals(self, through_ns):
        intervals, current = [], None
        for event in self.events:
            if event["monotonic_ns"] > through_ns:
                break
            if event["active"]:
                current = {
                    "start_monotonic_ns": event["monotonic_ns"],
                    "start_epoch": event["control_epoch"],
                    "start_event_id": event["event_id"],
                    "end_monotonic_ns": None,
                    "end_epoch": None,
                    "complete": False,
                }
                intervals.append(current)
            elif current:
                current.update(
                    end_monotonic_ns=event["monotonic_ns"],
                    end_epoch=event["control_epoch"],
                    end_event_id=event["event_id"],
                    complete=True,
                )
                current = None
        return intervals

    def statistics(self):
        return {
            "admitted": self.admitted,
            "selected": self.selected,
            "excluded": self.excluded,
            "pending_messages": len(self.samples),
            "pending_bytes": self.bytes,
            "watermark_monotonic_ns": self.watermark_ns,
            "window_ns": self.delay_ns,
            "exclusion_reasons": dict(self.exclusion_reasons),
            "event_count": len(self.events),
            "event_bytes": self.event_bytes,
        }
