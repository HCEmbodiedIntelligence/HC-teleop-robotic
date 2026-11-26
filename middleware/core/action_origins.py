"""Track the original /action/joint_states label selected on each training grid row."""
import struct

from .recording_cdr import read_event
from .recording_action_labels import ACTION_TOPIC, FEEDBACK_TOPIC


class ActionOriginIndex:
    """Join provenance on MCAP log time before translating to the alignment clock."""

    def __init__(self):
        self.by_log_time = {}

    def add_event(self, log_time_ns, raw, message_encoding, schema_name):
        try:
            marker = read_event(raw, message_encoding, schema_name)
        except (ValueError, TypeError, struct.error):
            marker = None
        if not isinstance(marker, dict):
            return
        if marker.get('included_in_action_labels') is not True:
            return
        source = marker.get('source')
        published = source == 'published_action'
        source_topic = ACTION_TOPIC if published else FEEDBACK_TOPIC
        if (marker.get('schema_version') != 'hc-action-provenance/v1'
                or marker.get('action_log_time_ns') != log_time_ns
                or marker.get('action_topic', ACTION_TOPIC) != ACTION_TOPIC
                or source not in ('published_action', 'feedback_hold')
                or marker.get('sent_to_robot') is not published
                or marker.get('source_topic', source_topic) != source_topic
                or marker.get('publication_ns') != (log_time_ns if published else None)):
            return
        # Two included markers at one log time cannot identify one label uniquely.
        if log_time_ns in self.by_log_time:
            self.by_log_time[log_time_ns] = None
        else:
            self.by_log_time[log_time_ns] = (source, marker.get('source_stamp_ns'))

    def align(self, original_action_samples, headers, clock):
        """Use the same header or log clock selected for the action stream by the exporter."""
        origins = {}
        if not self.by_log_time:
            return origins
        # iter_messages(log_time_order=True) keeps duplicate receive times adjacent.
        for i, (log_time, _) in enumerate(original_action_samples):
            marker = self.by_log_time.get(log_time)
            if marker is None:
                continue
            if ((i and original_action_samples[i-1][0] == log_time)
                    or (i+1 < len(original_action_samples) and original_action_samples[i+1][0] == log_time)):
                continue
            source, source_stamp = marker
            header = headers[i]
            if source_stamp is not None and source_stamp != header:
                continue
            origins[header if clock == 'header' else log_time] = source
        return origins


def append_origin_range(ranges, frame_index, selected_action, aligned_sources):
    """Append an episode-local, half-open frame interval; freshness is tracked separately."""
    source = ('missing' if selected_action is None else
              aligned_sources.get(selected_action[0], 'unverified'))
    if ranges and ranges[-1]['source'] == source:
        ranges[-1]['end_frame_exclusive'] = frame_index + 1
    else:
        ranges.append(dict(start_frame=frame_index, end_frame_exclusive=frame_index+1, source=source))
