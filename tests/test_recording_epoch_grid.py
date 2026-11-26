from middleware.core.lerobot_export import apply_source_clock, synchronized_episodes
from middleware.core.recording_time import epoch_grid


def test_grid_is_independent_of_recorder_start_and_does_not_drift():
    base = 1_799_000_000_000_000_000
    whole = list(epoch_grid(base, base + 1_000_000_000, 30))
    assert len(whole) == 31 and whole[0] == base and whole[-1] == base + 1_000_000_000
    assert whole[1:4] == [base + 33_333_333, base + 66_666_667, base + 100_000_000]
    assert list(epoch_grid(whole[1] + 1, whole[-2], 30)) == whole[2:-1]


def test_live_grid_keeps_sparse_action_and_never_selects_a_future_measurement():
    base = 1_800_000_000_000_000_000
    streams = {'action': [], 'observation.state': [(base + 1, 'first'), (base + 50_000_001, 'second')]}
    audit = {}
    episode, = synchronized_episodes(streams, 20, audit, (base, base + 100_000_000), use_epoch_grid=True)
    rows = list(episode)
    assert [v['observation.state'] for _, v in rows] == [None, streams['observation.state'][0], streams['observation.state'][1]]
    assert all(v['action'] is None for _, v in rows)
    assert audit['invalid_by_stream']['action'] == 3


def test_bad_header_in_live_data_only_falls_back_for_that_sample():
    raw = [(10_000_000_123, 'a'), (10_100_000_456, 'b')]
    clock, values = apply_source_clock(raw, [10_000_000_000, None], live=True)
    assert clock == 'mixed'
    assert values == [(10_000_000_000, 'a'), raw[1]]
    assert apply_source_clock(raw, [10_000_000_000, None]) == ('log_time', raw)
