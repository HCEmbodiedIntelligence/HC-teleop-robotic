from hc_dashboard.state import DashboardModel, StreamMetric


def test_stream_metric_reports_rate_age_and_staleness():
    metric = StreamMetric(window_seconds=2.0, stale_seconds=0.25)
    metric.observe(10.0)
    metric.observe(10.1)
    metric.observe(10.2)
    active = metric.snapshot(10.2)
    assert active["hz"] == 10.0
    assert active["age_ms"] == 0.0
    assert not active["stale"]
    assert metric.snapshot(10.5)["stale"]


def test_dashboard_model_keeps_independent_command_groups_and_copies_payloads():
    model = DashboardModel("x1", "x1", "sim")
    left = {"positions": [0.1]}
    model.observe_group("commands", "left_arm", left)
    model.observe_group("commands", "right_arm", {"positions": [-0.2]})
    left["positions"][0] = 9.0
    snapshot = model.snapshot()
    assert snapshot["robot_id"] == "x1"
    assert snapshot["commands"]["left_arm"]["positions"] == [0.1]
    assert snapshot["commands"]["right_arm"]["positions"] == [-0.2]
    assert snapshot["streams"]["commands"]["total"] == 2
