import unittest
from middleware.core.receiver_metrics import ReceiverMetrics


class ReceiverMetricsTests(unittest.TestCase):
    def setUp(self):
        self.now = [0.0]
        self.metrics = ReceiverMetrics(clock=lambda: self.now[0])
        self.metrics.reset('current')

    def report(self, sequence, **kwargs):
        report = dict(session_id='current', sequence=sequence, sample_ms=sequence*1000,
                      stats_id='video', packets_received=sequence*95, packets_lost=sequence*5,
                      frames_received=sequence*30, frames_decoded=sequence*29, frames_rendered=sequence*28,
                      jitter_buffer_delay_ms=sequence*290, jitter_buffer_emitted_count=sequence*29,
                      decode_time_ms=sequence*58, freeze_count=sequence, freeze_duration_ms=sequence*100)
        report.update(kwargs)
        return report

    def test_counter_deltas_rates_and_client_frame_quantiles(self):
        self.metrics.ingest(self.report(0))
        self.now[0] = 1
        self.metrics.ingest(self.report(1, render_gap_ms=[10, 20, 30], receive_to_render_ms=[15]))
        status = self.metrics.snapshot()
        self.assertEqual(status['receiver_received_fps'], 30)
        self.assertEqual(status['receiver_decoded_fps'], 29)
        self.assertEqual(status['receiver_render_fps'], 28)
        self.assertEqual(status['receiver_loss_pct'], 5)
        self.assertEqual(status['receiver_jitter_buffer_ms'], 10)
        self.assertEqual(status['receiver_decode_ms'], 2)
        self.assertEqual(status['receiver_freeze_count'], 1)
        self.assertEqual(status['receiver_render_gap_p50_ms'], 20)
        self.assertEqual(status['receiver_render_gap_p99_ms'], 30)
        self.assertIsNone(status['receiver_end_to_end_ms'])

    def test_session_sequence_and_expiry(self):
        with self.assertRaises(ValueError): self.metrics.ingest(self.report(0, session_id='old'))
        self.metrics.ingest(self.report(0))
        self.assertFalse(self.metrics.ingest(self.report(0)))
        self.now[0] = 6
        self.assertTrue(self.metrics.snapshot()['receiver_stale'])
        self.assertIsNone(self.metrics.snapshot()['receiver_loss_pct'])
        self.metrics.reset('new')
        self.assertFalse(self.metrics.snapshot()['receiver_reported'])

    def test_reset_counters_and_ssrc_do_not_create_negative_rates(self):
        self.metrics.ingest(self.report(2))
        self.metrics.ingest(self.report(3, stats_id='replacement', frames_decoded=0))
        self.assertIsNone(self.metrics.snapshot()['receiver_decoded_fps'])
        self.metrics.ingest(self.report(4, stats_id='replacement', frames_decoded=30))
        self.assertEqual(self.metrics.snapshot()['receiver_decoded_fps'], 30)

    def test_reject_nonfinite_oversized_boolean_and_regressing_clock(self):
        for changes in ({'render_gap_ms':[1]*241}, {'decode_time_ms':float('nan')},
                        {'frames_decoded':True}, {'sequence':False}, {'stats_id':'x'*129}):
            with self.subTest(changes=list(changes)):
                with self.assertRaises(ValueError): self.metrics.ingest({**self.report(0), **changes})
        self.metrics.ingest(self.report(1))
        with self.assertRaises(ValueError): self.metrics.ingest(self.report(2, sample_ms=1000))

    def test_fec_native_counters_are_deltas_not_recovery_claims(self):
        self.metrics.ingest(self.report(0, fec_packets_received=10, fec_packets_discarded=2))
        self.metrics.ingest(self.report(1, fec_packets_received=14, fec_packets_discarded=3))
        result = self.metrics.snapshot()
        self.assertEqual(result['receiver_fec_packets_received'], 4)
        self.assertEqual(result['receiver_fec_packets_discarded'], 1)
        self.assertNotIn('receiver_fec_packets_recovered', result)

    def test_storage_is_bounded_and_missing_values_remain_unknown(self):
        for n in range(200): self.metrics.ingest(self.report(n))
        self.assertEqual(len(self.metrics.samples), 120)
        self.metrics.reset('current')
        self.metrics.ingest(dict(session_id='current', sequence=0, sample_ms=0, stats_id='video'))
        self.assertIsNone(self.metrics.snapshot()['receiver_freeze_count'])
        self.assertIsNone(self.metrics.snapshot()['receiver_loss_pct'])


if __name__ == '__main__': unittest.main()
