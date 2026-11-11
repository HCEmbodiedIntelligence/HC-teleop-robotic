import unittest
from unittest.mock import patch

from tools.diagnostics.wifi_preflight import busy_rates, make_report, parse_survey, run_iw
from tools.diagnostics.qos_compare import compare, percentile, samples_from


class WifiPreflightTests(unittest.TestCase):
    def test_busy_uses_deltas_and_rejects_reset_or_absent_counter(self):
        before = {5200: {'active': 1000, 'busy': 400}, 5220: {'active': 1000, 'busy': 400}}
        after = {5200: {'active': 1200, 'busy': 450}, 5220: {'active': 50, 'busy': 10},
                 5240: {'active': 500, 'busy': 100}}
        rates = busy_rates(before, after)
        self.assertEqual(rates[5200]['busy_pct'], 25)
        self.assertIsNone(rates[5220]['busy_pct'])
        self.assertIsNone(rates[5240]['busy_pct'])

    def test_survey_parser_keeps_frequencies_separate(self):
        data = parse_survey('frequency: 5200 MHz [in use]\n channel active time: 150 ms\n channel busy time: 30 ms\nfrequency: 5220 MHz\n noise: -96 dBm')
        self.assertEqual(data[5200], {'active': 150, 'busy': 30})
        self.assertEqual(data[5220], {'noise_dbm': -96})

    def test_80mhz_group_and_dfs_candidates_not_automatically_recommended(self):
        phy = '\n'.join(f'* {5000 + 5 * c} MHz [{c}] (20.0 dBm)' + (' (radar detection)' if c >= 52 else '')
                        for c in (36, 40, 44, 48, 52, 56, 60, 64))
        scan = 'BSS aa:bb:cc:dd:ee:ff(on wlan0)\n freq: 5200\n signal: -45.0 dBm'
        report = make_report('', 'freq: 5200', phy, '', scan, '', '', 80, False, [])
        self.assertEqual(report['candidates'][0]['channels'], [36, 40, 44, 48])
        self.assertTrue(report['candidates'][0]['current_group'])
        self.assertTrue(report['candidates'][1]['dfs'])
        self.assertFalse(report['candidates'][1]['recommendation_eligible'])
        self.assertIsNone(report['current_airtime']['busy_pct'])
        self.assertEqual(report['suggested_trial_channels'], [36, 40, 44, 48])

    def test_disabled_and_no_ir_rejected(self):
        phy = '* 5180 MHz [36] (disabled)\n* 5200 MHz [40] (20 dBm) (no IR)'
        report = make_report('', '', phy, '', 'BSS x\n freq: 5180', '', '', 20, False, [])
        self.assertIsNone(report['suggested_trial_channels'])
        self.assertTrue(all(not c['recommendation_eligible'] for c in report['candidates']))

    def test_timeout_is_reported(self):
        import subprocess
        errors = []
        with patch('tools.diagnostics.wifi_preflight.subprocess.run', side_effect=subprocess.TimeoutExpired('iw', 12)):
            self.assertEqual(run_iw(['dev'], errors), '')
        self.assertEqual(len(errors), 1)


class QosCompareTests(unittest.TestCase):
    def test_missing_metrics_not_fabricated_and_counter_not_averaged(self):
        result = compare([{'webrtc_rtt_ms': 20, 'totalFreezesDuration': 6}],
                         [{'webrtc_rtt_ms': 12}, {'webrtc_rtt_ms': None}])['cameras']['main']
        self.assertEqual(result['delta_treatment_minus_baseline']['webrtc_rtt_ms']['mean'], -8)
        self.assertNotIn('totalFreezesDuration', result['baseline']['metrics'])
        self.assertFalse(result['fec']['changes_configuration'])
        self.assertEqual(result['fec']['decision'], 'insufficient_evidence')

    def test_loss_candidate_needs_enough_intervals(self):
        data = [{'webrtc_fraction_lost_pct': 1, 'webrtc_feedback_received': True}] * 30
        self.assertEqual(compare(data, data)['cameras']['main']['fec']['decision'], 'requires_packet_level_measurement')
        self.assertFalse(compare(data, data)['cameras']['main']['fec']['changes_configuration'])

    def test_camera_and_session_isolation(self):
        data = {'samples': [{'streams': [{'id': 'head', 'receiver_session_id': 'a', 'webrtc_rtt_ms': 5},
                                          {'id': 'wrist', 'webrtc_rtt_ms': 500}]}]}
        report = compare(data, data, camera_id='head')
        self.assertEqual(set(report['cameras']), {'head'})
        self.assertEqual(report['cameras']['head']['baseline']['metrics']['webrtc_rtt_ms']['mean'], 5)
        mixed = [{'id': 'head', 'receiver_session_id': 'a'}, {'id': 'head', 'receiver_session_id': 'b'}]
        with self.assertRaises(ValueError):
            compare(mixed, mixed)
        self.assertIn('head', compare(mixed, mixed, baseline_session='a', treatment_session='b')['cameras'])

    def test_wrapped_samples_and_invalid_input(self):
        self.assertEqual(samples_from({'samples': [{'camera': {'webrtc_rtt_ms': 3}}]}), [{'webrtc_rtt_ms': 3}])
        with self.assertRaises(ValueError):
            samples_from({'samples': 'bad'})
        self.assertEqual(percentile([0, 100], .95), 95)


if __name__ == '__main__':
    unittest.main()
