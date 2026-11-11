import json
import unittest
from tools.diagnostics.check_jetson_encoder import parse_probe_output


class ProbeOutputTests(unittest.TestCase):
    def result(self, passed=False):
        return {'backend':'jetson','hardware_available':passed,'state':'passed' if passed else 'unavailable','error':'encoder timeout'}

    def test_native_stdout_after_json_does_not_hide_result(self):
        result = self.result()
        parsed = parse_probe_output('NvMMLiteOpen\n'+json.dumps(result)+'\nOpening in BLOCKING MODE\n', 'driver error', 1)
        self.assertEqual(parsed['state'], 'unavailable')
        self.assertEqual(parsed['error'], 'encoder timeout')
        self.assertIn('Opening in BLOCKING MODE', parsed['native_log_tail'])
        self.assertIn('driver error', parsed['native_log_tail'])

    def test_success_survives_late_native_log(self):
        parsed = parse_probe_output(json.dumps(self.result(True))+'\nNVIDIA cleanup\n')
        self.assertTrue(parsed['hardware_available'])

    def test_native_failure_without_json_retains_diagnostic(self):
        parsed = parse_probe_output('BlockError', 'could not initialize', -11)
        self.assertEqual(parsed['state'], 'failed')
        self.assertEqual(parsed['returncode'], -11)
        self.assertIn('BlockError', parsed['native_log_tail'])

    def test_crash_after_success_is_not_reported_as_available(self):
        parsed = parse_probe_output(json.dumps(self.result(True)), 'native crash', -11)
        self.assertFalse(parsed['hardware_available'])
        self.assertEqual(parsed['state'], 'failed')
