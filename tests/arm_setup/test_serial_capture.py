"""Read-only capture regressions. All ports and clocks are fake."""

import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import arm_serial_capture as capture


class FakeClock:
    def __init__(self):
        self.current = 0.0

    def monotonic(self):
        self.current += 0.001
        return self.current

    def sleep(self, duration):
        self.current += duration


class FakePort:
    def __init__(self, responses=(), stale=b""):
        self.responses = list(responses)
        self.pending = [stale] if stale else []
        self.writes = []
        self.resets = 0
        self.closed = False
        self.timeout = 10.0

    def reset_input_buffer(self):
        self.resets += 1
        self.pending.clear()

    def write(self, data):
        self.writes.append(data)
        response = self.responses.pop(0) if self.responses else []
        self.pending.extend([response] if isinstance(response, bytes) else response)
        return len(data)

    def read(self, size):
        chunk = self.pending.pop(0) if self.pending else b""
        if isinstance(chunk, Exception):
            raise chunk
        return chunk

    def close(self):
        self.closed = True


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.patches = [
            patch.object(capture.time, "monotonic", self.clock.monotonic),
            patch.object(capture.time, "sleep", self.clock.sleep),
        ]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)

    def read(self, port, servo_id=2):
        return capture.read_position(port, servo_id, timeout_s=0.04)

    def collect(self, port, ids=(2, 3), **kwargs):
        factory = unittest.mock.Mock(return_value=port)
        with patch.dict(sys.modules, {"serial": types.SimpleNamespace(Serial=factory)}):
            result = capture.capture_positions("FAKE", ids, timeout_s=0.04, **kwargs)
        self.assertEqual(factory.call_count, 1)
        return result

    def test_parse_split_echo_multiple_and_move_frame(self):
        data = b"noise#002PRAD!#003P1500!#002P0785!#002P1500T1000!#002P12"
        self.assertEqual(capture.parse_frames(data), [(3, 1500), (2, 785)])
        self.assertEqual(capture.parse_frames(b"#002P0000!"), [(2, 0)])

    def test_split_response_user_example(self):
        port = FakePort([[b"#002PRAD!", b"#002P0", b"785!"]])
        self.assertEqual(self.read(port), 785)
        self.assertEqual(port.writes, [b"#002PRAD!"])
        self.assertEqual(port.resets, 1)
        self.assertEqual(port.timeout, 10.0)

    def test_other_id_echo_and_junk_are_ignored(self):
        port = FakePort([[b"#003P1500!#002PRAD!\xff", b"#002P1500!"]])
        self.assertEqual(self.read(port), 1500)

    def test_fake_port_needs_no_timeout_attribute(self):
        port = FakePort([b"#002P0500!"])
        del port.timeout
        self.assertEqual(self.read(port), 500)

    def test_boundaries_are_valid(self):
        for value in (500, 2500):
            with self.subTest(value=value):
                self.assertEqual(self.read(FakePort([f"#002P{value:04d}!".encode()])), value)

    def test_out_of_range_never_becomes_zero(self):
        for value in (0, 499, 2501, 9999):
            with self.subTest(value=value):
                with self.assertRaisesRegex(capture.CaptureError, r"500\.\.2500.*raw HEX"):
                    self.read(FakePort([f"#002P{value:04d}!".encode()]))

    def test_conflicting_same_id_frames_fail(self):
        with self.assertRaisesRegex(capture.CaptureError, "conflicting"):
            self.read(FakePort([b"#002P1500!#002P1600!"]))

    def test_identical_same_id_frames_can_be_read(self):
        self.assertEqual(self.read(FakePort([b"#002P1500!#002P1500!"])), 1500)

    def test_no_response_and_echo_only_have_raw_diagnostics(self):
        for response in (b"", b"#002PRAD!", b"#003P1500!", b"#002P078"):
            with self.subTest(response=response):
                with self.assertRaises(capture.CaptureError) as context:
                    self.read(FakePort([response]))
                self.assertIn("timeout", str(context.exception))
                self.assertIn("raw HEX:", str(context.exception))
                self.assertEqual(context.exception.raw, response)

    def test_buffered_stale_data_is_discarded(self):
        with self.assertRaisesRegex(capture.CaptureError, "timeout") as context:
            self.read(FakePort(stale=b"#002P1500!"))
        self.assertEqual(context.exception.raw, b"")

    def test_response_buffer_and_diagnostic_output_are_bounded(self):
        with self.assertRaisesRegex(capture.CaptureError, "exceeded") as context:
            self.read(FakePort([[b"x" * 256] * 17]))
        self.assertEqual(len(context.exception.raw), 4096)
        self.assertLess(len(str(context.exception)), 600)
        self.assertIn("truncated", str(context.exception))

    def test_io_failure_retains_received_bytes(self):
        with self.assertRaises(capture.CaptureError) as context:
            self.read(FakePort([[b"partial", OSError("disconnected")]]))
        self.assertEqual(context.exception.raw, b"partial")
        self.assertIn("disconnected", str(context.exception))

    def test_short_write_fails(self):
        port = FakePort()
        port.write = lambda data: 1
        with self.assertRaisesRegex(capture.CaptureError, "incomplete query"):
            self.read(port)

    def test_two_complete_stable_rounds_returns_latest_and_closes(self):
        port = FakePort([b"#002P0785!", b"#003P1500!", b"#002P0787!", b"#003P1499!"])
        self.assertEqual(self.collect(port), {2: 787, 3: 1499})
        self.assertEqual(port.writes, [b"#002PRAD!", b"#003PRAD!", b"#002PRAD!", b"#003PRAD!"])
        self.assertEqual(port.resets, 4)
        self.assertTrue(port.closed)

    def test_unstable_complete_capture_fails_and_closes(self):
        port = FakePort([b"echo#002PRAD!#002P0785!", b"noise#002P0790!"])
        with self.assertRaisesRegex(capture.CaptureError, "Port FAKE.*unstable.*raw HEX") as context:
            self.collect(port, ids=(2,))
        self.assertEqual(context.exception.raw, b"echo#002PRAD!#002P0785!noise#002P0790!")
        self.assertTrue(port.closed)

    def test_incomplete_round_never_returns_partial_and_closes(self):
        port = FakePort([b"#002P0785!", b""])
        with self.assertRaisesRegex(capture.CaptureError, "Port FAKE.*003.*timeout"):
            self.collect(port)
        self.assertTrue(port.closed)
        self.assertEqual(len(port.writes), 2)

    def test_stability_uses_all_rounds_not_only_last_two(self):
        port = FakePort([b"#002P0780!", b"#002P0785!", b"#002P0786!"])
        with self.assertRaisesRegex(capture.CaptureError, "unstable"):
            self.collect(port, ids=(2,), samples=3)
        self.assertTrue(port.closed)

    def test_invalid_arguments_rejected_before_import_or_open(self):
        invalid = [
            {"ids": (2, 2)}, {"ids": ()}, {"ids": (255,)}, {"ids": (True,)},
            {"ids": ("002PRAD!",)}, {"ids": None}, {"samples": 1}, {"samples": True},
            {"tolerance": -1}, {"baud": 0}, {"timeout_s": float("inf")},
            {"timeout_s": float("nan")}, {"timeout_s": 0}, {"port_name": ""},
        ]
        with patch.object(capture, "_load_serial") as loader:
            for override in invalid:
                with self.subTest(override=override):
                    args = {"port_name": "FAKE", "ids": (2,)}
                    args.update(override)
                    with self.assertRaises(capture.CaptureError):
                        capture.capture_positions(**args)
            loader.assert_not_called()

    def test_missing_dependency_is_actionable(self):
        with patch.dict(sys.modules, {"serial": None}):
            with self.assertRaisesRegex(capture.CaptureError, "pyserial.*pip install.*raw HEX"):
                capture.capture_positions("FAKE", (2,))

    def test_open_error_contains_port_and_bounded_diagnostics(self):
        factory = unittest.mock.Mock(side_effect=OSError("access denied"))
        with patch.dict(sys.modules, {"serial": types.SimpleNamespace(Serial=factory)}):
            with self.assertRaisesRegex(capture.CaptureError, "Port FAKE.*access denied.*raw HEX"):
                capture.capture_positions("FAKE", (2,))

    def test_close_failure_prevents_success(self):
        port = FakePort([b"#002P0785!", b"#002P0785!"])
        port.close = unittest.mock.Mock(side_effect=OSError("close failed"))
        with self.assertRaisesRegex(capture.CaptureError, "close failed"):
            self.collect(port, ids=(2,))

    def test_list_ports_enumerates_without_open(self):
        factory = unittest.mock.Mock(side_effect=AssertionError("must not open"))
        tool = types.ModuleType("serial.tools.list_ports")
        tool.comports = lambda: [types.SimpleNamespace(device="FAKE", description="adapter", hwid="USB TEST")]
        tools = types.ModuleType("serial.tools")
        tools.list_ports = tool
        fake_serial = types.ModuleType("serial")
        fake_serial.Serial = factory
        with patch.dict(sys.modules, {"serial": fake_serial, "serial.tools": tools, "serial.tools.list_ports": tool}):
            self.assertEqual(capture.list_ports(), [{"device": "FAKE", "description": "adapter", "hwid": "USB TEST"}])
        factory.assert_not_called()


if __name__ == "__main__":
    unittest.main()
