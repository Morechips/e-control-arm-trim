"""Read-only ZL bus-servo pose capture; no motion or torque commands.

Protocol sources (local official manuals):
* ``D:/工科大/众灵舵机使用手册-250508.pdf``, pp. 25-26: three-digit
  ID, four-digit PWM equivalent, ``#000PRAD!`` -> ``#000P1500!``.
  The user's confirmed ID-2 response follows the same form: ``#002P0785!``.
* ``D:/工科大/012-24路舵机控制器/001-文档教程/24路舵机控制器使用手册-20241230.pdf``,
  p. 14: position readback is available only for bus servos; pp. 39-40:
  manual teaching uses a bus-servo arm.

Ordinary Analog/PWM servos provide no physical position feedback on their
PWM connection. This module cannot read a hand-moved Analog arm or turn a
controller's commanded pulse width into a measured angle. A missing response
is an error, never position zero. The module does not change controller mode,
release torque, reset a board, move a joint, or save incomplete captures.

The protocol has no transaction number. Clearing input before each query and
sampling complete rounds rejects buffered old data and unstable poses, but
cannot prove that a same-ID response delayed until after a new query is fresh.
Use an exclusive serial connection without another program issuing commands.
"""

from __future__ import annotations

import math
import re
import time
from typing import Any, Dict, Iterable, List, Tuple


MAX_RESPONSE_BYTES = 4096
MAX_ERROR_BYTES = 128
READ_SLICE_S = 0.05
_FRAME_PATTERN = re.compile(rb"#([0-9]{3})P([0-9]{4})!")


class CaptureError(RuntimeError):
    """A complete capture failed; ``raw`` is bounded received data.

    ``raw_hex`` and the exception text contain at most 128 received bytes.
    This preserves useful diagnostics without printing unlimited serial input.
    """

    def __init__(self, reason: str, raw: bytes = b"") -> None:
        self.reason = str(reason)
        self.raw = bytes(raw[:MAX_RESPONSE_BYTES])
        self.raw_hex = self.raw[:MAX_ERROR_BYTES].hex(" ").upper() or "<empty>"
        suffix = " ... (truncated)" if len(self.raw) > MAX_ERROR_BYTES else ""
        super().__init__(f"{self.reason}; raw HEX: {self.raw_hex}{suffix}")


def parse_frames(data: bytes) -> List[Tuple[int, int]]:
    """Extract complete ``#NNNPdddd!`` frames in their wire order.

    Query echoes, movement commands, partial frames and unrelated text are
    ignored. Numeric range validation belongs to ``read_position`` so a
    malformed value for the requested ID raises an explicit error.
    """
    return [(int(match[1]), int(match[2])) for match in _FRAME_PATTERN.finditer(data)]


def _validate_id(servo_id: int) -> None:
    if type(servo_id) is not int or not 0 <= servo_id <= 254:
        raise CaptureError("Servo ID must be an integer from 0 to 254")


def _validate_timeout(timeout_s: float) -> float:
    if isinstance(timeout_s, bool) or not isinstance(timeout_s, (float, int)):
        raise CaptureError("Timeout must be a finite positive number of seconds")
    value = float(timeout_s)
    if not math.isfinite(value) or value <= 0:
        raise CaptureError("Timeout must be a finite positive number of seconds")
    return value


def _exception_detail(exc: Exception) -> str:
    return f"{type(exc).__name__}: {str(exc)[:200]}"


def read_position(port: Any, id: int, timeout_s: float = 1.0) -> int:
    """Query one ID through a serial-like object, without importing pyserial.

    ``port`` must offer ``reset_input_buffer()``, ``write(bytes)`` and
    ``read(size)``. Reads must be finite; a pyserial-style ``timeout`` attribute,
    when present, is temporarily bounded to 50 ms. Other fake ports need no
    timeout attribute. Only the fixed numeric-ID PRAD query is transmitted.
    """
    return _query_position(port, id, timeout_s)[0]


def _query_position(port: Any, servo_id: int, timeout_s: float) -> Tuple[int, bytes]:
    """Keep the actual received bytes for whole-capture diagnostics."""
    _validate_id(servo_id)
    timeout = _validate_timeout(timeout_s)
    query = f"#{servo_id:03d}PRAD!".encode("ascii")
    raw = bytearray()
    previous_timeout = None
    changed_timeout = False
    try:
        if hasattr(port, "timeout"):
            previous_timeout = port.timeout
            port.timeout = min(timeout, READ_SLICE_S)
            changed_timeout = True
        port.reset_input_buffer()
        written = port.write(query)
        if written is not None and written != len(query):
            raise CaptureError(f"ID {servo_id:03d}: incomplete query write", bytes(raw))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = port.read(256)
            if not isinstance(chunk, (bytes, bytearray)):
                raise CaptureError(f"ID {servo_id:03d}: read returned non-byte data", bytes(raw))
            if len(raw) + len(chunk) > MAX_RESPONSE_BYTES:
                raw.extend(chunk[:MAX_RESPONSE_BYTES - len(raw)])
                raise CaptureError(
                    f"ID {servo_id:03d}: response exceeded {MAX_RESPONSE_BYTES} bytes", bytes(raw)
                )
            raw.extend(chunk)
            if time.monotonic() >= deadline:
                break
            values = [value for frame_id, value in parse_frames(raw) if frame_id == servo_id]
            if values:
                if any(not 500 <= value <= 2500 for value in values):
                    raise CaptureError(
                        f"ID {servo_id:03d}: position outside 500..2500", bytes(raw)
                    )
                if len(set(values)) != 1:
                    raise CaptureError(
                        f"ID {servo_id:03d}: conflicting response frames", bytes(raw)
                    )
                return values[0], bytes(raw)
            if not chunk:
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    time.sleep(min(0.005, remaining))
        raise CaptureError(f"ID {servo_id:03d}: no valid response before timeout", bytes(raw))
    except CaptureError:
        raise
    except Exception as exc:
        raise CaptureError(
            f"ID {servo_id:03d}: serial I/O failed ({_exception_detail(exc)})", bytes(raw)
        ) from exc
    finally:
        if changed_timeout:
            try:
                port.timeout = previous_timeout
            except Exception:
                # The connection may have disappeared; capture_positions still
                # closes it, and its read/write exception carries the raw data.
                pass


def _load_serial() -> Any:
    # Delayed import keeps parsing and fake-port tests standard-library-only.
    try:
        import serial
    except ImportError as exc:
        raise CaptureError("pyserial is required for serial capture (python -m pip install pyserial)") from exc
    return serial


def capture_positions(
    port_name: str,
    ids: Iterable[int],
    baud: int = 115200,
    timeout_s: float = 1.0,
    samples: int = 2,
    tolerance: int = 2,
) -> Dict[int, int]:
    """Open once, read full stable rounds, close, then return the latest round.

    Every requested joint must respond in every round. The maximum spread for
    each ID must be <= ``tolerance`` in PWM equivalents. Any open/read/close or
    stability error fails the entire call; no partial dictionary is returned.
    Opening a serial port may change adapter control-line states, but no board
    reset command is sent. This API writes no files.
    """
    if not isinstance(port_name, str) or not port_name.strip():
        raise CaptureError("A non-empty serial port name is required")
    timeout = _validate_timeout(timeout_s)
    if type(baud) is not int or baud <= 0:
        raise CaptureError("Baud must be a positive integer")
    if type(samples) is not int or not 2 <= samples <= 100:
        raise CaptureError("Samples must be an integer from 2 to 100")
    if type(tolerance) is not int or not 0 <= tolerance <= 2000:
        raise CaptureError("Tolerance must be an integer from 0 to 2000")
    try:
        servo_ids = tuple(ids)
    except TypeError as exc:
        raise CaptureError("IDs must be a non-empty iterable of integers") from exc
    if not servo_ids:
        raise CaptureError("At least one servo ID is required")
    for servo_id in servo_ids:
        _validate_id(servo_id)
    if len(set(servo_ids)) != len(servo_ids):
        raise CaptureError("Duplicate servo IDs are not allowed")

    serial = _load_serial()
    port = None
    rounds = []
    received = {servo_id: bytearray() for servo_id in servo_ids}
    failure = None
    try:
        port = serial.Serial(
            port=port_name,
            baudrate=baud,
            timeout=min(timeout, READ_SLICE_S),
            write_timeout=timeout,
        )
        for _ in range(samples):
            round_values = {}
            for servo_id in servo_ids:
                value, raw = _query_position(port, servo_id, timeout)
                round_values[servo_id] = value
                room = MAX_RESPONSE_BYTES - len(received[servo_id])
                received[servo_id].extend(raw[:room])
            rounds.append(round_values)
        for servo_id in servo_ids:
            values = [round_values[servo_id] for round_values in rounds]
            if max(values) - min(values) > tolerance:
                raise CaptureError(
                    f"ID {servo_id:03d}: unstable samples {values} exceed tolerance {tolerance}",
                    bytes(received[servo_id]),
                )
    except CaptureError as exc:
        failure = CaptureError(f"Port {port_name}: {exc.reason}", exc.raw)
    except Exception as exc:
        failure = CaptureError(f"Port {port_name}: serial capture failed ({_exception_detail(exc)})")
    finally:
        if port is not None:
            try:
                port.close()
            except Exception as exc:
                if failure is None:
                    raw = b"".join(bytes(received[servo_id]) for servo_id in servo_ids)[:MAX_RESPONSE_BYTES]
                    failure = CaptureError(f"Port {port_name}: close failed ({_exception_detail(exc)})", raw)
    if failure is not None:
        raise failure
    return dict(rounds[-1])


def list_ports() -> List[Dict[str, str]]:
    """Enumerate serial-port metadata without opening any port."""
    _load_serial()
    try:
        from serial.tools import list_ports as serial_ports

        return [
            {"device": port.device, "description": port.description or "", "hwid": port.hwid or ""}
            for port in serial_ports.comports()
        ]
    except Exception as exc:
        raise CaptureError(f"Serial-port enumeration failed ({_exception_detail(exc)})") from exc
