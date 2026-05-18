#!/usr/bin/env python3
"""Iterate a Techlaser OPU over azimuth using the service TCP protocol.

The script uses the service protocol documented for similar Techlaser Ethernet
OPU units. Default control port is 9760.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time


class TechlaserOPU:
    def __init__(self, host: str, port: int = 9760, timeout: float = 2.0) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: socket.socket | None = None
        self._buffer = b""

    def __enter__(self) -> "TechlaserOPU":
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def _read_frame(self) -> str:
        if self.sock is None:
            raise RuntimeError("Not connected")

        while b"#" not in self._buffer:
            chunk = self.sock.recv(1024)
            if not chunk:
                raise ConnectionError("Connection closed by device")
            self._buffer += chunk

        frame, self._buffer = self._buffer.split(b"#", 1)
        return (frame + b"#").decode("ascii", errors="replace").strip()

    def send(self, text: str) -> None:
        if self.sock is None:
            raise RuntimeError("Not connected")
        if not text.startswith("$") or not text.endswith("#"):
            raise ValueError("Command must look like '$...#'")

        self.sock.sendall(text.encode("ascii"))

    def command(self, text: str) -> str:
        if self.sock is None:
            raise RuntimeError("Not connected")
        if not text.startswith("$") or not text.endswith("#"):
            raise ValueError("Command must look like '$...#'")

        expected_prefix = text[1:].split(",", 1)[0].rstrip("#")
        self.sock.sendall(text.encode("ascii"))

        deadline = time.monotonic() + self.timeout
        skipped: list[str] = []
        while time.monotonic() < deadline:
            try:
                frame = self._read_frame()
            except socket.timeout as exc:
                raise TimeoutError(
                    f"Timed out waiting for response to {text}; skipped frames: {skipped}"
                ) from exc
            frame_prefix = frame.strip("$#").split(",", 1)[0]
            if frame_prefix in (expected_prefix, "X"):
                return frame
            skipped.append(frame)

        raise TimeoutError(
            f"Timed out waiting for response to {text}; skipped frames: {skipped}"
        )

    def get_axis_state(self) -> int:
        response = self.command("$a#")
        return int(response.strip("$#").split(",")[1])

    def start_axis_self_test(self) -> None:
        self.send("$a,1#")

    def get_axis_faults(self) -> str:
        response = self.command("$b#")
        return response.strip("$#").split(",")[1]

    def get_position(self) -> float:
        response = self.command("$c#")
        return float(response.strip("$#").split(",")[1])

    def get_busy_status(self) -> int:
        response = self.command("$e#")
        return int(response.strip("$#").split(",")[1])

    def stop(self) -> None:
        self.command("$g#")

    def goto_azimuth(self, degrees: float, max_speed: float) -> str:
        return self.command(f"$j,{degrees:.2f},{max_speed:.2f}#")


def generate_angles(start: float, stop: float, step: float) -> list[float]:
    if step == 0:
        raise ValueError("step must not be zero")
    if start < stop and step < 0:
        raise ValueError("step must be positive when start < stop")
    if start > stop and step > 0:
        raise ValueError("step must be negative when start > stop")

    angles: list[float] = []
    value = start
    if step > 0:
        while value <= stop + 1e-9:
            angles.append(value % 360)
            value += step
    else:
        while value >= stop - 1e-9:
            angles.append(value % 360)
            value += step
    return angles


def angular_error(current: float, target: float) -> float:
    return abs((current - target + 180) % 360 - 180)


def wait_until_position(
    opu: TechlaserOPU,
    target: float,
    tolerance: float,
    poll_interval: float,
    max_wait: float,
) -> float:
    deadline = time.monotonic() + max_wait
    last_position = opu.get_position()

    while time.monotonic() < deadline:
        last_position = opu.get_position()
        busy = opu.get_busy_status()
        if angular_error(last_position, target) <= tolerance and busy in (0, 1):
            return last_position
        time.sleep(poll_interval)

    raise TimeoutError(
        f"Timed out waiting for {target:.2f} deg; last position {last_position:.2f} deg"
    )


def wait_until_ready(
    opu: TechlaserOPU,
    poll_interval: float,
    max_wait: float,
) -> int:
    deadline = time.monotonic() + max_wait
    last_state = opu.get_axis_state()

    while time.monotonic() < deadline:
        last_state = opu.get_axis_state()
        if last_state == 2:
            return last_state
        time.sleep(poll_interval)

    return last_state


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move a Techlaser Ethernet OPU through azimuth positions."
    )
    parser.add_argument("host", help="OPU IP address, for example 192.168.1.115")
    parser.add_argument("--port", type=int, default=9760, help="service TCP port")
    parser.add_argument("--start", type=float, default=0.0, help="start azimuth in degrees")
    parser.add_argument("--stop", type=float, default=359.0, help="stop azimuth in degrees")
    parser.add_argument("--step", type=float, default=10.0, help="azimuth step in degrees")
    parser.add_argument("--speed", type=float, default=5.0, help="max movement speed, deg/s")
    parser.add_argument("--dwell", type=float, default=1.0, help="pause after each position, sec")
    parser.add_argument("--tolerance", type=float, default=0.2, help="position tolerance, deg")
    parser.add_argument("--poll", type=float, default=0.2, help="position polling interval, sec")
    parser.add_argument("--timeout", type=float, default=2.0, help="TCP response timeout, sec")
    parser.add_argument("--max-wait", type=float, default=60.0, help="max wait per position, sec")
    parser.add_argument(
        "--init",
        action="store_true",
        help="start axis self-test before movement and wait until the axis is ready",
    )
    parser.add_argument(
        "--init-only",
        action="store_true",
        help="start axis self-test, wait until ready, and exit without movement",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print target azimuths without connecting or moving",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    angles = generate_angles(args.start, args.stop, args.step)

    if args.dry_run:
        print("Angles:", ", ".join(f"{angle:.2f}" for angle in angles))
        return 0

    with TechlaserOPU(args.host, args.port, timeout=args.timeout) as opu:
        state = opu.get_axis_state()
        print(f"Axis state: {state}")

        if args.init or args.init_only:
            print("Starting axis self-test...")
            opu.start_axis_self_test()
            print("Self-test command sent.")
            state = wait_until_ready(opu, poll_interval=args.poll, max_wait=args.max_wait)
            print(f"Axis state after self-test: {state}")

            if args.init_only:
                if state == 2:
                    print("Axis is ready.")
                    return 0
                print(f"Axis is not ready, state={state}, faults={opu.get_axis_faults()}")
                return 2

        if state != 2:
            print(
                f"Axis is not ready, state={state}, faults={opu.get_axis_faults()}. "
                "Run again with --init if it is safe to start axis self-test."
            )
            return 2

        try:
            for angle in angles:
                print(f"Moving to {angle:.2f} deg at <= {args.speed:.2f} deg/s")
                print("Device:", opu.goto_azimuth(angle, args.speed))
                position = wait_until_position(
                    opu,
                    target=angle,
                    tolerance=args.tolerance,
                    poll_interval=args.poll,
                    max_wait=args.max_wait,
                )
                print(f"Reached {position:.2f} deg; dwell {args.dwell:.2f}s")
                time.sleep(args.dwell)
        except KeyboardInterrupt:
            print("\nInterrupted, sending stop command...", file=sys.stderr)
            opu.stop()
            return 130
        except Exception:
            try:
                opu.stop()
            finally:
                raise

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
