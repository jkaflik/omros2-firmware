#!/usr/bin/env python3
"""Interactive hardware-in-the-loop emergency test.

This test expects real OpenMower hardware, flashed firmware, and a running
micro-ROS agent. It guides the operator through physical emergency inputs and
verifies the ROS 2 emergency API exposed by the firmware.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8


class TestFailure(RuntimeError):
    pass


@dataclass
class EmergencySnapshot:
    active: Optional[bool] = None
    stop_active: Optional[bool] = None
    lift_active: Optional[bool] = None
    tilt_active: Optional[bool] = None
    software_requested: Optional[bool] = None
    release_blocked: Optional[bool] = None
    lifted_wheels: Optional[int] = None

    def is_complete(self) -> bool:
        return all(value is not None for value in self.__dict__.values())

    def physical_inputs_clear(self) -> bool:
        return (
            self.stop_active is False
            and self.lift_active is False
            and self.tilt_active is False
            and self.lifted_wheels == 0
        )

    def format(self) -> str:
        return (
            f"active={self.active} "
            f"stop={self.stop_active} "
            f"lift={self.lift_active} "
            f"tilt={self.tilt_active} "
            f"software={self.software_requested} "
            f"release_blocked={self.release_blocked} "
            f"lifted_wheels={self.lifted_wheels}"
        )


class EmergencyProbe(Node):
    def __init__(self, command_interval: float):
        super().__init__("emergency_hil_test")
        self.snapshot = EmergencySnapshot()
        self.command_interval = command_interval
        self._subscriptions = []
        self.command_publisher = self.create_publisher(Bool, "emergency/command", 10)

        self._subscribe_bool("emergency/active", "active")
        self._subscribe_bool("emergency/stop_active", "stop_active")
        self._subscribe_bool("emergency/lift_active", "lift_active")
        self._subscribe_bool("emergency/tilt_active", "tilt_active")
        self._subscribe_bool("emergency/software_requested", "software_requested")
        self._subscribe_bool("emergency/release_blocked", "release_blocked")
        self._subscriptions.append(
            self.create_subscription(UInt8, "emergency/lifted_wheels", self._lifted_wheels_cb, 10)
        )

    def _subscribe_bool(self, topic: str, field: str) -> None:
        self._subscriptions.append(
            self.create_subscription(Bool, topic, lambda msg, field=field: self._set(field, msg.data), 10)
        )

    def _lifted_wheels_cb(self, msg: UInt8) -> None:
        self.snapshot.lifted_wheels = int(msg.data)

    def _set(self, field: str, value: bool) -> None:
        setattr(self.snapshot, field, bool(value))

    def spin_for(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(
        self,
        predicate: Callable[[EmergencySnapshot], bool],
        timeout: float,
        description: str,
    ) -> EmergencySnapshot:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate(self.snapshot):
                print(f"PASS: {description}")
                print(f"      {self.snapshot.format()}")
                return self.snapshot

        raise TestFailure(f"Timeout waiting for {description}. Last state: {self.snapshot.format()}")

    def wait_for_initial_messages(self, timeout: float) -> None:
        self.wait_for(lambda state: state.is_complete(), timeout, "all emergency topics")

    def wait_for_command_subscriber(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.command_publisher.get_subscription_count() > 0:
                print("PASS: emergency command subscriber discovered")
                return

        raise TestFailure("No subscriber discovered for emergency/command")

    def send_command(self, value: bool, count: int = 3) -> None:
        msg = Bool()
        msg.data = value
        print(f"Publishing emergency/command={value} {count} time(s)")
        for _ in range(count):
            self.command_publisher.publish(msg)
            self.spin_for(self.command_interval)

    def release_latch(self, timeout: float) -> None:
        self.send_command(False)
        self.wait_for(lambda state: state.active is False, timeout, "emergency latch released")

    def request_latch(self, timeout: float) -> None:
        self.send_command(True)
        self.wait_for(lambda state: state.active is True, timeout, "emergency latch active")


def prompt(message: str) -> None:
    input(f"\nACTION: {message}\nPress Enter when ready. ")


def print_section(title: str) -> None:
    print(f"\n=== {title} ===")


def wait_for_clear_physical_inputs(probe: EmergencyProbe, timeout: float) -> None:
    probe.wait_for(
        lambda state: state.physical_inputs_clear(),
        timeout,
        "all physical emergency inputs clear",
    )


def test_initial_latch(probe: EmergencyProbe, timeout: float) -> None:
    print_section("Initial latch")
    probe.wait_for(lambda state: state.active is True, timeout, "emergency latch active after boot")


def test_command_confirmation(probe: EmergencyProbe, timeout: float) -> None:
    print_section("Command confirmation")
    wait_for_clear_physical_inputs(probe, timeout)

    probe.wait_for(lambda state: state.active is True, timeout, "emergency latch active before release")
    probe.send_command(False, count=2)
    probe.spin_for(0.3)
    if probe.snapshot.active is not True:
        raise TestFailure("Emergency latch released after only two release commands")
    print("PASS: two release commands are not enough")

    probe.send_command(False, count=1)
    probe.wait_for(lambda state: state.active is False, timeout, "third release command accepted")


def test_command_window_timeout(probe: EmergencyProbe, timeout: float) -> None:
    print_section("Command window timeout")
    wait_for_clear_physical_inputs(probe, timeout)
    if probe.snapshot.active is True:
        probe.release_latch(timeout)

    probe.send_command(True, count=2)
    probe.spin_for(1.2)
    probe.send_command(True, count=1)
    probe.spin_for(0.3)
    if probe.snapshot.active is not False:
        raise TestFailure("Emergency latch accepted commands outside the one second window")
    print("PASS: command confirmation window expires")


def test_software_latch(probe: EmergencyProbe, timeout: float) -> None:
    print_section("Software latch")
    wait_for_clear_physical_inputs(probe, timeout)

    probe.request_latch(timeout)
    probe.wait_for(
        lambda state: state.software_requested is True,
        timeout,
        "software emergency request reported",
    )

    probe.release_latch(timeout)
    probe.wait_for(
        lambda state: state.software_requested is False,
        timeout,
        "software emergency request cleared",
    )


def test_stop_input(probe: EmergencyProbe, timeout: float) -> None:
    print_section("STOP input")
    wait_for_clear_physical_inputs(probe, timeout)
    if probe.snapshot.active is True:
        probe.release_latch(timeout)

    prompt("Press and hold STOP")
    probe.wait_for(
        lambda state: state.stop_active is True and state.active is True,
        timeout,
        "STOP input latches emergency",
    )

    probe.send_command(False)
    probe.wait_for(
        lambda state: state.active is True and state.release_blocked is True,
        timeout,
        "release blocked while STOP is held",
    )

    prompt("Release STOP")
    probe.wait_for(
        lambda state: state.stop_active is False and state.release_blocked is False,
        timeout,
        "STOP input released",
    )
    probe.release_latch(timeout)


def test_single_lift_input(probe: EmergencyProbe, timeout: float) -> None:
    print_section("Single lift / tilt input")
    wait_for_clear_physical_inputs(probe, timeout)
    if probe.snapshot.active is True:
        probe.release_latch(timeout)

    prompt("Activate exactly one lift sensor and hold it")
    probe.wait_for(
        lambda state: state.lifted_wheels == 1,
        timeout,
        "one lift sensor detected",
    )
    probe.wait_for(
        lambda state: state.tilt_active is True and state.active is True,
        max(timeout, 4.0),
        "single lift becomes tilt emergency",
    )

    prompt("Release the lift sensor")
    wait_for_clear_physical_inputs(probe, timeout)
    probe.release_latch(timeout)


def test_double_lift_input(probe: EmergencyProbe, timeout: float) -> None:
    print_section("Double lift input")
    wait_for_clear_physical_inputs(probe, timeout)
    if probe.snapshot.active is True:
        probe.release_latch(timeout)

    prompt("Activate two lift sensors at the same time and hold them")
    probe.wait_for(
        lambda state: state.lifted_wheels == 2,
        timeout,
        "two lift sensors detected",
    )
    probe.wait_for(
        lambda state: state.lift_active is True and state.active is True,
        timeout,
        "double lift latches emergency",
    )

    prompt("Release both lift sensors")
    wait_for_clear_physical_inputs(probe, timeout)
    probe.release_latch(timeout)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=10.0, help="Timeout for each wait step")
    parser.add_argument(
        "--command-interval",
        type=float,
        default=0.2,
        help="Interval between repeated emergency commands",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print("Interactive OpenMower emergency HIL test")
    print("Prerequisites:")
    print("- Firmware from this branch is flashed on the OpenMower mainboard.")
    print("- micro-ROS agent is running and connected to the firmware.")
    print("- The mower is physically safe to handle.")
    print("- You can press STOP and activate one or two lift sensors on demand.")
    prompt("Confirm prerequisites")

    rclpy.init()
    probe = EmergencyProbe(args.command_interval)

    try:
        print_section("ROS connectivity")
        probe.wait_for_initial_messages(args.timeout)
        probe.wait_for_command_subscriber(args.timeout)

        test_initial_latch(probe, args.timeout)
        test_command_confirmation(probe, args.timeout)
        test_command_window_timeout(probe, args.timeout)
        test_software_latch(probe, args.timeout)
        test_stop_input(probe, args.timeout)
        test_single_lift_input(probe, args.timeout)
        test_double_lift_input(probe, args.timeout)

        print("\nAll emergency HIL checks passed.")
        return 0
    except (KeyboardInterrupt, EOFError):
        print("\nTest interrupted.")
        return 130
    except TestFailure as exc:
        print(f"\nFAIL: {exc}")
        return 1
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
