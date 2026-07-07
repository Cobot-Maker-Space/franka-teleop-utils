import pathlib
import random
import socket
import struct
import sys
import argparse
import termios
import threading
import time
import tty

from ipaddress import IPv4Address
from typing import Optional

import capnp

capnp.remove_import_hook()
robotstate_capnp = capnp.load("robot-state.capnp")

VINCENT_HOST = "224.3.29.71"
VINCENT_PORT = 49185
VINCENT_FEEDBACK_PORT = 49187
BOB_HOST = "224.3.29.71"
BOB_PORT = 49186
BOB_FEEDBACK_PORT = 49188
MESSAGE_SIZE = 248
#BASE_PATH = "/Users/pszdp1/Library/CloudStorage/OneDrive-TheUniversityofNottingham/Development/embrace-angels/eapy/recordings"
BASE_PATH = "recordings"


class PauseController:
    def __init__(self, enabled: bool = True, hold_rate: float = 20.0):
        self.enabled = enabled and sys.stdin.isatty()
        self.hold_rate = hold_rate
        self.paused = threading.Event()
        self.stop_requested = threading.Event()
        self._thread = None
        self._terminal_settings = None

    def start(self):
        if not self.enabled or self._thread is not None:
            return

        self._terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()
        print("Press SPACE to pause/resume playback.", flush=True)

    def stop(self):
        self.stop_requested.set()
        if self.enabled and self._terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._terminal_settings)

    def _listen(self):
        while not self.stop_requested.is_set():
            char = sys.stdin.read(1)
            if char == " ":
                if self.paused.is_set():
                    self.paused.clear()
                    print("Playback resumed.", flush=True)
                else:
                    self.paused.set()
                    print("Playback paused. Press SPACE to resume.", flush=True)

    def wait_if_paused(self, hold_callback=None):
        if not self.enabled or not self.paused.is_set():
            return 0.0

        paused_at = time.monotonic()
        hold_interval = 1.0 / self.hold_rate
        next_hold = 0.0
        while self.paused.is_set() and not self.stop_requested.is_set():
            now = time.monotonic()
            if hold_callback and now >= next_hold:
                hold_callback()
                next_hold = now + hold_interval
            time.sleep(0.01)
        return time.monotonic() - paused_at


def sleep_with_pause(seconds: float, pause_controller: Optional[PauseController], hold_callback=None):
    end_time = time.monotonic() + seconds
    paused_seconds = 0.0
    while time.monotonic() < end_time:
        if pause_controller:
            paused = pause_controller.wait_if_paused(hold_callback)
            if paused:
                paused_seconds += paused
                end_time += paused
        time.sleep(min(0.02, max(0.0, end_time - time.monotonic())))
    return paused_seconds


def get_most_recent(base_path):
    dirs = []
    for d in pathlib.Path.iterdir(pathlib.Path(base_path)):
        if d.is_dir():
            dirs.append(d)
    dirs.sort(key=lambda d: int(d.name))
    return dirs[-1]


def get_random(base_path):
    dirs = []
    for d in pathlib.Path.iterdir(pathlib.Path(base_path)):
        if d.is_dir():
            dirs.append(d)
    return dirs[random.randint(0, len(dirs) - 1)]


def joint_positions(state):
    return [
        state.joint1Pos,
        state.joint2Pos,
        state.joint3Pos,
        state.joint4Pos,
        state.joint5Pos,
        state.joint6Pos,
        state.joint7Pos,
    ]


def decode_packet(buf: bytes):
    with robotstate_capnp.RobotState.from_bytes(buf) as state:
        return state.time, joint_positions(state)


def read_next_valid_packet(f, name: str, file: pathlib.Path):
    while True:
        offset = f.tell()
        buf = f.read(MESSAGE_SIZE)
        if not buf:
            return None
        if len(buf) != MESSAGE_SIZE:
            print(
                f"{file} | {name} | skipped partial packet at byte {offset}",
                file=sys.stderr,
                flush=True,
            )
            return None
        try:
            timestamp, positions = decode_packet(buf)
            return offset, buf, timestamp, positions
        except Exception as exc:
            print(
                f"{file} | {name} | skipped invalid packet at byte {offset}: {exc}",
                file=sys.stderr,
                flush=True,
            )


def read_start_packet(f, name: str, file: pathlib.Path, start_position_seconds: float):
    first_packet = read_next_valid_packet(f, name, file)
    if first_packet is None:
        return None

    if start_position_seconds == 0:
        return first_packet

    _, _, recording_start_time, _ = first_packet
    start_timestamp = recording_start_time + round(start_position_seconds * 1000)
    packet = first_packet
    while packet is not None:
        _, _, timestamp, _ = packet
        if timestamp >= start_timestamp:
            skipped_seconds = (timestamp - recording_start_time) / 1000.0
            print(
                f"{file} | {name} | skipped to {skipped_seconds:.3f}s "
                f"(requested {start_position_seconds:.3f}s)",
                flush=True,
            )
            return packet
        packet = read_next_valid_packet(f, name, file)

    return None


def packet_time_bounds(file: pathlib.Path, name: str):
    with open(file, "rb") as f:
        first_packet = read_next_valid_packet(f, name, file)
        if first_packet is None:
            return None

        _, _, first_timestamp, _ = first_packet
        last_timestamp = first_timestamp
        while True:
            packet = read_next_valid_packet(f, name, file)
            if packet is None:
                break
            _, _, timestamp, _ = packet
            if timestamp >= first_timestamp:
                last_timestamp = max(last_timestamp, timestamp)

        return first_timestamp, last_timestamp


def recording_duration_seconds(file: pathlib.Path, name: str):
    bounds = packet_time_bounds(file, name)
    if bounds is None:
        raise ValueError(f"{file} does not contain any valid robot-state packets")

    start_timestamp, end_timestamp = bounds
    return max(0.0, (end_timestamp - start_timestamp) / 1000.0)


def make_feedback_socket(
    port: int,
    multicast_addr: str,
    iface: Optional[str],
    addr: Optional[str],
):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))

    mc_addr = IPv4Address(multicast_addr)
    if iface and addr:
        if_addr = IPv4Address(addr)
        if_index = socket.if_nametoindex(iface)
        mreq = struct.pack("@4s4si", mc_addr.packed, if_addr.packed, if_index)
    else:
        mreq = struct.pack("4s4s", mc_addr.packed, socket.inet_aton("0.0.0.0"))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(0.1)
    return sock


def wait_for_start_feedback(
    name: str,
    target_positions: list[float],
    target_packet: bytes,
    command_sock: socket.socket,
    command_host: str,
    command_port: int,
    feedback_port: int,
    multicast_addr: str,
    iface: Optional[str],
    addr: Optional[str],
    tolerance: float,
    stable_seconds: float,
    timeout_seconds: float,
    publish_rate: float,
):
    feedback_sock = make_feedback_socket(feedback_port, multicast_addr, iface, addr)
    publish_interval = 1.0 / publish_rate
    next_publish = time.monotonic()
    deadline = time.monotonic() + timeout_seconds
    stable_since = None

    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                command_sock.sendto(target_packet, (command_host, command_port))
                next_publish = now + publish_interval

            try:
                data = feedback_sock.recv(MESSAGE_SIZE)
            except socket.timeout:
                continue

            with robotstate_capnp.RobotState.from_bytes(data) as state:
                actual_positions = joint_positions(state)
            max_error = max(
                abs(actual - target)
                for actual, target in zip(actual_positions, target_positions)
            )

            if max_error <= tolerance:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= stable_seconds:
                    print(f"{name}: start pose reached, max error {max_error:.4f}", flush=True)
                    return
            else:
                stable_since = None
    finally:
        feedback_sock.close()

    raise TimeoutError(f"{name}: timed out waiting for start pose feedback")


def play(
    name: str,
    file: pathlib.Path,
    host: str,
    port: int,
    feedback_port: int,
    start_hold_seconds: float,
    start_hold_rate: float,
    wait_for_feedback: bool,
    feedback_multicast_addr: str,
    feedback_iface: Optional[str],
    feedback_addr: Optional[str],
    start_tolerance: float,
    start_stable_seconds: float,
    start_timeout: float,
    start_position_seconds: float,
    pause_controller: Optional[PauseController] = None,
):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with open(file, "rb") as f:
        first_packet = read_start_packet(f, name, file, start_position_seconds)
        if first_packet is None:
            raise ValueError(
                f"{file} does not contain a valid robot-state packet at or after "
                f"{start_position_seconds:.3f}s"
            )

        _, buf, basetime, target_positions = first_packet
        lasttime = basetime
        print(f"{file} | {name} | robot_time={lasttime}", flush=True)
        sock.sendto(buf, (host, port))
        last_sent_buf = buf

        if wait_for_feedback:
            wait_for_start_feedback(
                name,
                target_positions,
                buf,
                sock,
                host,
                port,
                feedback_port,
                feedback_multicast_addr,
                feedback_iface,
                feedback_addr,
                start_tolerance,
                start_stable_seconds,
                start_timeout,
                start_hold_rate,
            )
        elif start_hold_seconds > 0:
            hold_interval = 1.0 / start_hold_rate
            hold_until = time.monotonic() + start_hold_seconds
            while time.monotonic() < hold_until:
                sock.sendto(last_sent_buf, (host, port))
                sleep_with_pause(
                    hold_interval,
                    pause_controller,
                    lambda: sock.sendto(last_sent_buf, (host, port)),
                )

        starttime = round(time.time() * 1000)
        while True:
            packet = read_next_valid_packet(f, name, file)
            if packet is None:
                break

            _, buf, timestamp, _ = packet
            if timestamp < lasttime:
                continue

            timediff = timestamp - basetime
            while True:
                paused_seconds = (
                    pause_controller.wait_if_paused(lambda: sock.sendto(last_sent_buf, (host, port)))
                    if pause_controller
                    else 0.0
                )
                if paused_seconds:
                    starttime += round(paused_seconds * 1000)

                now = round(time.time() * 1000)
                target_time = starttime + timediff
                if target_time <= now:
                    break
                time.sleep(min((target_time - now) / 1000, 0.02))

            lasttime = timestamp
            sock.sendto(buf, (host, port))
            last_sent_buf = buf
            print(f"{file} | {name} | robot_time={lasttime}", flush=True)


def main(argv):
    parser = argparse.ArgumentParser(description="Play back recorded robot movement data")
    parser.add_argument("recording", nargs="?", help="Recording folder name, or 'r' for random")
    parser.add_argument("--bob-only", action="store_true", help="Only play Bob's recording")
    parser.add_argument("--vincent-only", action="store_true", help="Only play Vincent's recording")
    parser.add_argument(
        "--start-hold-seconds",
        type=float,
        default=5.0,
        help="Seconds to repeatedly publish the first pose before timed playback starts",
    )
    parser.add_argument(
        "--start-hold-rate",
        type=float,
        default=20.0,
        help="Rate in Hz for republishing the first pose during the start hold",
    )
    parser.add_argument(
        "--wait-for-start-feedback",
        action="store_true",
        help="Wait for robot state feedback to reach the first pose before timed playback",
    )
    parser.add_argument(
        "--start-tolerance",
        type=float,
        default=0.02,
        help="Maximum per-joint start error in radians when using feedback wait",
    )
    parser.add_argument(
        "--start-stable-seconds",
        type=float,
        default=0.25,
        help="Seconds the robot must remain within tolerance before playback starts",
    )
    parser.add_argument(
        "--start-timeout",
        type=float,
        default=30.0,
        help="Maximum seconds to wait for start feedback",
    )
    parser.add_argument(
        "--start-position",
        type=float,
        default=0.0,
        help="Elapsed recording time in seconds to start playback from",
    )
    parser.add_argument(
        "--maddr",
        default=VINCENT_HOST,
        help="Feedback multicast address to join when using feedback wait",
    )
    parser.add_argument("-i", "--iface", help="Interface name for feedback multicast")
    parser.add_argument("-a", "--addr", help="Local interface IP address for feedback multicast")
    parser.add_argument(
        "--no-keyboard-pause",
        action="store_true",
        help="Disable SPACE pause/resume handling",
    )
    parser.add_argument(
        "--pause-hold-rate",
        type=float,
        default=20.0,
        help="Rate in Hz for republishing the current pose while paused",
    )
    args = parser.parse_args(argv[1:])

    if args.start_hold_seconds < 0:
        parser.error("--start-hold-seconds must be >= 0")
    if args.start_hold_rate <= 0:
        parser.error("--start-hold-rate must be > 0")
    if args.start_tolerance <= 0:
        parser.error("--start-tolerance must be > 0")
    if args.start_stable_seconds < 0:
        parser.error("--start-stable-seconds must be >= 0")
    if args.start_timeout <= 0:
        parser.error("--start-timeout must be > 0")
    if args.start_position < 0:
        parser.error("--start-position must be >= 0")
    if args.pause_hold_rate <= 0:
        parser.error("--pause-hold-rate must be > 0")

    if args.recording:
        if args.recording == "r":
            path = get_random(BASE_PATH)
        else:
            path = pathlib.Path(BASE_PATH, args.recording)
    else:
        path = get_most_recent(BASE_PATH)

    threads = []
    pause_controller = PauseController(
        enabled=not args.no_keyboard_pause,
        hold_rate=args.pause_hold_rate,
    )
    pause_controller.start()

    # Add Vincent thread unless bob-only flag is specified
    if not args.bob_only:
        threads.append(
            threading.Thread(
                target=play,
                args=(
                    "Vincent",
                    pathlib.Path(path, "vincent"),
                    VINCENT_HOST,
                    VINCENT_PORT,
                    VINCENT_FEEDBACK_PORT,
                    args.start_hold_seconds,
                    args.start_hold_rate,
                    args.wait_for_start_feedback,
                    args.maddr,
                    args.iface,
                    args.addr,
                    args.start_tolerance,
                    args.start_stable_seconds,
                    args.start_timeout,
                    args.start_position,
                    pause_controller,
                ),
            )
        )
    
    # Add Bob thread unless vincent-only flag is specified
    if not args.vincent_only:
        threads.append(
            threading.Thread(
                target=play,
                args=(
                    "Bob",
                    pathlib.Path(path, "bob"),
                    BOB_HOST,
                    BOB_PORT,
                    BOB_FEEDBACK_PORT,
                    args.start_hold_seconds,
                    args.start_hold_rate,
                    args.wait_for_start_feedback,
                    args.maddr,
                    args.iface,
                    args.addr,
                    args.start_tolerance,
                    args.start_stable_seconds,
                    args.start_timeout,
                    args.start_position,
                    pause_controller,
                ),
            )
        )

    try:
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    finally:
        pause_controller.stop()


if __name__ == "__main__":
    main(sys.argv)
