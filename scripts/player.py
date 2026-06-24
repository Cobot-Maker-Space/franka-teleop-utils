import pathlib
import random
import socket
import struct
import sys
import argparse
import threading
import time

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
):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with open(file, "rb") as f:
        buf = f.read(MESSAGE_SIZE)
        with robotstate_capnp.RobotState.from_bytes(buf) as state:
            basetime = state.time
            target_positions = joint_positions(state)
        lasttime = basetime
        print(f"{lasttime}: {name}", flush=True)
        sock.sendto(buf, (host, port))

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
                sock.sendto(buf, (host, port))
                time.sleep(hold_interval)

        starttime = round(time.time() * 1000)
        while buf := f.read(MESSAGE_SIZE):
            with robotstate_capnp.RobotState.from_bytes(buf) as state:
                if state.time < lasttime:
                    continue
                timediff = state.time - basetime
                now = round(time.time() * 1000)
                if starttime + timediff > now:
                    time.sleep(((starttime + timediff) - now) / 1000)
                lasttime = state.time
                sock.sendto(buf, (host, port))
                print(f"{lasttime}: {name}", flush=True)


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
        "--maddr",
        default=VINCENT_HOST,
        help="Feedback multicast address to join when using feedback wait",
    )
    parser.add_argument("-i", "--iface", help="Interface name for feedback multicast")
    parser.add_argument("-a", "--addr", help="Local interface IP address for feedback multicast")
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

    if args.recording:
        if args.recording == "r":
            path = get_random(BASE_PATH)
        else:
            path = pathlib.Path(BASE_PATH, args.recording)
    else:
        path = get_most_recent(BASE_PATH)

    threads = []

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
                ),
            )
        )

    for t in threads:
        t.start()
    for t in threads:
        t.join()


if __name__ == "__main__":
    main(sys.argv)
