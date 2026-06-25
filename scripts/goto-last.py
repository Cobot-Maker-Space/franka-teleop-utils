import argparse
import pathlib
import random
import socket
import sys
import threading
import time

import capnp

capnp.remove_import_hook()
robotstate_capnp = capnp.load("robot-state.capnp")

VINCENT_HOST = "224.3.29.71"
VINCENT_PORT = 49185
BOB_HOST = "224.3.29.71"
BOB_PORT = 49186
MESSAGE_SIZE = 248
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


def read_last_packet(file: pathlib.Path):
    size = file.stat().st_size
    if size < MESSAGE_SIZE:
        raise ValueError(f"{file} is smaller than one robot-state packet")

    complete_packets = size // MESSAGE_SIZE
    if complete_packets == 0:
        raise ValueError(f"{file} does not contain a complete robot-state packet")

    with open(file, "rb") as f:
        f.seek((complete_packets - 1) * MESSAGE_SIZE)
        packet = f.read(MESSAGE_SIZE)

    if len(packet) != MESSAGE_SIZE:
        raise ValueError(f"Could not read a complete final packet from {file}")

    return packet


def describe_packet(name: str, file: pathlib.Path, packet: bytes):
    with robotstate_capnp.RobotState.from_bytes(packet) as state:
        positions = [
            state.joint1Pos,
            state.joint2Pos,
            state.joint3Pos,
            state.joint4Pos,
            state.joint5Pos,
            state.joint6Pos,
            state.joint7Pos,
        ]
        print(
            f"{file} | {name} | final robot_time={state.time} | joints "
            + ", ".join(f"{position:.4f}" for position in positions),
            flush=True,
        )


def publish_packet(
    name: str,
    file: pathlib.Path,
    host: str,
    port: int,
    repeat_seconds: float,
    repeat_rate: float,
):
    packet = read_last_packet(file)
    describe_packet(name, file, packet)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(packet, (host, port))
        if repeat_seconds > 0:
            interval = 1.0 / repeat_rate
            end_time = time.monotonic() + repeat_seconds
            while time.monotonic() < end_time:
                time.sleep(interval)
                sock.sendto(packet, (host, port))
    finally:
        sock.close()

    if repeat_seconds > 0:
        print(f"{name}: sent final packet for {repeat_seconds:.1f}s", flush=True)
    else:
        print(f"{name}: sent final packet once", flush=True)


def main(argv):
    parser = argparse.ArgumentParser(description="Move robot(s) to the final pose of a recording")
    parser.add_argument("recording", nargs="?", help="Recording folder name, or 'r' for random")
    parser.add_argument("--bob-only", action="store_true", help="Only send Bob's final pose")
    parser.add_argument("--vincent-only", action="store_true", help="Only send Vincent's final pose")
    parser.add_argument(
        "--repeat-seconds",
        type=float,
        default=0.0,
        help="Seconds to keep resending the final packet after the first send",
    )
    parser.add_argument(
        "--repeat-rate",
        type=float,
        default=20.0,
        help="Rate in Hz when --repeat-seconds is greater than zero",
    )
    args = parser.parse_args(argv[1:])

    if args.repeat_seconds < 0:
        parser.error("--repeat-seconds must be >= 0")
    if args.repeat_rate <= 0:
        parser.error("--repeat-rate must be > 0")
    if args.bob_only and args.vincent_only:
        parser.error("--bob-only and --vincent-only cannot be used together")

    if args.recording:
        if args.recording == "r":
            path = get_random(BASE_PATH)
        else:
            path = pathlib.Path(BASE_PATH, args.recording)
    else:
        path = get_most_recent(BASE_PATH)

    threads = []

    if not args.bob_only:
        threads.append(
            threading.Thread(
                target=publish_packet,
                args=(
                    "Vincent",
                    pathlib.Path(path, "vincent"),
                    VINCENT_HOST,
                    VINCENT_PORT,
                    args.repeat_seconds,
                    args.repeat_rate,
                ),
            )
        )

    if not args.vincent_only:
        threads.append(
            threading.Thread(
                target=publish_packet,
                args=(
                    "Bob",
                    pathlib.Path(path, "bob"),
                    BOB_HOST,
                    BOB_PORT,
                    args.repeat_seconds,
                    args.repeat_rate,
                ),
            )
        )

    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()


if __name__ == "__main__":
    main(sys.argv)
