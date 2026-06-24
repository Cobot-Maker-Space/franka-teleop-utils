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


def read_last_positions(file: pathlib.Path):
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

    with robotstate_capnp.RobotState.from_bytes(packet) as state:
        return state.time, [
            state.joint1Pos,
            state.joint2Pos,
            state.joint3Pos,
            state.joint4Pos,
            state.joint5Pos,
            state.joint6Pos,
            state.joint7Pos,
        ]


def make_position_packet(positions):
    state = robotstate_capnp.RobotState()
    state.time = round(time.time() * 1000)
    state.joint1Pos = positions[0]
    state.joint2Pos = positions[1]
    state.joint3Pos = positions[2]
    state.joint4Pos = positions[3]
    state.joint5Pos = positions[4]
    state.joint6Pos = positions[5]
    state.joint7Pos = positions[6]
    state.joint1Vel = 0
    state.joint2Vel = 0
    state.joint3Vel = 0
    state.joint4Vel = 0
    state.joint5Vel = 0
    state.joint6Vel = 0
    state.joint7Vel = 0
    state.joint1Torque = 0
    state.joint2Torque = 0
    state.joint3Torque = 0
    state.joint4Torque = 0
    state.joint5Torque = 0
    state.joint6Torque = 0
    state.joint7Torque = 0
    state.joint1ExtTorque = 0
    state.joint2ExtTorque = 0
    state.joint3ExtTorque = 0
    state.joint4ExtTorque = 0
    state.joint5ExtTorque = 0
    state.joint6ExtTorque = 0
    state.joint7ExtTorque = 0
    return state.to_bytes()


def publish_packet(name: str, file: pathlib.Path, host: str, port: int, seconds: float, rate: float):
    recording_time, positions = read_last_positions(file)
    print(
        f"{name}: final recording time {recording_time}, joints "
        + ", ".join(f"{position:.4f}" for position in positions),
        flush=True,
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / rate
    end_time = time.monotonic() + seconds

    try:
        while time.monotonic() < end_time:
            sock.sendto(make_position_packet(positions), (host, port))
            time.sleep(interval)
    finally:
        sock.close()

    print(f"{name}: sent final pose for {seconds:.1f}s", flush=True)


def main(argv):
    parser = argparse.ArgumentParser(description="Move robot(s) to the final pose of a recording")
    parser.add_argument("recording", nargs="?", help="Recording folder name, or 'r' for random")
    parser.add_argument("--bob-only", action="store_true", help="Only send Bob's final pose")
    parser.add_argument("--vincent-only", action="store_true", help="Only send Vincent's final pose")
    parser.add_argument(
        "--seconds",
        type=float,
        default=8.0,
        help="Seconds to publish the final pose",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Rate in Hz for publishing the final pose",
    )
    args = parser.parse_args(argv[1:])

    if args.seconds <= 0:
        parser.error("--seconds must be > 0")
    if args.rate <= 0:
        parser.error("--rate must be > 0")
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
                    args.seconds,
                    args.rate,
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
                    args.seconds,
                    args.rate,
                ),
            )
        )

    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()


if __name__ == "__main__":
    main(sys.argv)
