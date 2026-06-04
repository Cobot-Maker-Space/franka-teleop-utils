import argparse
import pathlib
import socket
import struct
import threading
import time

from collections.abc import Callable
from ipaddress import IPv4Address

import capnp

import netifaces

capnp.remove_import_hook()
robotstate_capnp = capnp.load(
    str(
        pathlib.Path(__file__).parent.parent.joinpath(
            pathlib.Path("messages/robot-state.capnp")
        )
    )
)


def compute_message_size_bytes() -> int:
    message = robotstate_capnp.RobotState()
    segments = message.to_segments()
    segment_table_size = (len(segments) // 2) + 1
    words = segment_table_size + sum(len(seg) // 8 for seg in segments)
    return words * 8


def record(
    file: pathlib.Path,
    sock: socket.socket,
    is_running: Callable[[], bool],
    robot_data: dict,
):
    message_size = compute_message_size_bytes()
    try:
        with open(file, "wb") as f:
            while is_running():
                data = sock.recv(message_size)
                f.write(data)
                f.flush()
                try:
                    with robotstate_capnp.RobotState.from_bytes(data) as robot_state:
                        robot_data["joint_positions"] = [
                            robot_state.joint1Pos,
                            robot_state.joint2Pos,
                            robot_state.joint3Pos,
                            robot_state.joint4Pos,
                            robot_state.joint5Pos,
                            robot_state.joint6Pos,
                            robot_state.joint7Pos,
                        ]

                        robot_data["joint_torques"] = [
                            robot_state.joint1Torque,
                            robot_state.joint2Torque,
                            robot_state.joint3Torque,
                            robot_state.joint4Torque,
                            robot_state.joint5Torque,
                            robot_state.joint6Torque,
                            robot_state.joint7Torque,
                        ]

                        robot_data["external_torques"] = [
                            robot_state.joint1ExtTorque,
                            robot_state.joint2ExtTorque,
                            robot_state.joint3ExtTorque,
                            robot_state.joint4ExtTorque,
                            robot_state.joint5ExtTorque,
                            robot_state.joint6ExtTorque,
                            robot_state.joint7ExtTorque,
                        ]

                        robot_data["packet_count"] += 1
                except Exception:
                    # If decoding fails, just increment packet count
                    robot_data["packet_count"] += 1

    except Exception as e:
        return


def display_status(shared_data: dict, is_running: Callable[[], bool]):
    # Clear screen and hide cursor
    print("\033[2J\033[H\033[?25l")

    while is_running():
        print("\033[H\033[2J")
        print("Recording, press ctrl+c to stop recording")
        print()
        for key in shared_data.keys():
            robot_data = shared_data[key]
            print(f"Robot: {key} | Packets received: {robot_data["packet_count"]}")
            print(
                f"{'Joint':>8} {'Position (rad)':>15} {'Torque (Nm)':>15} {'Ext Torque (Nm)':>18}"
            )
            print(f"{'-'*8} {'-'*15} {'-'*15} {'-'*18}")
            for i in range(7):
                print(
                    f"Joint {i+1:2d}: {robot_data["joint_positions"][i]:10.4f}    {robot_data["joint_torques"][i]:10.4f}     {robot_data["external_torques"][i]:13.4f}"
                )
            print()

        time.sleep(1 / 10)


def main() -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "-r", "--robot", nargs="+", action="append"
    )  # name port path interface multicast_host
    args = parser.parse_args()
    for r in args.robot:
        l = len(r)
        if not (l == 3 or l == 5):
            parser.error("argument -r/--robot: expected either 3 or 5 arguments")

    threads: list[threading.Thread] = []
    shared_data = {}
    running = True

    def is_running() -> bool:
        return running

    for r in args.robot:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", int(r[1])))
        if len(r) == 5:
            if_addr_str = netifaces.ifaddresses(r[3])[netifaces.AF_INET][0]["addr"]
            if_addr = IPv4Address(if_addr_str)
            if_index = socket.if_nametoindex(r[3])
            mc_addr = IPv4Address(r[4])
            mreq = struct.pack(
                "@4s4si",
                mc_addr.packed,
                if_addr.packed,
                if_index,
            )
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        outfile = pathlib.Path(r[2])
        if outfile.exists():
            raise SystemExit(f"File '{outfile}' already exists, exiting")
        robot_data = {
            "packet_count": 0,
            "joint_positions": [0.0] * 7,
            "joint_torques": [0.0] * 7,
            "external_torques": [0.0] * 7,
        }
        shared_data[r[0]] = robot_data
        threads.append(
            threading.Thread(
                target=record, args=(outfile, sock, is_running, robot_data)
            )
        )

    threading.Thread(
        target=display_status, args=(shared_data, is_running), daemon=True
    ).start()

    try:
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        running = False

    print("\033[?25h")  # Show cursor
    print("Stopped recording")


if __name__ == "__main__":
    main()
