import argparse
import datetime
import pathlib
import socket
import struct
import threading
import time

from ipaddress import IPv4Address

import capnp

capnp.remove_import_hook()
robotstate_capnp = capnp.load("../messages/robot-state.capnp")

VINCENT_PORT = 49186
BOB_PORT = 49188
MESSAGE_SIZE = 248
BASE_PATH = "recordings"

# Shared packet counters and robot data
packet_counts = {"vincent": 0, "bob": 0}
joint_positions = {"vincent": [0.0] * 7, "bob": [0.0] * 7}
joint_torques = {"vincent": [0.0] * 7, "bob": [0.0] * 7}
external_torques = {"vincent": [0.0] * 7, "bob": [0.0] * 7}
counts_lock = threading.Lock()


def record(file: pathlib.Path, sock: socket.socket, name: str):
    try:
        with open(file, "wb") as f:
            while True:
                data = sock.recv(MESSAGE_SIZE)
                f.write(data)
                f.flush()

                # Decode the message to extract joint positions
                try:
                    # Parse the CapnProto message using the context manager approach (like convert.py)
                    with robotstate_capnp.RobotState.from_bytes(data) as robot_state:
                        positions = [
                            robot_state.joint1Pos,
                            robot_state.joint2Pos,
                            robot_state.joint3Pos,
                            robot_state.joint4Pos,
                            robot_state.joint5Pos,
                            robot_state.joint6Pos,
                            robot_state.joint7Pos
                        ]

                        torques = [
                            robot_state.joint1Torque,
                            robot_state.joint2Torque,
                            robot_state.joint3Torque,
                            robot_state.joint4Torque,
                            robot_state.joint5Torque,
                            robot_state.joint6Torque,
                            robot_state.joint7Torque
                        ]

                        ext_torques = [
                            robot_state.joint1ExtTorque,
                            robot_state.joint2ExtTorque,
                            robot_state.joint3ExtTorque,
                            robot_state.joint4ExtTorque,
                            robot_state.joint5ExtTorque,
                            robot_state.joint6ExtTorque,
                            robot_state.joint7ExtTorque
                        ]

                        with counts_lock:
                            packet_counts[name] += 1
                            joint_positions[name] = positions
                            joint_torques[name] = torques
                            external_torques[name] = ext_torques
                except Exception as decode_error:
                    # If decoding fails, just increment packet count
                    with counts_lock:
                        packet_counts[name] += 1

    except:
        return


def display_status(outdir: pathlib.Path):
    # Clear screen and hide cursor
    print("\033[2J\033[H\033[?25l")

    while True:
        with counts_lock:
            vincent_count = packet_counts["vincent"]
            bob_count = packet_counts["bob"]
            vincent_joints = joint_positions["vincent"].copy()
            bob_joints = joint_positions["bob"].copy()
            vincent_torques = joint_torques["vincent"].copy()
            vincent_ext_torques = external_torques["vincent"].copy()
            bob_torques = joint_torques["bob"].copy()
            bob_ext_torques = external_torques["bob"].copy()

        # Move cursor to top and display status
        print(f"\033[H\033[2J")
        print(f"Recording to: {outdir}")
        print(f"Press Ctrl+C to stop recording...")
        print()
        print(f"{'='*80}")
        print(f"PACKETS: Vincent: {vincent_count:6d} | Bob: {bob_count:6d}")
        print(f"{'='*80}")
        print()

        # Display Vincent's data in three columns
        print(f"VINCENT Robot Data:")
        print(f"{'Joint':>8} {'Position (rad)':>15} {'Torque (Nm)':>15} {'Ext Torque (Nm)':>18}")
        print(f"{'-'*8} {'-'*15} {'-'*15} {'-'*18}")
        for i in range(7):
            print(f"Joint {i+1:2d}: {vincent_joints[i]:10.4f}    {vincent_torques[i]:10.4f}     {vincent_ext_torques[i]:13.4f}")
        print()

        # Display Bob's data in three columns (mirroring Vincent's format)
        print(f"BOB Robot Data:")
        print(f"{'Joint':>8} {'Position (rad)':>15} {'Torque (Nm)':>15} {'Ext Torque (Nm)':>18}")
        print(f"{'-'*8} {'-'*15} {'-'*15} {'-'*18}")
        for i in range(7):
            print(f"Joint {i+1:2d}: {bob_joints[i]:10.4f}    {bob_torques[i]:10.4f}     {bob_ext_torques[i]:13.4f}")
        print()

        time.sleep(0.1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Record robot teleoperation data")
    parser.add_argument("--vincent-name", help="Custom filename for Vincent robot recording")
    parser.add_argument("--bob-name", help="Custom filename for Bob robot recording")
    parser.add_argument("-i", "--iface", help="Interface name to listen on", type=str)
    parser.add_argument("-a", "--addr", help="IP address to listeb on", type=str)
    parser.add_argument("-m", "--maddr", help="Multicast IP address to listen on", type=str)
    args = parser.parse_args()

    mc_addr = IPv4Address(args.maddr)
    if_addr = IPv4Address(args.addr)
    if_index = socket.if_nametoindex(args.iface)

    vsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mreq = struct.pack(
        "@4s4si",
        mc_addr.packed,
        if_addr.packed,
        if_index,
    )
    vsock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    vsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    vsock.bind(("", VINCENT_PORT))

    bsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mreq = struct.pack(
        "@4s4si",
        mc_addr.packed,
        if_addr.packed,
        if_index,
    )
    bsock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    bsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    bsock.bind(("", BOB_PORT))

    # Create output directory and files
    if args.vincent_name or args.bob_name:
        # If custom names provided, use recordings folder directly
        outdir = pathlib.Path(BASE_PATH)
        outdir.mkdir(exist_ok=True)

        vfile = pathlib.Path(outdir, args.vincent_name) if args.vincent_name else pathlib.Path(outdir, "vincent_default")
        bfile = pathlib.Path(outdir, args.bob_name) if args.bob_name else pathlib.Path(outdir, "bob_default")
    else:
        # Default behavior: create timestamp folder
        now = datetime.datetime.now(datetime.timezone.utc)
        outdir = pathlib.Path(BASE_PATH, now.strftime("%y%m%d%H%M%S"))
        outdir.mkdir()

        vfile = pathlib.Path(outdir, "vincent")
        bfile = pathlib.Path(outdir, "bob")

    threads = []

    threads.append(threading.Thread(target=record, args=(vfile, vsock, "vincent")))
    threads.append(threading.Thread(target=record, args=(bfile, bsock, "bob")))

    # Start display status thread
    status_thread = threading.Thread(target=display_status, args=(outdir,))
    status_thread.daemon = True
    status_thread.start()

    try:
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        print("\033[?25h")  # Show cursor
        print("\nStopping recording...")
        try:
          vsock.shutdown(socket.SHUT_RDWR)
          bsock.shutdown(socket.SHUT_RDWR)
        except:
            pass
