import argparse
import csv
import pathlib
import socket
import sys
import time

import capnp

capnp.remove_import_hook()
robotstate_capnp = capnp.load(
    str(
        pathlib.Path(__file__).parent.parent.joinpath(
            pathlib.Path("messages/robot-state.capnp")
        )
    )
)


def play(file: pathlib.Path, frequency: int, host: str, port: int, skip_header: bool):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with open(file, "r") as f:
        csvreader = csv.reader(f, delimiter=",", quotechar='"')

        if skip_header:
            next(csvreader)

        first_row = True

        state = robotstate_capnp.RobotState()
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

        for row in csvreader:
            state.time = int(time.time() * 1000)
            state.joint1Pos = float(row[0])
            state.joint2Pos = float(row[1])
            state.joint3Pos = float(row[2])
            state.joint4Pos = float(row[3])
            state.joint5Pos = float(row[4])
            state.joint6Pos = float(row[5])
            state.joint7Pos = float(row[6])

            sock.sendto(state.to_bytes(), (host, port))

            if first_row:
                input("First position sent, press enter to continue when ready")
                first_row = False
            else:
                time.sleep(1 / frequency)


def main(args: list[str]):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "-h", "--host", help="Robot host/IP address", type=str, required=True
    )
    parser.add_argument(
        "-p", "--port", help="Robot port number", type=int, required=True
    )
    parser.add_argument(
        "-f", "--frequency", help="Playback frequency (Hz)", type=int, required=True
    )
    parser.add_argument(
        "-s", "--skip-header", help="Skip first line of CSV file", action="store_true"
    )
    parser.add_argument("csvfile", help="Path to CSV file to play from", type=str)
    pargs = parser.parse_args(args=args)
    play(pargs.csvfile, pargs.frequency, pargs.host, pargs.port, pargs.skip_header)


# TODO: Use readchar lib to interactively send messages
if __name__ == "__main__":
    main(sys.argv)
