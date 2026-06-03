import argparse
import csv
import datetime
import pathlib
import sys

from typing import BinaryIO, TextIO

import capnp

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


def convert(input: BinaryIO, output: TextIO) -> None:
    writer = csv.writer(output)
    writer.writerow(
        [
            "Timestamp",
            "Position1",
            "Position2",
            "Position3",
            "Position4",
            "Position5",
            "Position6",
            "Position7",
            "Velocity1",
            "Velocity2",
            "Velocity3",
            "Velocity4",
            "Velocity5",
            "Velocity6",
            "Velocity7",
            "Torque1",
            "Torque2",
            "Torque3",
            "Torque4",
            "Torque5",
            "Torque6",
            "Torque7",
            "ExtTorque1",
            "ExtTorque2",
            "ExtTorque3",
            "ExtTorque4",
            "ExtTorque5",
            "ExtTorque6",
            "ExtTorque7",
        ]
    )

    message_size = compute_message_size_bytes()
    while buf := input.read(message_size):
        with robotstate_capnp.RobotState.from_bytes(buf) as state:
            # Convert Unix timestamp (milliseconds) to readable format
            timestamp = datetime.datetime.fromtimestamp(
                state.time / 1000.0, tz=datetime.timezone.utc
            )
            writer.writerow(
                [
                    f"{timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}Z",
                    state.joint1Pos,
                    state.joint2Pos,
                    state.joint3Pos,
                    state.joint4Pos,
                    state.joint5Pos,
                    state.joint6Pos,
                    state.joint7Pos,
                    state.joint1Vel,
                    state.joint2Vel,
                    state.joint3Vel,
                    state.joint4Vel,
                    state.joint5Vel,
                    state.joint6Vel,
                    state.joint7Vel,
                    state.joint1Torque,
                    state.joint2Torque,
                    state.joint3Torque,
                    state.joint4Torque,
                    state.joint5Torque,
                    state.joint6Torque,
                    state.joint7Torque,
                    state.joint1ExtTorque,
                    state.joint2ExtTorque,
                    state.joint3ExtTorque,
                    state.joint4ExtTorque,
                    state.joint5ExtTorque,
                    state.joint6ExtTorque,
                    state.joint7ExtTorque,
                ]
            )


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("infile")
    parser.add_argument("-o", "--outfile", default=None, required=False)
    args = parser.parse_args()
    if args.outfile is None:
        with open(args.infile, "rb") as f:
            convert(f, sys.stdout)
    else:
        with open(args.infile, "rb") as f1:
            with open(args.outfile, "w") as f2:
                convert(f1, f2)


if __name__ == "__main__":
    main()
