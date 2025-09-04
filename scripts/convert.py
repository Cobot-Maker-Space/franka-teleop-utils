import datetime
import pathlib
import sys

import capnp

capnp.remove_import_hook()
robotstate_capnp = capnp.load("../messages/robot-state.capnp")

MESSAGE_SIZE = 192  # Updated to include torque values

print("Timestamp,Position1,Position2,Position3,Position4,Position5,Position6,Position7,Velocity1,Velocity2,Velocity3,Velocity4,Velocity5,Velocity6,Velocity7,Torque1,Torque2,Torque3,Torque4,Torque5,Torque6,Torque7")

start_ts = pathlib.Path(sys.argv[1]).stat().st_ctime


with open(sys.argv[1], "rb") as f:
    while buf := f.read(MESSAGE_SIZE):
        with robotstate_capnp.RobotState.from_bytes(buf) as state:
            time = datetime.datetime.fromtimestamp(start_ts + (state.time / 1000))
            print(f"{time.strftime('%d/%m/%y %H:%M:%S')},", end="")
            print(f"{state.joint1Pos},", end="")
            print(f"{state.joint2Pos},", end="")
            print(f"{state.joint3Pos},", end="")
            print(f"{state.joint4Pos},", end="")
            print(f"{state.joint5Pos},", end="")
            print(f"{state.joint6Pos},", end="")
            print(f"{state.joint7Pos},", end="")
            print(f"{state.joint1Vel},", end="")
            print(f"{state.joint2Vel},", end="")
            print(f"{state.joint3Vel},", end="")
            print(f"{state.joint4Vel},", end="")
            print(f"{state.joint5Vel},", end="")
            print(f"{state.joint6Vel},", end="")
            print(f"{state.joint7Vel},", end="")
            print(f"{state.joint1Torque},", end="")
            print(f"{state.joint2Torque},", end="")
            print(f"{state.joint3Torque},", end="")
            print(f"{state.joint4Torque},", end="")
            print(f"{state.joint5Torque},", end="")
            print(f"{state.joint6Torque},", end="")
            print(f"{state.joint7Torque}")

