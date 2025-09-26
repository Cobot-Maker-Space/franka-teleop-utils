import csv
import pathlib
import socket
import sys
import threading
import time

import capnp

capnp.remove_import_hook()
robotstate_capnp = capnp.load("robot-state.capnp")

ROBOT_HOST = "224.3.29.71"
ROBOT_PORT = 49187
MESSAGE_SIZE = 248

frequency = 20

def play(name: str, file: pathlib.Path, host: str, port: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with open(file, "r") as f:
        csvreader = csv.reader(f, delimiter=',', quotechar='"')
        first_row = True
        
        for row in csvreader:
            state = robotstate_capnp.RobotState()
            state.time = int(time.time() * 1000)
            state.joint1Pos = float(row[0].replace('\ufeff', ''))
            print(float(row[0].replace('\ufeff', '')))
            state.joint2Pos = float(row[1].replace('\ufeff', ''))
            state.joint3Pos = float(row[2].replace('\ufeff', ''))
            state.joint4Pos = float(row[3].replace('\ufeff', ''))
            state.joint5Pos = float(row[4].replace('\ufeff', ''))
            state.joint6Pos = float(row[5].replace('\ufeff', ''))
            state.joint7Pos = float(row[6].replace('\ufeff', ''))
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
            
            sock.sendto(state.to_bytes(), (host, port))
            
            if first_row:
                print(f"Sent first position for {name}, waiting 5 seconds...")
                time.sleep(5.0)  # 5 second delay after first position
                first_row = False
            else:
                time.sleep(20 / 100)  # Normal timing for subsequent positions





def main(argv):
    path = argv[1]
    threads = []

    threads.append(
        threading.Thread(
            target=play,
            args=("Vincent", path, ROBOT_HOST, ROBOT_PORT),
        )
    )
    """
    threads.append(
        threading.Thread(
            target=play, args=("Bob", pathlib.Path(path, "bob"), BOB_HOST, BOB_PORT)
        )
    )
    """

    for t in threads:
        t.start()
    for t in threads:
        t.join()


if __name__ == "__main__":
    main(sys.argv)
