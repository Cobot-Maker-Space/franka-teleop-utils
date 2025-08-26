import pathlib
import random
import socket
import struct
import sys
import threading
import time

import capnp

capnp.remove_import_hook()
robotstate_capnp = capnp.load("robot-state.capnp")

VINCENT_HOST = "192.168.50.4"
VINCENT_PORT = 49187
BOB_HOST = "192.168.50.5"
BOB_PORT = 49189
MESSAGE_SIZE = 136
BASE_PATH = "/Users/pszdp1/Library/CloudStorage/OneDrive-TheUniversityofNottingham/Development/embrace-angels/eapy/movements"


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


def play(name: str, file: pathlib.Path, host: str, port: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with open(file, "rb") as f:
        buf = f.read(MESSAGE_SIZE)
        starttime = round(time.time() * 1000)
        with robotstate_capnp.RobotState.from_bytes(buf) as state:
            basetime = state.time
        lasttime = basetime
        print(f"{lasttime}: {name}", flush=True)
        sock.sendto(buf, (host, port))

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
    if len(argv) > 1:
        if argv[1] == "r":
            path = get_random(BASE_PATH)
        else:
            path = pathlib.Path(BASE_PATH, argv[1])
    else:
        path = get_most_recent(BASE_PATH)

    threads = []

    threads.append(
        threading.Thread(
            target=play,
            args=("Vincent", pathlib.Path(path, "vincent"), VINCENT_HOST, VINCENT_PORT),
        )
    )
    threads.append(
        threading.Thread(
            target=play, args=("Bob", pathlib.Path(path, "bob"), BOB_HOST, BOB_PORT)
        )
    )

    for t in threads:
        t.start()
    for t in threads:
        t.join()


if __name__ == "__main__":
    main(sys.argv)
