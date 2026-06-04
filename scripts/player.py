import argparse
import pathlib
import socket
import threading
import time

from collections.abc import Callable

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


def play(
    name: str,
    file: pathlib.Path,
    host: str,
    port: int,
    counters: dict[str, int],
    is_running: Callable[[], bool],
):
    message_size = compute_message_size_bytes()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with open(file, "rb") as f:
        buf = f.read(message_size)
        starttime = round(time.time() * 1000)
        with robotstate_capnp.RobotState.from_bytes(buf) as state:
            basetime = state.time
        lasttime = basetime
        sock.sendto(buf, (host, port))

        # Pause for 5 seconds after sending the first packet
        time.sleep(5)

        while (buf := f.read(message_size)) and is_running():
            with robotstate_capnp.RobotState.from_bytes(buf) as state:
                if state.time < lasttime:
                    continue
                timediff = state.time - basetime
                now = round(time.time() * 1000)
                if starttime + timediff > now:
                    time.sleep(((starttime + timediff) - now) / 1000)
                lasttime = state.time
                sock.sendto(buf, (host, port))
                counters[name] += 1


def display_status(counters: dict[str, int]):
    # Clear screen and hide cursor
    print("\033[2J\033[H\033[?25l")

    while True:
        print("\033[H\033[2J")
        for k, v in counters.items():
            print(f"{k}: {v} messages sent")
        time.sleep(1 / 10)


def main() -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "-r", "--robot", nargs=4, action="append"
    )  # name host port path
    args = parser.parse_args()

    threads: list[threading.Thread] = []
    counters: dict[str, int] = {}

    running = True

    def is_running() -> bool:
        return running

    for r in args.robot:
        counters[r[0]] = 0
        threads.append(
            threading.Thread(
                target=play,
                args=(r[0], pathlib.Path(r[3]), r[1], int(r[2]), counters, is_running),
            )
        )

    threading.Thread(target=display_status, args=(counters,), daemon=True).start()

    try:
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        running = False

    print("\033[?25h")  # Show cursor
    print("Playback complete")


if __name__ == "__main__":
    main()
