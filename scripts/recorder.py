import datetime
import pathlib
import socket
import threading
import time

import capnp

capnp.remove_import_hook()
robotstate_capnp = capnp.load("robot-state.capnp")

VINCENT_PORT = 49186
BOB_PORT = 49188
MESSAGE_SIZE = 192
BASE_PATH = "/Users/pszsdc/repos/soma/franka-teleop-utils/recordings"

# Shared packet counters
packet_counts = {"vincent": 0, "bob": 0}
counts_lock = threading.Lock()


def record(file: pathlib.Path, sock: socket.socket, name: str):
    try:
        with open(file, "wb") as f:
            while True:
                f.write(sock.recv(MESSAGE_SIZE))
                f.flush()
                with counts_lock:
                    packet_counts[name] += 1
    except:
        return


def display_status(outdir: pathlib.Path):
    # Move cursor up to the status line and update in place
    print("Status: Waiting for packets...")  # Initial status line
    while True:
        with counts_lock:
            vincent_count = packet_counts["vincent"]
            bob_count = packet_counts["bob"]
        
        # Move cursor up one line, clear it, write status, then move cursor back
        print(f"\033[1A\033[2KStatus: Recording | Vincent: {vincent_count} packets | Bob: {bob_count} packets")
        time.sleep(0.5)


if __name__ == "__main__":

    vsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    vsock.bind(("", VINCENT_PORT))

    bsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    bsock.bind(("", BOB_PORT))

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
    
    print(f"Recording to directory: {outdir}")
    print(f"Vincent file: {vfile}")
    print(f"Bob file: {bfile}")
    print("Press Ctrl+C to stop recording...")
    print()

    try:
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        print("\nStopping recording...")
        try:
          vsock.shutdown(socket.SHUT_RDWR)
          bsock.shutdown(socket.SHUT_RDWR)
        except:
            pass
