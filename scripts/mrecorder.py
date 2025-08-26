import datetime
import pathlib
import socket
import threading

import capnp

capnp.remove_import_hook()
robotstate_capnp = capnp.load("robot-state.capnp")

VINCENT_PORT = 49186
BOB_PORT = 49188
MESSAGE_SIZE = 136
BASE_PATH = "/Users/pszdp1/Library/CloudStorage/OneDrive-TheUniversityofNottingham/Development/embrace-angels/eapy/movements"


def record(file: pathlib.Path, sock: socket.socket):
    try:
        with open(file, "wb") as f:
            while True:
                f.write(sock.recv(MESSAGE_SIZE))
                f.flush()
    except:
        return


if __name__ == "__main__":

    vsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    vsock.bind(("", VINCENT_PORT))

    bsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    bsock.bind(("", BOB_PORT))

    now = datetime.datetime.now(datetime.UTC)
    outdir = pathlib.Path(BASE_PATH, now.strftime("%y%m%d%H%M%S"))
    outdir.mkdir()

    vfile = pathlib.Path(outdir, "vincent")
    bfile = pathlib.Path(outdir, "bob")

    threads = []

    threads.append(threading.Thread(target=record, args=(vfile, vsock)))
    threads.append(threading.Thread(target=record, args=(bfile, bsock)))

    try:
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        try:
          vsock.shutdown(socket.SHUT_RDWR)
          bsock.shutdown(socket.SHUT_RDWR)
        except:
            pass
