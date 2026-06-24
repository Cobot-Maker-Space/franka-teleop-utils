import argparse
import pathlib
import sys
import threading
import time
from typing import List

import player


def read_recording_names(list_file: pathlib.Path) -> List[str]:
    recordings = []
    with open(list_file, "r") as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            recordings.append(stripped)
    return recordings


def recording_path(recording: str) -> pathlib.Path:
    path = pathlib.Path(recording)
    if path.is_absolute():
        return path
    return pathlib.Path(player.BASE_PATH, recording)


def run_threaded_playback(path: pathlib.Path, args):
    errors = []

    def run_play(name, file, host, port, feedback_port):
        try:
            player.play(
                name,
                file,
                host,
                port,
                feedback_port,
                args.start_hold_seconds,
                args.start_hold_rate,
                args.wait_for_start_feedback,
                args.maddr,
                args.iface,
                args.addr,
                args.start_tolerance,
                args.start_stable_seconds,
                args.start_timeout,
            )
        except Exception as exc:
            errors.append((name, exc))

    threads = []

    if not args.bob_only:
        threads.append(
            threading.Thread(
                target=run_play,
                args=(
                    "Vincent",
                    pathlib.Path(path, "vincent"),
                    player.VINCENT_HOST,
                    player.VINCENT_PORT,
                    player.VINCENT_FEEDBACK_PORT,
                ),
            )
        )

    if not args.vincent_only:
        threads.append(
            threading.Thread(
                target=run_play,
                args=(
                    "Bob",
                    pathlib.Path(path, "bob"),
                    player.BOB_HOST,
                    player.BOB_PORT,
                    player.BOB_FEEDBACK_PORT,
                ),
            )
        )

    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    if errors:
        message = "; ".join(f"{name}: {exc}" for name, exc in errors)
        raise RuntimeError(message)


def main(argv):
    parser = argparse.ArgumentParser(description="Play a list of recordings sequentially")
    parser.add_argument("list_file", help="Text file containing one recording folder per line")
    parser.add_argument("--bob-only", action="store_true", help="Only play Bob's recordings")
    parser.add_argument("--vincent-only", action="store_true", help="Only play Vincent's recordings")
    parser.add_argument(
        "--pause-seconds",
        type=float,
        default=0.0,
        help="Seconds to wait between recordings",
    )
    parser.add_argument(
        "--continue-on-error",
        action="store_true",
        help="Continue with later recordings if one playback fails",
    )
    parser.add_argument(
        "--start-hold-seconds",
        type=float,
        default=5.0,
        help="Seconds to repeatedly publish each first pose before timed playback starts",
    )
    parser.add_argument(
        "--start-hold-rate",
        type=float,
        default=20.0,
        help="Rate in Hz for republishing each first pose during the start hold",
    )
    parser.add_argument(
        "--wait-for-start-feedback",
        action="store_true",
        help="Wait for robot state feedback to reach each first pose before timed playback",
    )
    parser.add_argument(
        "--start-tolerance",
        type=float,
        default=0.02,
        help="Maximum per-joint start error in radians when using feedback wait",
    )
    parser.add_argument(
        "--start-stable-seconds",
        type=float,
        default=0.25,
        help="Seconds the robot must remain within tolerance before playback starts",
    )
    parser.add_argument(
        "--start-timeout",
        type=float,
        default=30.0,
        help="Maximum seconds to wait for start feedback",
    )
    parser.add_argument(
        "--maddr",
        default=player.VINCENT_HOST,
        help="Feedback multicast address to join when using feedback wait",
    )
    parser.add_argument("-i", "--iface", help="Interface name for feedback multicast")
    parser.add_argument("-a", "--addr", help="Local interface IP address for feedback multicast")
    args = parser.parse_args(argv[1:])

    if args.bob_only and args.vincent_only:
        parser.error("--bob-only and --vincent-only cannot be used together")
    if args.pause_seconds < 0:
        parser.error("--pause-seconds must be >= 0")
    if args.start_hold_seconds < 0:
        parser.error("--start-hold-seconds must be >= 0")
    if args.start_hold_rate <= 0:
        parser.error("--start-hold-rate must be > 0")
    if args.start_tolerance <= 0:
        parser.error("--start-tolerance must be > 0")
    if args.start_stable_seconds < 0:
        parser.error("--start-stable-seconds must be >= 0")
    if args.start_timeout <= 0:
        parser.error("--start-timeout must be > 0")

    recordings = read_recording_names(pathlib.Path(args.list_file))
    if not recordings:
        parser.error("list file does not contain any recordings")

    failed = []
    for index, recording in enumerate(recordings, start=1):
        path = recording_path(recording)
        print(f"[{index}/{len(recordings)}] Playing {path}", flush=True)
        try:
            run_threaded_playback(path, args)
        except Exception as exc:
            print(f"Failed to play {recording}: {exc}", file=sys.stderr, flush=True)
            failed.append(recording)
            if not args.continue_on_error:
                return 1

        if index < len(recordings) and args.pause_seconds > 0:
            time.sleep(args.pause_seconds)

    if failed:
        print("Failed recordings: " + ", ".join(failed), file=sys.stderr, flush=True)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
