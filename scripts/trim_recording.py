import argparse
import pathlib
import shutil
import sys

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
BASE_PATH = SCRIPT_DIR.parent / "recordings"
MESSAGE_SIZE = 248

robotstate_capnp = None

ROBOT_STATE_FIELDS = [
    "joint1Pos",
    "joint2Pos",
    "joint3Pos",
    "joint4Pos",
    "joint5Pos",
    "joint6Pos",
    "joint7Pos",
    "joint1Vel",
    "joint2Vel",
    "joint3Vel",
    "joint4Vel",
    "joint5Vel",
    "joint6Vel",
    "joint7Vel",
    "joint1Torque",
    "joint2Torque",
    "joint3Torque",
    "joint4Torque",
    "joint5Torque",
    "joint6Torque",
    "joint7Torque",
    "joint1ExtTorque",
    "joint2ExtTorque",
    "joint3ExtTorque",
    "joint4ExtTorque",
    "joint5ExtTorque",
    "joint6ExtTorque",
    "joint7ExtTorque",
]


def load_dependencies():
    global robotstate_capnp

    try:
        import capnp
    except ModuleNotFoundError:
        print(
            "Missing Python dependency 'capnp'. Install the script requirements before "
            "trimming recordings.",
            file=sys.stderr,
        )
        sys.exit(1)

    capnp.remove_import_hook()
    robotstate_capnp = capnp.load(str(SCRIPT_DIR / "robot-state.capnp"))


def recording_path(recording: str) -> pathlib.Path:
    path = pathlib.Path(recording)
    if path.is_absolute() or path.exists():
        return path
    return pathlib.Path(BASE_PATH, recording)


def parse_trim_range(value: str) -> tuple[float, float]:
    if "-" not in value:
        raise argparse.ArgumentTypeError("trim ranges must use START-END seconds")

    start_text, end_text = value.split("-", 1)
    try:
        start = float(start_text)
        end = float(end_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "trim range start and end must be numbers"
        ) from exc

    if start < 0 or end < 0:
        raise argparse.ArgumentTypeError("trim times must be >= 0")
    if end <= start:
        raise argparse.ArgumentTypeError("trim range end must be greater than start")

    return start, end


def merge_ranges(ranges: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if not ranges:
        return []

    merged = []
    for start, end in sorted(ranges):
        if merged and start <= merged[-1][1]:
            previous_start, previous_end = merged[-1]
            merged[-1] = (previous_start, max(previous_end, end))
        else:
            merged.append((start, end))
    return merged


def should_trim(elapsed_seconds: float, ranges: list[tuple[float, float]]) -> bool:
    return any(start <= elapsed_seconds <= end for start, end in ranges)


def removed_before(elapsed_seconds: float, ranges: list[tuple[float, float]]) -> float:
    removed = 0.0
    for start, end in ranges:
        if elapsed_seconds <= start:
            break
        removed += min(elapsed_seconds, end) - start
    return removed


def copy_state_with_time(state, timestamp_ms: int) -> bytes:
    message = robotstate_capnp.RobotState.new_message()
    message.time = timestamp_ms
    for field in ROBOT_STATE_FIELDS:
        setattr(message, field, getattr(state, field))

    data = message.to_bytes()
    if len(data) != MESSAGE_SIZE:
        raise ValueError(
            f"rewritten packet is {len(data)} bytes, expected {MESSAGE_SIZE}"
        )
    return data


def trim_file(
    input_file: pathlib.Path,
    output_file: pathlib.Path,
    ranges: list[tuple[float, float]],
    preserve_timestamps: bool,
):
    output_file.parent.mkdir(parents=True, exist_ok=True)

    if input_file.stat().st_size == 0:
        output_file.write_bytes(b"")
        print(f"{input_file} -> {output_file}: empty input, wrote empty output")
        return

    valid_packets = 0
    kept_packets = 0
    trimmed_packets = 0
    invalid_packets = 0
    partial_packet_bytes = 0
    base_time = None

    with open(input_file, "rb") as source, open(output_file, "wb") as destination:
        while True:
            offset = source.tell()
            packet = source.read(MESSAGE_SIZE)
            if not packet:
                break
            if len(packet) != MESSAGE_SIZE:
                partial_packet_bytes = len(packet)
                print(
                    f"{input_file}: skipped partial packet at byte {offset} "
                    f"({len(packet)} of {MESSAGE_SIZE} bytes)",
                    file=sys.stderr,
                )
                break

            try:
                with robotstate_capnp.RobotState.from_bytes(packet) as state:
                    timestamp_ms = int(state.time)
                    if base_time is None:
                        base_time = timestamp_ms
                    elapsed_seconds = (timestamp_ms - base_time) / 1000.0
                    valid_packets += 1

                    if should_trim(elapsed_seconds, ranges):
                        trimmed_packets += 1
                        continue

                    if preserve_timestamps:
                        destination.write(packet)
                    else:
                        adjusted_timestamp_ms = timestamp_ms - round(
                            removed_before(elapsed_seconds, ranges) * 1000
                        )
                        destination.write(copy_state_with_time(state, adjusted_timestamp_ms))
                    kept_packets += 1
            except Exception as exc:
                invalid_packets += 1
                print(
                    f"{input_file}: skipped invalid packet at byte {offset}: {exc}",
                    file=sys.stderr,
                )

    if valid_packets == 0:
        raise ValueError(f"{input_file} does not contain any valid robot-state packets")
    if kept_packets == 0:
        raise ValueError(f"trim ranges removed every valid packet from {input_file}")

    print(f"{input_file} -> {output_file}")
    print(f"  valid packets: {valid_packets}")
    print(f"  kept packets: {kept_packets}")
    print(f"  trimmed packets: {trimmed_packets}")
    print(f"  invalid packets skipped: {invalid_packets}")
    if partial_packet_bytes:
        print(f"  trailing partial bytes skipped: {partial_packet_bytes}")


def resolve_input_files(
    input_path: pathlib.Path,
    output_path: pathlib.Path,
    include_bob: bool,
    include_vincent: bool,
) -> list[tuple[pathlib.Path, pathlib.Path]]:
    if input_path.is_file():
        if output_path.exists() and output_path.is_dir():
            return [(input_path, output_path / input_path.name)]
        return [(input_path, output_path)]

    if not input_path.is_dir():
        raise FileNotFoundError(f"{input_path} is not a file or recording folder")

    files = []
    if include_vincent:
        files.append("vincent")
    if include_bob:
        files.append("bob")

    pairs = []
    for name in files:
        source = input_path / name
        if not source.exists():
            raise FileNotFoundError(f"Missing recording file: {source}")
        pairs.append((source, output_path / name))
    return pairs


def copy_sidecar_files(input_path: pathlib.Path, output_path: pathlib.Path):
    if not input_path.is_dir():
        return

    output_path.mkdir(parents=True, exist_ok=True)
    for child in input_path.iterdir():
        if child.name in {"bob", "vincent"}:
            continue
        target = output_path / child.name
        if child.is_dir():
            shutil.copytree(child, target, dirs_exist_ok=True)
        elif child.is_file():
            shutil.copy2(child, target)


def main(argv):
    parser = argparse.ArgumentParser(
        description="Trim elapsed-time ranges from robot recording files"
    )
    parser.add_argument("input", help="Recording file, recording folder, or folder under recordings/")
    parser.add_argument("output", help="Output recording file or folder")
    parser.add_argument(
        "--trim",
        action="append",
        type=parse_trim_range,
        required=True,
        metavar="START-END",
        help="Elapsed time range to remove in seconds. Repeat for multiple ranges.",
    )
    parser.add_argument("--bob-only", action="store_true", help="Only trim bob from a folder")
    parser.add_argument(
        "--vincent-only",
        action="store_true",
        help="Only trim vincent from a folder",
    )
    parser.add_argument(
        "--preserve-timestamps",
        action="store_true",
        help="Keep original packet timestamps. By default timestamps are compacted.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Allow writing into an existing output path",
    )
    args = parser.parse_args(argv[1:])

    if args.bob_only and args.vincent_only:
        parser.error("--bob-only and --vincent-only cannot be used together")

    input_path = recording_path(args.input)
    output_path = pathlib.Path(args.output)
    ranges = merge_ranges(args.trim)

    if input_path.resolve() == output_path.resolve():
        parser.error("output must be different from input")
    if output_path.exists() and not args.force:
        parser.error(f"{output_path} already exists; use --force to write into it")

    load_dependencies()

    pairs = resolve_input_files(
        input_path,
        output_path,
        include_bob=not args.vincent_only,
        include_vincent=not args.bob_only,
    )

    if input_path.is_dir():
        copy_sidecar_files(input_path, output_path)

    print(
        "Trimming elapsed seconds: "
        + ", ".join(f"{start:.3f}-{end:.3f}" for start, end in ranges)
    )
    if args.preserve_timestamps:
        print("Preserving original timestamps")
    else:
        print("Compacting timestamps after removed ranges")

    for input_file, output_file in pairs:
        trim_file(input_file, output_file, ranges, args.preserve_timestamps)

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
