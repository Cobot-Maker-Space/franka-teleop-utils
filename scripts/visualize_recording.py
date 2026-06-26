import argparse
import pathlib
import sys
from dataclasses import dataclass

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
robotstate_capnp = None
plt = None

MESSAGE_SIZE = 248
BASE_PATH = SCRIPT_DIR.parent / "recordings"
JOINT_COUNT = 7


def load_dependencies():
    global plt, robotstate_capnp

    try:
        import capnp
    except ModuleNotFoundError:
        print(
            "Missing Python dependency 'capnp'. Install the script requirements before "
            "visualizing recordings.",
            file=sys.stderr,
        )
        sys.exit(1)

    try:
        import matplotlib.pyplot as matplotlib_pyplot
    except ModuleNotFoundError:
        print(
            "Missing Python dependency 'matplotlib'. Install the script requirements before "
            "visualizing recordings.",
            file=sys.stderr,
        )
        sys.exit(1)

    capnp.remove_import_hook()
    robotstate_capnp = capnp.load(str(SCRIPT_DIR / "robot-state.capnp"))
    plt = matplotlib_pyplot


@dataclass
class RecordingData:
    file: pathlib.Path
    elapsed_seconds: list[float]
    robot_times_ms: list[int]
    positions: list[list[float]]
    invalid_packets: int
    partial_packet_bytes: int


def recording_path(recording: str) -> pathlib.Path:
    path = pathlib.Path(recording)
    if path.is_absolute() or path.exists():
        return path
    return pathlib.Path(BASE_PATH, recording)


def joint_positions(state):
    return [
        state.joint1Pos,
        state.joint2Pos,
        state.joint3Pos,
        state.joint4Pos,
        state.joint5Pos,
        state.joint6Pos,
        state.joint7Pos,
    ]


def read_recording(file: pathlib.Path) -> RecordingData:
    elapsed_seconds = []
    robot_times_ms = []
    positions = []
    invalid_packets = 0
    partial_packet_bytes = 0
    base_time = None

    with open(file, "rb") as f:
        while True:
            offset = f.tell()
            packet = f.read(MESSAGE_SIZE)
            if not packet:
                break
            if len(packet) != MESSAGE_SIZE:
                partial_packet_bytes = len(packet)
                print(
                    f"{file}: skipped partial packet at byte {offset} "
                    f"({len(packet)} of {MESSAGE_SIZE} bytes)",
                    file=sys.stderr,
                )
                break

            try:
                with robotstate_capnp.RobotState.from_bytes(packet) as state:
                    timestamp_ms = int(state.time)
                    if base_time is None:
                        base_time = timestamp_ms
                    elapsed_seconds.append((timestamp_ms - base_time) / 1000.0)
                    robot_times_ms.append(timestamp_ms)
                    positions.append(joint_positions(state))
            except Exception as exc:
                invalid_packets += 1
                print(
                    f"{file}: skipped invalid packet at byte {offset}: {exc}",
                    file=sys.stderr,
                )

    if not positions:
        raise ValueError(f"{file} does not contain any valid robot-state packets")

    return RecordingData(
        file=file,
        elapsed_seconds=elapsed_seconds,
        robot_times_ms=robot_times_ms,
        positions=positions,
        invalid_packets=invalid_packets,
        partial_packet_bytes=partial_packet_bytes,
    )


def resolve_input(path: pathlib.Path, include_bob: bool, include_vincent: bool) -> list[pathlib.Path]:
    if path.is_file():
        return [path]

    if not path.is_dir():
        raise FileNotFoundError(f"{path} is not a file or recording folder")

    files = []
    if include_vincent:
        files.append(path / "vincent")
    if include_bob:
        files.append(path / "bob")

    missing = [str(file) for file in files if not file.is_file()]
    if missing:
        raise FileNotFoundError("Missing recording file(s): " + ", ".join(missing))

    return files


def read_requested_recordings(files: list[pathlib.Path], strict: bool) -> list[RecordingData]:
    recordings = []
    errors = []

    for file in files:
        try:
            recordings.append(read_recording(file))
        except Exception as exc:
            errors.append(f"{file}: {exc}")
            if strict:
                raise
            print(f"Skipping {file}: {exc}", file=sys.stderr)

    if not recordings:
        raise ValueError("No valid recording files found:\n  " + "\n  ".join(errors))

    return recordings


def movement_rates(data: RecordingData) -> tuple[list[float], list[float]]:
    rate_times = []
    rates = []
    for index in range(1, len(data.positions)):
        dt = data.elapsed_seconds[index] - data.elapsed_seconds[index - 1]
        if dt <= 0:
            continue
        max_delta = max(
            abs(current - previous)
            for current, previous in zip(data.positions[index], data.positions[index - 1])
        )
        rate_times.append(data.elapsed_seconds[index])
        rates.append(max_delta / dt)
    return rate_times, rates


def quiet_intervals(
    data: RecordingData,
    threshold: float,
    min_duration: float,
    gap_tolerance: float,
) -> list[tuple[float, float]]:
    rate_times, rates = movement_rates(data)
    raw_intervals = []
    start = None
    previous_time = None

    for time_s, rate in zip(rate_times, rates):
        if rate <= threshold:
            if start is None:
                start = previous_time if previous_time is not None else 0.0
        elif start is not None:
            end = previous_time if previous_time is not None else time_s
            raw_intervals.append((start, end))
            start = None
        previous_time = time_s

    if start is not None:
        raw_intervals.append((start, data.elapsed_seconds[-1]))

    intervals = []
    for start, end in raw_intervals:
        if intervals and start - intervals[-1][1] <= gap_tolerance:
            previous_start, _ = intervals[-1]
            intervals[-1] = (previous_start, end)
        else:
            intervals.append((start, end))

    return [
        (start, end)
        for start, end in intervals
        if end - start >= min_duration
    ]


def edge_trim_suggestions(
    data: RecordingData,
    intervals: list[tuple[float, float]],
) -> list[tuple[str, float, float]]:
    suggestions = []
    if not intervals:
        return suggestions

    duration = data.elapsed_seconds[-1]
    first_start, first_end = intervals[0]
    if first_start <= 0.001:
        suggestions.append(("start", first_start, first_end))

    last_start, last_end = intervals[-1]
    if duration - last_end <= 0.001:
        suggestions.append(("end", last_start, last_end))

    return suggestions


def print_summary(
    data: RecordingData,
    threshold: float,
    min_duration: float,
    gap_tolerance: float,
):
    intervals = quiet_intervals(data, threshold, min_duration, gap_tolerance)
    suggestions = edge_trim_suggestions(data, intervals)
    duration = data.elapsed_seconds[-1] if data.elapsed_seconds else 0.0
    print(f"\n{data.file}")
    print(f"  valid packets: {len(data.positions)}")
    print(f"  invalid packets skipped: {data.invalid_packets}")
    if data.partial_packet_bytes:
        print(f"  trailing partial bytes skipped: {data.partial_packet_bytes}")
    print(f"  duration: {duration:.3f}s")
    print(f"  robot time: {data.robot_times_ms[0]} to {data.robot_times_ms[-1]} ms")

    start_suggestion = next(
        (suggestion for suggestion in suggestions if suggestion[0] == "start"),
        None,
    )
    end_suggestion = next(
        (suggestion for suggestion in suggestions if suggestion[0] == "end"),
        None,
    )

    print("  suggested trim times (seconds):")
    if start_suggestion:
        _, _, end = start_suggestion
        print(f"    start: 0.000-{end:.3f}")
    else:
        print("    start: none")

    if end_suggestion:
        _, start, _ = end_suggestion
        print(f"    end: {start:.3f}-{duration:.3f}")
    else:
        print("    end: none")

    if not intervals:
        print(
            f"  no quiet intervals >= {min_duration:.3f}s "
            f"at <= {threshold:.6f} rad/s"
        )
        return []

    return intervals


def print_all_quiet_intervals(
    data: RecordingData,
    intervals: list[tuple[float, float]],
    threshold: float,
    min_duration: float,
):
    if not intervals:
        return

    print(
        f"  all quiet intervals >= {min_duration:.3f}s "
        f"at <= {threshold:.6f} rad/s:"
    )
    for start, end in intervals:
        print(f"    {start:.3f}-{end:.3f}s ({end - start:.3f}s)")


def plot_recording(
    data: RecordingData,
    axes,
    show_movement: bool,
    threshold: float,
    min_quiet_duration: float,
    gap_tolerance: float,
):
    title = data.file.name
    if data.file.parent.name:
        title = f"{data.file.parent.name}/{data.file.name}"

    position_axis = axes[0] if show_movement else axes
    for joint_index in range(JOINT_COUNT):
        joint_values = [sample[joint_index] for sample in data.positions]
        position_axis.plot(
            data.elapsed_seconds,
            joint_values,
            label=f"J{joint_index + 1}",
            linewidth=1.2,
        )

    position_axis.set_title(title)
    position_axis.set_ylabel("Joint position (rad)")
    position_axis.grid(True, alpha=0.3)
    position_axis.legend(loc="upper right", ncol=JOINT_COUNT)

    intervals = quiet_intervals(data, threshold, min_quiet_duration, gap_tolerance)
    for start, end in intervals:
        position_axis.axvspan(start, end, color="tab:green", alpha=0.08)

    if show_movement:
        rate_times, rates = movement_rates(data)
        movement_axis = axes[1]
        movement_axis.plot(rate_times, rates, color="black", linewidth=1.0)
        movement_axis.axhline(
            threshold,
            color="tab:red",
            linestyle="--",
            linewidth=1.0,
            label="quiet threshold",
        )
        movement_axis.set_ylabel("Max joint speed (rad/s)")
        movement_axis.grid(True, alpha=0.3)
        movement_axis.legend(loc="upper right")
        movement_axis.set_xlabel("Elapsed time (s)")
        for start, end in intervals:
            movement_axis.axvspan(start, end, color="tab:green", alpha=0.08)
    else:
        position_axis.set_xlabel("Elapsed time (s)")


def main(argv):
    parser = argparse.ArgumentParser(
        description="Plot recorded joint states over elapsed time for trim selection"
    )
    parser.add_argument(
        "recording",
        help="Recording file, recording folder, or folder name under recordings/",
    )
    parser.add_argument("--bob-only", action="store_true", help="Only plot bob from a folder")
    parser.add_argument(
        "--vincent-only",
        action="store_true",
        help="Only plot vincent from a folder",
    )
    parser.add_argument(
        "--six-joints",
        action="store_true",
        help="Plot joints 1-6 only instead of all 7 joints",
    )
    parser.add_argument(
        "--no-movement",
        action="store_true",
        help="Do not show the max-joint-speed subplot",
    )
    parser.add_argument(
        "--quiet-threshold",
        type=float,
        default=0.02,
        help="Max joint speed in rad/s considered quiet",
    )
    parser.add_argument(
        "--quiet-gap-tolerance",
        type=float,
        default=0.25,
        help="Merge quiet intervals separated by this many seconds or less",
    )
    parser.add_argument(
        "--min-quiet-duration",
        type=float,
        default=1.0,
        help="Minimum quiet interval duration to print in seconds",
    )
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        help="Save the plot to this image file instead of opening an interactive window",
    )
    parser.add_argument(
        "--show-all-quiet",
        action="store_true",
        help="Print all low-movement intervals, including ones in the middle",
    )
    args = parser.parse_args(argv[1:])

    if args.bob_only and args.vincent_only:
        parser.error("--bob-only and --vincent-only cannot be used together")
    if args.quiet_threshold < 0:
        parser.error("--quiet-threshold must be >= 0")
    if args.min_quiet_duration < 0:
        parser.error("--min-quiet-duration must be >= 0")
    if args.quiet_gap_tolerance < 0:
        parser.error("--quiet-gap-tolerance must be >= 0")

    load_dependencies()

    global JOINT_COUNT
    JOINT_COUNT = 6 if args.six_joints else 7

    path = recording_path(args.recording)
    strict = path.is_file() or args.bob_only or args.vincent_only
    files = resolve_input(
        path,
        include_bob=not args.vincent_only,
        include_vincent=not args.bob_only,
    )
    recordings = read_requested_recordings(files, strict)

    for data in recordings:
        intervals = print_summary(
            data,
            args.quiet_threshold,
            args.min_quiet_duration,
            args.quiet_gap_tolerance,
        )
        if args.show_all_quiet and intervals:
            print_all_quiet_intervals(
                data,
                intervals,
                args.quiet_threshold,
                args.min_quiet_duration,
            )

    show_movement = not args.no_movement
    rows_per_recording = 2 if show_movement else 1
    figure, axes = plt.subplots(
        len(recordings) * rows_per_recording,
        1,
        sharex=False,
        figsize=(14, 4.5 * len(recordings) * rows_per_recording),
        constrained_layout=True,
    )

    if len(recordings) * rows_per_recording == 1:
        axes = [axes]

    for index, data in enumerate(recordings):
        start = index * rows_per_recording
        plot_axes = axes[start : start + rows_per_recording]
        if not show_movement:
            plot_axes = plot_axes[0]
        plot_recording(
            data,
            plot_axes,
            show_movement,
            args.quiet_threshold,
            args.min_quiet_duration,
            args.quiet_gap_tolerance,
        )

    if args.output:
        figure.savefig(args.output, dpi=150)
        print(f"\nSaved plot to {args.output}")
    else:
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
