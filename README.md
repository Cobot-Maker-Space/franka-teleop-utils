# franka-teleop-utils

Basic utility applications to support projects using Franka Emika Robots (née Panda), focused mostly around teleoperation tasks. This is very much a work-in-progress.

## Requirements

* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) - Version 3.x.
* [libfranka](https://github.com/frankaemika/libfranka) - Version 0.9.2.
* [CapnProto](https://capnproto.org/)
* [CMake](https://cmake.org) - Version 3.5.0 or greater.

## Building

* Create a directory in the source folder, e.g., `mkdir build`.
* Change to the build directory, `cd build`.
* Run cmake on the source, `cmake ..`.
* Run make to build the source, `make`.

## Core (C++) Components

### Publisher (publisher.cpp)

Publishes joint states and velocities from an arm.
By default the arm starts publishing from its current position. Pass `--home`
to first move to `robot.initial_position`.

### Subscriber (subscriber.cpp)

Listens for updates from a publisher and attempts to apply the joint states to
the connected arm.
By default the arm waits for the first received packet, moves to that joint
position, then starts following the stream. Pass `--home` to first move to
`robot.initial_position` before waiting for the first packet.
If a later target jumps farther than `robot.playback_reposition_threshold`
radians from the current joint position, the torque loop pauses, moves to the
new target with `MotionGenerator`, then resumes following. This lets batch
playback move between recordings without restarting the subscriber.

### PublisherSubscriber (publisher_subscriber.cpp)

Simulataneously publishes state whilst listening for and applying joint states.
By default the arm waits for the first received packet, moves to that joint
position, then starts following the stream. Pass `--home` to first move to
`robot.initial_position` before waiting for the first packet.
It uses the same `robot.playback_reposition_threshold` jump handling as
`subscriber`.

### Grasp (grasp.cpp)

Causes the gripper to grasp an object.

### Release (release.cpp)

Causes the gripper to release any grasped object.

### Teleop Utils (teleop_utils.cpp)

Common code for other components.

## Message Format

[CapnProto](https://capnproto.org/) is used for message serialisation/deserialisation
during transmission and recording. See [messages/robot-state.capnp](messages/robot-state.capnp) for the message structure. Each message is 136 bytes.

## Configuration

See `config.yml` for the full configuration. The utilities above accept the name of a configuration file with the `-c` parameter.

### Configuration Paramters

* send
  * host: IP address to send to
  * port: Port to send to
  * rate: Rate in Hz to attempt to send data at
* receive
  * host: IP address to receive on (0.0.0.0 for all local addresses)
  * port: Port to receive on
  * multicast_host: Multicast host to receive on (optional)
* robot:
  * host: Robot IP address
  * autorecovery:
  * enabled: Boolean, true to enable autorecovery
  * wait_time_ms: Time in milliseconds to wait before auto recovering

## Utility Scripts

These scripts were written quickly for the Embrace Angels contemporary deployment. A lot of values are hard-coded so some care will need to be taken when running them.

### convert.py

Takes a Cap'n'Proto encoded recording file and dumps it to CSV.

### kplayer.py

Takes a CSV file of joint positions (no header) and plays it back on an arm that is in subscriber mode.

### player.py

Used for Embrace Angels, takes two recordings and plays them back, one on each arm.
The first pose is republished for five seconds by default before timed playback
starts, giving subscribers time to move to the recording's start pose. Use
`--start-hold-seconds` and `--start-hold-rate` to tune this. When playing back
to a `publisher_subscriber`, use `--wait-for-start-feedback` to wait for robot
state feedback to reach the first pose before timed playback starts; `--iface`,
`--addr`, and `--maddr` select the feedback multicast interface.

### batch_playback.py

Plays multiple recordings sequentially from a text file containing one recording
folder name per line. Blank lines and lines starting with `#` are ignored. It
accepts the same arm-selection and first-pose hold/feedback options as
`player.py`, plus `--pause-seconds` between recordings.

### goto-last.py

Publishes only the final pose from a recording, so a subscriber can move to the
end position without replaying the whole file. Use `--bob-only` or
`--vincent-only` to target one arm, and `--seconds` / `--rate` to tune how long
the final pose is published.

### recorder.py

Used for Embrace Angels, makes recordings from both arms.
Command line arguments:

* -i / --iface: Network interface to listen on
* -a / --addr: Your IP address to listen on
* -m / -maddr: Multicast address to listen on

TODO: These values should be read from a configuration file, and in the case of interface name and IP address, automatically discovered.
