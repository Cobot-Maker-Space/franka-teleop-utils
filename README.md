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

## Components

### Receiver
Listens for updates from a leader robot and attempts to copy the motion.

### Sender
Publishes joint states and velocities.

## Message Format

[CapnProto](https://capnproto.org/) is used for message serialisation/deserialisation
during transmission and recording. See [messages/robot-state.capnp](messages/robot-state.capnp) 
for the message structure. Each message is 136 bytes.

## Configuration

```yaml
send:
  host: "224.3.29.71"
  port: 49186
  rate: 1000
receive:
  host: "224.3.29.71"
  port: 49186
robot:
  host: "172.16.0.2"
  autorecovery:
    enabled: true
    wait_time_ms: 500
  cutoff_frequency: 1000.0
  rate_limit: true
  collision_behaviour:
    lower_torque_thresholds:
      joint1: 10.0
      joint2: 10.0
      joint3: 10.0
      joint4: 10.0
      joint5: 10.0
      joint6: 10.0
      joint7: 10.0
    upper_torque_thresholds:
      joint1: 1000.0
      joint2: 1000.0
      joint3: 1000.0
      joint4: 1000.0
      joint5: 1000.0
      joint6: 1000.0
      joint7: 1000.0
    lower_force_thresholds:
      x: 10.0
      y: 10.0
      z: 10.0
      R: 10.0
      P: 10.0
      Y: 10.0
    upper_force_thresholds:
      x: 1000.0
      y: 1000.0
      z: 1000.0
      R: 1000.0
      P: 1000.0
      Y: 1000.0
  joint_impedance:
    joint1: 3000
    joint2: 3000
    joint3: 3000
    joint4: 2500
    joint5: 2500
    joint6: 2000
    joint7: 2000
  cartesian_impedance:
    x: 3000
    y: 3000
    z: 3000
    R: 300
    P: 300
    Y: 300
  stiffness:
    joint1: 600
    joint2: 600
    joint3: 600
    joint4: 600
    joint5: 250
    joint6: 150
    joint7: 50
  damping:
    joint1: 50
    joint2: 50
    joint3: 50
    joint4: 50
    joint5: 30
    joint6: 25
    joint7: 15
```
