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

### Follower
Listens for updates from a leader robot and attempts to copy the motion.

### Publisher
Publishes joint states and velocities.

## Message Format

[CapnProto](https://capnproto.org/) is used for message serialisation/deserialisation
during transmission and recording. See [messages/robot-state.capnp](messages/robot-state.capnp) 
for the message structure. Each message is 136 bytes. When writing to a file, messages
are stored contiguously.

