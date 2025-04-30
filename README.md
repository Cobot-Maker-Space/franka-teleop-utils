# franka-utils
Basic utility applications to support projects using Franka Emika Robots (née Panda).

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

**Usage**: ./follower &lt;robot_ip&gt; &lt;listen_port&gt;
  * &lt;robot_ip&gt;: IP or hostname of the robot.
  * &lt;listen_port&gt;: Port number to listen for leader messages on.

### Leader
Publishes joint states and velocities.

**Usage**: ./leader &lt;robot_ip&gt; &lt;follower_ip&gt; &lt;follower_port&gt; 
  * &lt;robot_ip&gt;: IP or hostname of the robot.
  * &lt;follower_ip&gt;: IP or hostname of a follower application instance, or a multicast address.
  * &lt;follower_port&gt;: Port number follower applications instance listens on.

### Player
Plays back a recording file.

**Usage**: ./player &lt;robot_ip&gt; &lt;filename&gt;
  * &lt;robot_ip&gt;: IP or hostname of the robot.
  * &lt;filename&gt;: File path to read data from.

### Recorder
Records joints states and velocities to a file.

**Usage**: ./recorder &lt;robot_ip&gt; &lt;filename&gt;
  * &lt;robot_ip&gt;: IP or hostname of the robot.
  * &lt;filename&gt;: File path to write data to.

### Recording2CSV
Converts recorded files from binary to CSV format.

**Usage**: ./recording2csv &lt;infile&gt; &lt;outfile&gt;
  * &lt;infile&gt;: Recording file to read data from.
  * &lt;outfile&gt;: CSV file to write to.

## Message Format

[CapnProto](https://capnproto.org/) is used for message serialisation/deserialisation
during transmission and recording. See [messages/robot-state.capnp](messages/robot-state.capnp) 
for the message structure. Each message is 120 bytes. When writing to a file, messages
are stored contiguously.

