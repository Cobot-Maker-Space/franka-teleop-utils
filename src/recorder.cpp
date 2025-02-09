/*
franka-utils
Copyright (C) 2025  Cobot Maker Space, University of Nottinghm

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <array>
#include <atomic>
#include <csignal>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <mutex>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

#include <franka/exception.h>
#include <franka/robot.h>

#include <capnp/message.h>
#include <capnp/serialize.h>

#include "examples_common.h"
#include "messages/robot-state.capnp.h"

namespace {
  std::function<void(int)> stop;
  void signal_handler(int signal) { stop(signal); }
}

/*
 * Message publishing rate (Hz)
 */
const double rate = 10.0; // TODO: Should be an optional command line argument

int main(int argc, const char** argv) {

  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot_ip> <filename>" << std::endl;
    return -1;
  }

  capnp::MallocMessageBuilder message;
  struct {
    std::mutex lock;
    RobotState::Builder state_builder;
    bool updated;
  } robot_data{
    .state_builder = message.initRoot<RobotState>()
  };

  int outfile;
  mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP;
  if (outfile = open(argv[2], O_WRONLY | O_CREAT, mode) == -1) {
    std::cerr << "Unable to open file " << argv[2] << " for writing." << std::endl;
    return -1;
  }

  std::atomic_bool running{ true };
  stop = [&running](int signum) -> void { running = false; };

  std::thread write_thread([&message, &robot_data, outfile, &running]() {
    while (running) {
      std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>((1.0 / rate * 1000.0))));
      if (robot_data.lock.try_lock()) {
        if (robot_data.updated) {
          capnp::writeMessageToFd(outfile, message);
          robot_data.updated = false;
        }
        robot_data.lock.unlock();
      }
    }
    });

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // TODO: Read vales from a file provided as a 'profile' argument
    robot.setCollisionBehavior(
      { {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} },
      { {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} },
      { {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} },
      { {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} },
      { {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} },
      { {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} },
      { {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} },
      { {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} });

    const franka::Torques torques = franka::Torques(
      std::array<double, 7>{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      control_callback = [&robot_data, &running, &torques](
        const franka::RobotState& state, franka::Duration) -> franka::Torques {
          if (!running) {
            return franka::MotionFinished(torques);
          }

          if (robot_data.lock.try_lock()) {
            robot_data.updated = true;
            robot_data.state_builder.setTime(state.time.toMSec());
            robot_data.state_builder.setJoint1Pos(state.q[0]);
            robot_data.state_builder.setJoint2Pos(state.q[1]);
            robot_data.state_builder.setJoint3Pos(state.q[2]);
            robot_data.state_builder.setJoint4Pos(state.q[3]);
            robot_data.state_builder.setJoint5Pos(state.q[4]);
            robot_data.state_builder.setJoint6Pos(state.q[5]);
            robot_data.state_builder.setJoint7Pos(state.q[6]);
            robot_data.state_builder.setJoint1Vel(state.dq[0]);
            robot_data.state_builder.setJoint2Vel(state.dq[1]);
            robot_data.state_builder.setJoint3Vel(state.dq[2]);
            robot_data.state_builder.setJoint4Vel(state.dq[3]);
            robot_data.state_builder.setJoint5Vel(state.dq[4]);
            robot_data.state_builder.setJoint6Vel(state.dq[5]);
            robot_data.state_builder.setJoint7Vel(state.dq[6]);
            robot_data.lock.unlock();
          }
          return torques;
      };

    std::cout << "Robot is ready, press Enter to start." << std::endl;
    std::cin.ignore();
    std::signal(SIGINT, signal_handler);
    std::cout << "Robot running, press CTRL-c to stop." << std::endl;
    robot.control(control_callback);

  }
  catch (const franka::Exception& ex) {
    running = false;
    std::cerr << "Robot has encountered an error and will stop." << std::endl
      << ex.what() << std::endl;
  }

  if (write_thread.joinable()) {
    write_thread.join();
  }

  if (close(outfile) == -1) {
    std::cerr
      << "There was an error closing the output file, some data may be missing."
      << std::endl;
  }

  return 0;
}
