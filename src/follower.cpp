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
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <franka/exception.h>
#include <franka/robot.h>

#include <capnp/message.h>
#include <capnp/serialize.h>

#include "examples_common.h"
#include "ioutils.h"
#include "messages/robot-state.capnp.h"

namespace {
  std::function<void(int)> stop;
  void signal_handler(int signal) { stop(signal); }
}

/*
 * Message size in bytes. Calculated by producing a message and checking size
 * in sample code.
 */
const size_t MESSAGE_SIZE = 136;

int main(int argc, const char** argv) {

  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot_ip> <listen_port>" << std::endl;
    return -1;
  }

  capnp::MallocMessageBuilder message;
  struct {
    std::mutex lock;
    bool updated;
  } robot_data{};

  int sock;
  if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    std::cerr << "Could not create socket." << std::endl;
    exit(EXIT_FAILURE);
  }

  std::atomic_bool running{ true };
  stop = [&running](int signum) -> void { running = false; };

  std::array<double, 7> current_leader_position = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> current_leader_velocity = {0, 0, 0, 0, 0, 0, 0};
  double last_received_time = 0.0;

  std::thread receive_thread([argv, &message, &robot_data, sock, &running, &last_received_time, &current_follower_position]() {
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(std::stoi(argv[2]));
    addr.sin_addr.s_addr = INADDR_ANY;
    socklen_t addr_len = sizeof(addr);
    if (bind(sock, (struct sockaddr*)&addr, addr_len) < 0) {
      std::cerr << "Could not bind to port." << std::endl;
      return EXIT_FAILURE;
    }
    kj::byte buffer[MESSAGE_SIZE];
    memset(&buffer, 0, MESSAGE_SIZE);
    kj::ArrayPtr<kj::byte> buffer_array(buffer, MESSAGE_SIZE);
    while (running) {
      if (recvfrom(sock, buffer, MESSAGE_SIZE, 0,
        (struct sockaddr*)&addr, &addr_len) < 0) {
        std::cerr << "Error during message reception." << std::endl;
        continue;
      }

      kj::ArrayInputStream in(buffer_array);
      capnp::InputStreamMessageReader reader(in);
      RobotState::Reader state = reader.getRoot<RobotState>();

      std::cout << state.getTime() << ", "
      << std::setw(5) << state.getJoint1Pos() << ", "
      << std::setw(5) << state.getJoint2Pos() << ", "
      << std::setw(5) << state.getJoint3Pos() << ", "
      << std::setw(5) << state.getJoint4Pos() << ", "
      << std::setw(5) << state.getJoint5Pos() << ", "
      << std::setw(5) << state.getJoint6Pos() << ", "
      << std::setw(5) << state.getJoint7Pos() << ", "
      << std::setw(5) << state.getJoint1Vel() << ", "
      << std::setw(5) << state.getJoint2Vel() << ", "
      << std::setw(5) << state.getJoint3Vel() << ", "
      << std::setw(5) << state.getJoint4Vel() << ", "
      << std::setw(5) << state.getJoint5Vel() << ", "
      << std::setw(5) << state.getJoint6Vel() << ", "
      << std::setw(5) << state.getJoint7Vel() << std::endl;

      // TODO: Check state time to make sure it's newer than the previous
      // message. If so, publish the state so that the control thread can
      // make the movements
      if (state.getTime() > last_received_time) {
        if (robot_data.lock.try_lock()) {
          robot_data.updated = true;
          last_received_time = state.getTime();
          current_leader_position = {
            state.getJoint1Pos(),
            state.getJoint2Pos(),
            state.getJoint3Pos(),
            state.getJoint4Pos(),
            state.getJoint5Pos(),
            state.getJoint6Pos(),
            state.getJoint7Pos()
          };
          current_leader_velocity = {
            state.getJoint1Vel(),
            state.getJoint2Vel(),
            state.getJoint3Vel(),
            state.getJoint4Vel(),
            state.getJoint5Vel(),
            state.getJoint6Vel(),
            state.getJoint7Vel()
          };
          robot_data.lock.unlock();
        }
      }
      
    }
    });

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // TODO: Read values from a file provided as a 'profile' argument
    robot.setCollisionBehavior(
      { {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} },
      { {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} },
      { {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} },
      { {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} },
      { {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} },
      { {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} },
      { {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} },
      { {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} });

    const franka::Torques torques_stop = franka::Torques(
      std::array<double, 7>{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      control_callback = [&robot_data, &running, &torques_stop, &current_leader_position, &current_leader_velocity](
        const franka::RobotState& state, franka::Duration) -> franka::Torques {
          if (!running) {
            return franka::MotionFinished(torques_stop);
          }

          if (robot_data.lock.try_lock()) {
            if (robot_data.updated == true) {
              // TODO: Calculate torques

              std::vector<double> Kp = {30, 30, 30, 30, 15, 15, 15};
              std::vector<double> Kd = {0.5, 0.5, 0.5, 0.5, 0.25, 0.25, 0.25};
              std::array<double, 7> calculated_torque = {0, 0, 0, 0, 0, 0, 0};

              for (int i = 0; i < _fstate.q.size(); i++)
              {
                  calculated_torque[i] = Kp[i] * (current_leader_position[i] - state.q[i])  + Kd[i] *(current_leader_velocity[i] - state.dq[i]);
              }

              franka::Torques torques_new(calculated_torque);

              robot_data.updated = false;
            }
            robot_data.lock.unlock();
          }
          return torques_new;
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

  if (receive_thread.joinable()) {
    receive_thread.join();
  }

  if (close(sock) == -1) {
    std::cerr
      << "There was an error closing the socket."
      << std::endl;
  }

  return 0;
}
