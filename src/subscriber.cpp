/*
franka-teleop-utils
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
#include <algorithm>
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/model.h>

#include "motion_generator.h"
#include "teleop_utils.h"

#include "messages/robot-state.capnp.h"

namespace {
  std::function<void(int)> stop;
  void signal_handler(int signal) { stop(signal); }

  double max_joint_error(
    const std::array<double, 7>& target,
    const std::array<double, 7>& actual) {
    double max_error = 0.0;
    for (size_t i = 0; i < target.size(); i++) {
      max_error = std::max(max_error, std::abs(target[i] - actual[i]));
    }
    return max_error;
  }

  std::array<double, 7> current_joint_position(const franka::RobotState& state) {
    return { {
      state.q[0],
      state.q[1],
      state.q[2],
      state.q[3],
      state.q[4],
      state.q[5],
      state.q[6]
    } };
  }
}

// TODO: Incorporate gripper state (if attached)
int main(int argc, const char** argv) {

  struct thread_data thread_data {};
  YAML::Node config = parse_options(argc, argv);

  asio::io_context io_ctx;
  using asio::ip::udp;
  auto port = config["subscribe"]["port"].as<asio::ip::port_type>();
  udp::endpoint endpoint =
    config["subscribe"]["host"]
    ? udp::endpoint(asio::ip::address::from_string(
      config["subscribe"]["host"].as<std::string>()), port)
    : udp::endpoint(udp::v4(), port);
  udp::socket socket(io_ctx);
  socket.open(udp::v4());
  socket.set_option(udp::socket::reuse_address(true));
  socket.bind(endpoint);
  if (config["subscribe"]["multicast_host"]) {
    socket.set_option(
      asio::ip::multicast::join_group(
        asio::ip::address::from_string(
          config["subscribe"]["multicast_host"].as<std::string>())));
  }

  stop = [&socket, &thread_data](int) -> void {
    thread_data.running = false;
    socket.shutdown(asio::socket_base::shutdown_both);
    socket.close();
    };
  std::signal(SIGINT, signal_handler);

  std::array<double, 7> leader_pos = { 0, 0, 0, 0, 0, 0, 0 };
  std::array<double, 7> leader_vel = { 0, 0, 0, 0, 0, 0, 0 };

  std::thread subscribe_thread(SubscribeThread{
    leader_pos, leader_vel, socket, thread_data });

  franka::Robot robot(config["robot"]["host"].as<std::string>());
  configure_robot(config, robot);
  auto model = robot.loadModel();
  const std::array<double, 7> stiffness = { {
    config["robot"]["stiffness"]["joint1"].as<double>(),
    config["robot"]["stiffness"]["joint2"].as<double>(),
    config["robot"]["stiffness"]["joint3"].as<double>(),
    config["robot"]["stiffness"]["joint4"].as<double>(),
    config["robot"]["stiffness"]["joint5"].as<double>(),
    config["robot"]["stiffness"]["joint6"].as<double>(),
    config["robot"]["stiffness"]["joint7"].as<double>()} };
  const std::array<double, 7> damping = { {
    config["robot"]["damping"]["joint1"].as<double>(),
    config["robot"]["damping"]["joint2"].as<double>(),
    config["robot"]["damping"]["joint3"].as<double>(),
    config["robot"]["damping"]["joint4"].as<double>(),
    config["robot"]["damping"]["joint5"].as<double>(),
    config["robot"]["damping"]["joint6"].as<double>(),
    config["robot"]["damping"]["joint7"].as<double>()} };
  std::array<double, 7> torques = { 0, 0, 0, 0, 0, 0, 0 };
  std::atomic_bool reposition_requested{ false };
  const double reposition_threshold =
    config["robot"]["playback_reposition_threshold"]
    ? config["robot"]["playback_reposition_threshold"].as<double>()
    : 0.5;

  auto control_callback = [
    &damping, &model, &leader_pos, &leader_vel, &reposition_requested,
    &reposition_threshold, &stiffness, &thread_data, &torques](
      const franka::RobotState& state, franka::Duration) -> franka::Torques {

        if (!thread_data.running) {
          return franka::MotionFinished(franka::Torques(
            std::array<double, 7>{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}));
        }

        if (thread_data.lock.try_lock()) {
          if (thread_data.updated == true) {
            if (max_joint_error(leader_pos, current_joint_position(state)) > reposition_threshold) {
              reposition_requested = true;
              thread_data.lock.unlock();
              return franka::MotionFinished(franka::Torques(
                std::array<double, 7>{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}));
            }

#ifdef REPORT_RATE
            thread_data.counter++;
#endif
            std::array<double, 7> coriolis = model.coriolis(state);
            for (size_t i = 0; i < 7; i++) {
              torques[i] =
                stiffness[i] *
                (leader_pos[i] - state.q[i])
                - damping[i] * state.dq[i] + coriolis[i];
            }

            //thread_data.updated = false;
            thread_data.lock.unlock();
            return torques;
          }
          thread_data.lock.unlock();
        }

        return torques;
    };

    if (should_home_on_start(config)) {
      std::cout << "Press enter to move robot to the configured start position." << std::endl;
      std::cin.ignore();
      robot.control(MotionGenerator(
        config["robot"]["initial_position"]["speed_factor"].as<double>(),
        configured_initial_position(config)));
    }
    std::cout << "Robot ready, press enter to start." << std::endl;
    std::cin.ignore();
    
    std::cout << "Waiting for first packet from leader..." << std::endl;
    while (thread_data.running && !thread_data.first_packet_received) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    if (!thread_data.running) {
      std::cout << "Stopped before receiving first packet." << std::endl;
      return EXIT_FAILURE;
    }
    
    std::array<double, 7> first_position;
    {
      std::lock_guard<std::mutex> lock(thread_data.lock);
      first_position = leader_pos;
    }
    
    std::cout << "Moving to first leader position..." << std::endl;
    robot.control(MotionGenerator(
      config["robot"]["initial_position"]["speed_factor"].as<double>(),
      first_position));
    
    std::cout << "Robot running, press CTRL-c to stop." << std::endl;

#ifdef REPORT_RATE
    std::thread report_thread(ReportThread{ "Subscriber", thread_data });
#endif

    const bool rate_limit = config["robot"]["rate_limit"].as<bool>();
    const double cutoff_freq = config["robot"]["cutoff_frequency"].as<double>();
    const bool autorecover = config["robot"]["autorecovery"]["enabled"].as<bool>();
    const long long autorecover_wait_time =
      config["robot"]["autorecovery"]["wait_time_ms"].as<long long>();

    while (thread_data.running) {
      try {
        robot.control(control_callback, rate_limit, cutoff_freq);
        if (reposition_requested && thread_data.running) {
          std::array<double, 7> reposition_target;
          {
            std::lock_guard<std::mutex> lock(thread_data.lock);
            reposition_target = leader_pos;
          }
          torques = { 0, 0, 0, 0, 0, 0, 0 };
          reposition_requested = false;
          std::cout << "Target jump detected, moving to new playback start position..." << std::endl;
          robot.control(MotionGenerator(
            config["robot"]["initial_position"]["speed_factor"].as<double>(),
            reposition_target));
          std::cout << "Playback following resumed." << std::endl;
        }
      }
      catch (const franka::Exception& ex) {
        std::cerr << "Error: " << std::endl << ex.what() << std::endl;
        if (autorecover) {
          std::cout << "Recovering in " << autorecover_wait_time << "ms." << std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds(autorecover_wait_time));
          robot.automaticErrorRecovery();
        }
        else {
          std::cout << "Stopping..." << std::endl;
          thread_data.running = false;
        }
      }
    }

    if (subscribe_thread.joinable()) {
      subscribe_thread.join();
    }
#ifdef REPORT_RATE
    if (report_thread.joinable()) {
      report_thread.join();
    }
#endif
    if (socket.is_open()) {
      socket.close();
    }

    return EXIT_SUCCESS;
}
