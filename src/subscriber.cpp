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
#include <iostream>

#include <franka/exception.h>
#include <franka/model.h>

#include "motion_generator.h"
#include "teleop_utils.h"

#include "messages/robot-state.capnp.h"

namespace {
  std::function<void(int)> stop;
  void signal_handler(int signal) { stop(signal); }
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

  auto control_callback = [
    &damping, &model, &leader_pos, &leader_vel, &stiffness, &thread_data, &torques](
      const franka::RobotState& state, franka::Duration) -> franka::Torques {

        if (!thread_data.running) {
          return franka::MotionFinished(franka::Torques(
            std::array<double, 7>{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}));
        }

        if (thread_data.lock.try_lock()) {
          if (thread_data.updated == true) {
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

    std::cout << "Press enter to move robot to the start position." << std::endl;
    std::cin.ignore();
    std::array<double, 7> initial_pos = { {
      config["robot"]["initial_position"]["joint1"].as<double>(),
      config["robot"]["initial_position"]["joint2"].as<double>(),
      config["robot"]["initial_position"]["joint3"].as<double>(),
      config["robot"]["initial_position"]["joint4"].as<double>(),
      config["robot"]["initial_position"]["joint5"].as<double>(),
      config["robot"]["initial_position"]["joint6"].as<double>(),
      config["robot"]["initial_position"]["joint7"].as<double>()
    } };
    robot.control(MotionGenerator(
      config["robot"]["initial_position"]["speed_factor"].as<double>(),
      initial_pos));
    std::cout << "Robot ready, press enter to start." << std::endl;
    std::cin.ignore();
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
