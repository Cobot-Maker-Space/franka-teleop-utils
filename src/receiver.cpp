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
#include <array>
#include <atomic>
#include <csignal>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <sys/stat.h>
#include <unistd.h>

#include <capnp/message.h>
#include <capnp/serialize.h>

#include <cxxopts.hpp>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <yaml-cpp/yaml.h>

#include "teleop_utils.h"

#include "messages/robot-state.capnp.h"

// https://linux.die.net/man/3/sigwait
namespace {
  std::function<void(int)> stop;
  void signal_handler(int signal) { stop(signal); }
}

bool in_multicast(in_addr_t address) {
  return (((long int)address) & 0xf0000000) == 0xe0000000;
}

// TODO: Incorporate gripper state (if attached)
int main(int argc, const char** argv) {

  cxxopts::Options options("sender", "Send robot state to receiver(s)");
  options.add_options()
    ("c,config-file", "Path to configuration file", cxxopts::value<std::string>());
  auto poptions = options.parse(argc, argv);
  YAML::Node config;
  if (poptions.count("config-file")) {
    config = YAML::LoadFile(poptions["config-file"].as<std::string>());
  }
  else {
    config = YAML::Load(std::cin);
  }

  struct {
    std::mutex lock;
    std::atomic_bool running{ true };
    bool updated = false;
    std::array<double, 7> leader_pos = { 0, 0, 0, 0, 0, 0, 0 };
    std::array<double, 7> leader_vel = { 0, 0, 0, 0, 0, 0, 0 };
#ifdef REPORT_RATE
    int counter = 0;
#endif
  } thread_data{};

  int sock;
  if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    std::cerr << "Could not create socket." << std::endl;
    exit(EXIT_FAILURE);
  }
  int reuse = 1;
  if ((setsockopt(sock, SOL_SOCKET, SO_REUSEADDR,
    (char*)&reuse, sizeof(reuse))) < 0) {
    std::cerr << "Could not set socket to reusable." << std::endl;
    exit(EXIT_FAILURE);
  }

  stop = [&sock, &thread_data](int) -> void {
    thread_data.running = false;
    shutdown(sock, SHUT_RDWR);
    };

#ifdef REPORT_RATE
  std::thread report_thread([&thread_data]() {
    std::cout << std::endl;
    while (thread_data.running) {
      if (thread_data.lock.try_lock()) {
        std::cout << "\33[2K\rUpdate rate: " << thread_data.counter << "hz" << std::endl;
        thread_data.counter = 0;
      }
      thread_data.lock.unlock();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return EXIT_SUCCESS;
    });
#endif

  std::thread receive_thread([&config, &sock, &thread_data]() {
    double last_received_time = 0.0;
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(config["listen"]["port"].as<u_short>());
    if (config["listen"]["host"]) {
      in_addr_t host = inet_addr(
        config["listen"]["host"].as<std::string>().c_str());
      if (in_multicast(host)) {
        addr.sin_addr.s_addr = INADDR_ANY;
        struct ifaddrs* ifaddr;
        if (getifaddrs(&ifaddr) == -1) {
          std::cerr << "Unable to enumerate network interfaces." << std::endl;
          exit(EXIT_FAILURE);
        }
        char addr_s[NI_MAXHOST];
        for (struct ifaddrs* ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
          if (ifa->ifa_addr != NULL && ifa->ifa_addr->sa_family == AF_INET) {
            ip_mreq group = {};
            group.imr_multiaddr.s_addr = host;
            getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
              addr_s, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
            group.imr_interface.s_addr = inet_addr(addr_s);
            setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&group, sizeof(group));
          }
        }
        freeifaddrs(ifaddr);
      }
      else {
        addr.sin_addr.s_addr = host;
      }
    }
    else {
      addr.sin_addr.s_addr = INADDR_ANY;
    }


    // TODO: Multicast setup if needed - https://gist.github.com/dksmiffs/d67afe6bda973d67752ae63dc49a7310
    socklen_t addr_len = sizeof(addr);
    if (bind(sock, (struct sockaddr*)&addr, addr_len) < 0) {
      std::cerr << "Could not bind to port." << std::endl;
      return EXIT_FAILURE;
    }
    kj::byte buffer[MESSAGE_SIZE];
    memset(&buffer, 0, MESSAGE_SIZE);
    kj::ArrayPtr<kj::byte> buffer_array(buffer, MESSAGE_SIZE);
    while (thread_data.running) {
      if (recvfrom(sock, buffer, MESSAGE_SIZE, 0,
        (struct sockaddr*)&addr, &addr_len) <= 0) {
        std::cerr << "Error during message reception." << std::endl;
        continue;
      }

      kj::ArrayInputStream in(buffer_array);
      capnp::InputStreamMessageReader reader(in);
      RobotState::Reader state = reader.getRoot<RobotState>();

      if (state.getTime() > last_received_time && thread_data.lock.try_lock()) {
        thread_data.updated = true;
        last_received_time = state.getTime();
        thread_data.leader_pos[0] = state.getJoint1Pos();
        thread_data.leader_pos[1] = state.getJoint2Pos();
        thread_data.leader_pos[2] = state.getJoint3Pos();
        thread_data.leader_pos[3] = state.getJoint4Pos();
        thread_data.leader_pos[4] = state.getJoint5Pos();
        thread_data.leader_pos[5] = state.getJoint6Pos();
        thread_data.leader_pos[6] = state.getJoint7Pos();
        thread_data.leader_vel[0] = state.getJoint1Vel();
        thread_data.leader_vel[1] = state.getJoint2Vel();
        thread_data.leader_vel[2] = state.getJoint3Vel();
        thread_data.leader_vel[3] = state.getJoint4Vel();
        thread_data.leader_vel[4] = state.getJoint5Vel();
        thread_data.leader_vel[5] = state.getJoint6Vel();
        thread_data.leader_vel[6] = state.getJoint7Vel();
        thread_data.lock.unlock();
      }

    }
    return EXIT_SUCCESS;
    });

  franka::Robot robot(config["robot"]["host"].as<std::string>());
  try {
    configure_robot(robot, config);
    franka::Model model = robot.loadModel();
    const bool rate_limit = config["robot"]["rate_limit"].as<bool>();
    const double cutoff_freq = config["robot"]["cutoff_frequency"].as<double>();
    const bool autorecover = config["robot"]["autorecovery"]["enabled"].as<bool>();
    const long long autorecover_wait_time =
      config["robot"]["autorecovery"]["wait_time_ms"].as<long long>();
    std::array<double, 7> stiffness = { {
      config["robot"]["stiffness"]["joint1"].as<double>(),
      config["robot"]["stiffness"]["joint2"].as<double>(),
      config["robot"]["stiffness"]["joint3"].as<double>(),
      config["robot"]["stiffness"]["joint4"].as<double>(),
      config["robot"]["stiffness"]["joint5"].as<double>(),
      config["robot"]["stiffness"]["joint6"].as<double>(),
      config["robot"]["stiffness"]["joint7"].as<double>()} };
    std::array<double, 7> damping = { {
      config["robot"]["damping"]["joint1"].as<double>(),
      config["robot"]["damping"]["joint2"].as<double>(),
      config["robot"]["damping"]["joint3"].as<double>(),
      config["robot"]["damping"]["joint4"].as<double>(),
      config["robot"]["damping"]["joint5"].as<double>(),
      config["robot"]["damping"]["joint6"].as<double>(),
      config["robot"]["damping"]["joint7"].as<double>()} };
    std::array<double, 7> torques = { 0, 0, 0, 0, 0, 0, 0 };

    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      control_callback = [&damping, &model, &stiffness, &thread_data, &torques](
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
                  (thread_data.leader_pos[i] - state.q[i])
                  - damping[i] * state.dq[i] + coriolis[i];
              }

              /*for (size_t i = 0; i < 7; i++) {
                torques[i] =
                  stiffness[i] *
                  (thread_data.leader_pos[i] - state.q[i])
                  + damping[i]
                  * (thread_data.leader_vel[i] - state.dq[i]);
              }*/

              thread_data.updated = false;
              thread_data.lock.unlock();
              return torques;
            }
            thread_data.lock.unlock();
          }

          return torques;
      };

    std::signal(SIGINT, signal_handler);
    std::cout << "Follower running, press CTRL-c to stop." << std::endl;

    while (thread_data.running) {
      try {
        robot.control(control_callback, rate_limit, cutoff_freq);
      }
      catch (const franka::Exception& ex) {
        std::cerr << "Robot has encountered an error." << std::endl
          << ex.what() << std::endl;
        if (autorecover) {
          std::cout << "Autorecover is true, robot will restart." << std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds(autorecover_wait_time));
          robot.automaticErrorRecovery();
        }
        else {
          std::cout << "Autorecover is false, robot will stop." << std::endl;
          thread_data.running = false;
        }
      }
    }

  }
  catch (const franka::Exception& ex) {
    thread_data.running = false;
    std::cerr << "Robot has encountered an error and will stop." << std::endl
      << ex.what() << std::endl;
  }

  if (receive_thread.joinable()) {
    receive_thread.join();
  }

  if (close(sock) == -1) {
    std::cerr << "There was an error closing the socket." << std::endl;
  }

  return EXIT_SUCCESS;
}
