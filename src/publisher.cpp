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
#include <sys/stat.h>
#include <unistd.h>

#include <capnp/message.h>
#include <capnp/serialize.h>

#include <franka/exception.h>
#include <franka/robot.h>

#include <yaml-cpp/yaml.h>

#include "memory_outputstream.h"
#include "teleop_utils.h"

#include "messages/robot-state.capnp.h"

namespace {
	std::function<void(int)> stop;
	void signal_handler(int signal) { stop(signal); }
}

// TODO: Incorporate gripper state (if attached)
int main(int, const char**) {

	YAML::Node config = YAML::Load(std::cin);
	const double rate = config["publish"]["rate"].as<double>();
	const bool autorecover = config["robot"]["autorecovery"]["enabled"].as<bool>();
	const long long autorecover_wait_time =
		config["robot"]["autorecovery"]["wait_time_ms"].as<long long>();

	capnp::MallocMessageBuilder message;
	struct {
		std::mutex lock;
		RobotState::Builder state_builder;
		bool updated = false;
	} robot_data{
		.state_builder = message.initRoot<RobotState>()
	};

	int sock;
	if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		std::cerr << "Could not create socket." << std::endl;
		exit(EXIT_FAILURE);
	}

	std::atomic_bool running{ true };
	stop = [&running](int) -> void { running = false; };

	std::thread publish_thread([config, &message, &robot_data, sock, &rate, &running]() {
		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_port = htons(config["publish"]["port"].as<u_short>());
		addr.sin_addr.s_addr = inet_addr(config["publish"]["host"].as<std::string>().c_str());
		socklen_t addr_len = sizeof(addr);
		kj::byte buffer[MESSAGE_SIZE];
		memset(buffer, 0, MESSAGE_SIZE);
		while (running) {
			std::this_thread::sleep_for(
				std::chrono::milliseconds(static_cast<int>((1.0 / rate * 1000.0))));
			if (robot_data.lock.try_lock()) {
				if (robot_data.updated) {
					MemoryOutputStream os(buffer);
					capnp::writeMessage(os, message);
					if (sendto(sock, buffer, os.getSize(), MSG_CONFIRM,
						(const struct sockaddr*)&addr, addr_len) < 0) {
						std::cerr << "An error occurred during message transmission" << std::endl;
					}
					robot_data.updated = false;
				}
				robot_data.lock.unlock();
			}
		}
		return EXIT_SUCCESS;
		});

	try {
		franka::Robot robot(config["robot"]["host"].as<std::string>());
		configure_robot(robot, config);

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

		// TODO: Wait for SIGUSR1 instread of waiting for enter
		std::cout << "Robot is ready, press Enter to start." << std::endl;
		std::cin.ignore();
		std::signal(SIGINT, signal_handler);
		std::cout << "Publisher running, press CTRL-c to stop." << std::endl;

		while (running) {
			try {
				robot.control(control_callback);
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
					running = false;
				}
			}
		}

	}
	catch (const franka::Exception& ex) {
		running = false;
		std::cerr << "Robot has encountered an error and will stop." << std::endl
			<< ex.what() << std::endl;
	}

	if (publish_thread.joinable()) {
		publish_thread.join();
	}

	if (close(sock) == -1) {
		std::cerr << "There was an error closing the socket." << std::endl;
	}

	return EXIT_SUCCESS;
}
