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

#include <cxxopts.hpp>

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
#ifdef REPORT_RATE
		int counter = 0;
#endif
	} thread_data{};

	int sock;
	if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		std::cerr << "Could not create socket." << std::endl;
		exit(EXIT_FAILURE);
	}

	stop = [&thread_data](int) -> void { thread_data.running = false; };

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

	capnp::MallocMessageBuilder message;
	std::thread publish_thread([&config, &sock, &message, &thread_data]() {
		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_port = htons(config["publish"]["port"].as<u_short>());
		addr.sin_addr.s_addr = inet_addr(config["publish"]["host"].as<std::string>().c_str());
		socklen_t addr_len = sizeof(addr);
		kj::byte buffer[MESSAGE_SIZE];
		memset(buffer, 0, MESSAGE_SIZE);
		const double rate = config["publish"]["rate"].as<double>();
		std::chrono::milliseconds sleep_time =
			std::chrono::milliseconds(static_cast<int>((1.0 / rate * 1000.0)));
		while (thread_data.running) {
			std::this_thread::sleep_for(sleep_time);
			if (thread_data.lock.try_lock()) {
				if (thread_data.updated) {
					MemoryOutputStream os(buffer);
					capnp::writeMessage(os, message);
					if (sendto(sock, buffer, os.getSize(), MSG_CONFIRM,
						(const struct sockaddr*)&addr, addr_len) < 0) {
						std::cerr << "An error occurred during message transmission" << std::endl;
					}
					thread_data.updated = false;
				}
				thread_data.lock.unlock();
			}
		}
		return EXIT_SUCCESS;
		});

	try {
		franka::Robot robot(config["robot"]["host"].as<std::string>());
		configure_robot(robot, config);
		const bool rate_limit = config["robot"]["rate_limit"].as<bool>();
		const double cutoff_freq = config["robot"]["cutoff_frequency"].as<double>();
		const bool autorecover = config["robot"]["autorecovery"]["enabled"].as<bool>();
		const long long autorecover_wait_time =
			config["robot"]["autorecovery"]["wait_time_ms"].as<long long>();
		const franka::Torques torques = franka::Torques(
			std::array<double, 7>{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
		RobotState::Builder state_builder = message.initRoot<RobotState>();

		std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
			control_callback = [&state_builder, &thread_data, &torques](
				const franka::RobotState& state, franka::Duration) -> franka::Torques {
					if (!thread_data.running) {
						return franka::MotionFinished(torques);
					}

					if (thread_data.lock.try_lock()) {
						thread_data.updated = true;
						state_builder.setTime(state.time.toMSec());
						state_builder.setJoint1Pos(state.q[0]);
						state_builder.setJoint2Pos(state.q[1]);
						state_builder.setJoint3Pos(state.q[2]);
						state_builder.setJoint4Pos(state.q[3]);
						state_builder.setJoint5Pos(state.q[4]);
						state_builder.setJoint6Pos(state.q[5]);
						state_builder.setJoint7Pos(state.q[6]);
						state_builder.setJoint1Vel(state.dq[0]);
						state_builder.setJoint2Vel(state.dq[1]);
						state_builder.setJoint3Vel(state.dq[2]);
						state_builder.setJoint4Vel(state.dq[3]);
						state_builder.setJoint5Vel(state.dq[4]);
						state_builder.setJoint6Vel(state.dq[5]);
						state_builder.setJoint7Vel(state.dq[6]);
						thread_data.lock.unlock();
					}
#ifdef REPORT_RATE
					thread_data.counter++;
#endif
					return torques;
			};

		std::signal(SIGINT, signal_handler);
		std::cout << "Publisher running, press CTRL-c to stop." << std::endl;

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

	if (publish_thread.joinable()) {
		publish_thread.join();
	}

	if (close(sock) == -1) {
		std::cerr << "There was an error closing the socket." << std::endl;
	}

	return EXIT_SUCCESS;
}
