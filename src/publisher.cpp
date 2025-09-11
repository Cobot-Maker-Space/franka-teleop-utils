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
#include <chrono>

#include <franka/exception.h>

#include "memory_outputstream.h"
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
	stop = [&thread_data](int) -> void { thread_data.running = false; };
	std::signal(SIGINT, signal_handler);
	YAML::Node config = parse_options(argc, argv);
	capnp::MallocMessageBuilder message{};

	asio::io_context io_ctx;
	using asio::ip::udp;
	auto host = asio::ip::address::from_string(
		config["publish"]["host"].as<std::string>());
	auto port = config["publish"]["port"].as<asio::ip::port_type>();
	udp::endpoint endpoint(host, port);
	udp::socket socket(io_ctx);
	socket.open(udp::v4());

	std::thread publish_thread(PublishThread{
		socket,
		endpoint,
		config["publish"]["rate"].as<double_t>(),
		message,
		thread_data });

	franka::Robot robot(config["robot"]["host"].as<std::string>());
	configure_robot(config, robot);
	franka::Torques torques = franka::Torques(
		std::array<double, 7>{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
	RobotState::Builder state_builder = message.initRoot<RobotState>();

	auto control_callback = [&state_builder, &thread_data, &torques](
		const franka::RobotState& state, franka::Duration time_step) -> franka::Torques {
			// Get current Unix timestamp in milliseconds
			uint64_t robot_time = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::system_clock::now().time_since_epoch()
			).count();
			if (!thread_data.running) {
				return franka::MotionFinished(torques);
			}
			if (thread_data.lock.try_lock()) {
				thread_data.updated = true;
				state_builder.setTime(robot_time);
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
				state_builder.setJoint1Torque(state.tau_J[0]);
				state_builder.setJoint2Torque(state.tau_J[1]);
				state_builder.setJoint3Torque(state.tau_J[2]);
				state_builder.setJoint4Torque(state.tau_J[3]);
				state_builder.setJoint5Torque(state.tau_J[4]);
				state_builder.setJoint6Torque(state.tau_J[5]);
				state_builder.setJoint7Torque(state.tau_J[6]);
				state_builder.setJoint1ExtTorque(state.tau_ext_hat_filtered[0]);
				state_builder.setJoint2ExtTorque(state.tau_ext_hat_filtered[1]);
				state_builder.setJoint3ExtTorque(state.tau_ext_hat_filtered[2]);
				state_builder.setJoint4ExtTorque(state.tau_ext_hat_filtered[3]);
				state_builder.setJoint5ExtTorque(state.tau_ext_hat_filtered[4]);
				state_builder.setJoint6ExtTorque(state.tau_ext_hat_filtered[5]);
				state_builder.setJoint7ExtTorque(state.tau_ext_hat_filtered[6]);
				thread_data.lock.unlock();
#ifdef REPORT_RATE
				thread_data.counter++;
#endif
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
	std::thread report_thread(ReportThread{ "Publisher", thread_data });
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

	if (publish_thread.joinable()) {
		publish_thread.join();
	}
#ifdef REPORT_RATE
	if (report_thread.joinable()) {
		report_thread.join();
	}
#endif

	return EXIT_SUCCESS;
}
