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

#include <cxxopts.hpp>

#include "memory_outputstream.h"
#include "messages/robot-state.capnp.h"
#include "teleop_utils.h"

void configure_robot(YAML::Node& config, franka::Robot& robot) {
  YAML::Node ltt = config["robot"]["collision_behaviour"]["lower_torque_thresholds"];
  YAML::Node utt = config["robot"]["collision_behaviour"]["upper_torque_thresholds"];
  YAML::Node lft = config["robot"]["collision_behaviour"]["lower_force_thresholds"];
  YAML::Node uft = config["robot"]["collision_behaviour"]["upper_force_thresholds"];
  robot.setCollisionBehavior(
    { {
      ltt["joint1"].as<double>(),
      ltt["joint2"].as<double>(),
      ltt["joint3"].as<double>(),
      ltt["joint4"].as<double>(),
      ltt["joint5"].as<double>(),
      ltt["joint6"].as<double>(),
      ltt["joint7"].as<double>(),
    } },
    { {
      utt["joint1"].as<double>(),
      utt["joint2"].as<double>(),
      utt["joint3"].as<double>(),
      utt["joint4"].as<double>(),
      utt["joint5"].as<double>(),
      utt["joint6"].as<double>(),
      utt["joint7"].as<double>(),
    } },
    { {
      lft["x"].as<double>(),
      lft["y"].as<double>(),
      lft["z"].as<double>(),
      lft["R"].as<double>(),
      lft["P"].as<double>(),
      lft["Y"].as<double>(),
    } },
    { {
      uft["x"].as<double>(),
      uft["y"].as<double>(),
      uft["z"].as<double>(),
      uft["R"].as<double>(),
      uft["P"].as<double>(),
      uft["Y"].as<double>(),
    } });
  YAML::Node ji = config["robot"]["joint_impedance"];
  robot.setJointImpedance({ {
    ji["joint1"].as<double>(),
    ji["joint2"].as<double>(),
    ji["joint3"].as<double>(),
    ji["joint4"].as<double>(),
    ji["joint5"].as<double>(),
    ji["joint6"].as<double>(),
    ji["joint7"].as<double>(),
  } });
  YAML::Node ci = config["robot"]["cartesian_impedance"];
  robot.setCartesianImpedance({ {
    ci["x"].as<double>(),
    ci["y"].as<double>(),
    ci["z"].as<double>(),
    ci["R"].as<double>(),
    ci["P"].as<double>(),
    ci["Y"].as<double>(),
  } });
}

YAML::Node parse_options(int argc, const char** argv) {
  cxxopts::Options options("sender", "Send robot state to receiver(s)");
  options.add_options()
    ("c,config-file", "Path to configuration file", cxxopts::value<std::string>());
  auto poptions = options.parse(argc, argv);

  if (poptions.count("config-file")) {
    return YAML::LoadFile(poptions["config-file"].as<std::string>());
  }
  else {
    return YAML::Load(std::cin);
  }
}

void PublishThread::operator()() const {
  kj::byte buffer[MESSAGE_SIZE];
  memset(buffer, 0, MESSAGE_SIZE);
  MemoryOutputStream os(buffer);
  std::chrono::milliseconds sleep_time =
    std::chrono::milliseconds(static_cast<int>((1.0 / publish_rate * 1000.0)));
  while (thread_data.running) {
    std::this_thread::sleep_for(sleep_time);
    if (thread_data.lock.try_lock()) {
      if (thread_data.updated) {
        os.reset();
        capnp::writeMessage(os, message);
        socket.send_to(asio::buffer(buffer, MESSAGE_SIZE), endpoint);
        thread_data.updated = false;
      }
      thread_data.lock.unlock();
    }
  }
}

void SubscribeThread::operator()() const {
  double last_received_time = 0.0;
  std::array<kj::byte, MESSAGE_SIZE> buffer;
  auto mbuffer = asio::buffer(buffer);
  kj::ArrayPtr<kj::byte> buffer_array(buffer.data(), MESSAGE_SIZE);

  while (thread_data.running) {
    if (socket.receive(mbuffer) != MESSAGE_SIZE) {
      std::cerr << "Error during message reception." << std::endl;
      continue;
    }

    kj::ArrayInputStream in(buffer_array);
    capnp::InputStreamMessageReader reader(in);
    RobotState::Reader state = reader.getRoot<RobotState>();

    if (state.getTime() > last_received_time && thread_data.lock.try_lock()) {
      thread_data.updated = true;
      last_received_time = state.getTime();
      leader_pos[0] = state.getJoint1Pos();
      leader_pos[1] = state.getJoint2Pos();
      leader_pos[2] = state.getJoint3Pos();
      leader_pos[3] = state.getJoint4Pos();
      leader_pos[4] = state.getJoint5Pos();
      leader_pos[5] = state.getJoint6Pos();
      leader_pos[6] = state.getJoint7Pos();
      leader_vel[0] = state.getJoint1Vel();
      leader_vel[1] = state.getJoint2Vel();
      leader_vel[2] = state.getJoint3Vel();
      leader_vel[3] = state.getJoint4Vel();
      leader_vel[4] = state.getJoint5Vel();
      leader_vel[5] = state.getJoint6Vel();
      leader_vel[6] = state.getJoint7Vel();
      
      // Read gripper state
      leader_gripper_width = state.getGripperWidth();
      leader_gripper_grasped = state.getGripperIsGrasped();
      
      if (!thread_data.first_packet_received) {
        thread_data.first_packet_received = true;
      }
      
      thread_data.lock.unlock();
    }
  }
}

#ifdef REPORT_RATE
void ReportThread::operator()() const {
  std::cout << std::endl;
  while (thread_data.running) {
    if (thread_data.lock.try_lock()) {
      std::cout << name << "update rate: "
        << thread_data.counter << "hz" << std::endl;
      thread_data.counter = 0;
    }
    thread_data.lock.unlock();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}
#endif
