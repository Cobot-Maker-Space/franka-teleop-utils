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
#include "teleop_utils.h"

void configure_robot(franka::Robot& robot, YAML::Node& config) {
  YAML::Node ltt = config["robot"]["collision_behaviour"]["lower_torque_thresholds"];
  YAML::Node utt = config["robot"]["collision_behaviour"]["upper_torque_thresholds"];
  YAML::Node lft = config["robot"]["collision_behaviour"]["lower_force_thresholds"];
  YAML::Node uft = config["robot"]["collision_behaviour"]["upper_force_thresholds"];
  robot.setCollisionBehavior(
    {{
      ltt["joint1"].as<double_t>(),
      ltt["joint2"].as<double_t>(),
      ltt["joint3"].as<double_t>(),
      ltt["joint4"].as<double_t>(),
      ltt["joint5"].as<double_t>(),
      ltt["joint6"].as<double_t>(),
      ltt["joint7"].as<double_t>(),
    }},
    {{
      utt["joint1"].as<double_t>(),
      utt["joint2"].as<double_t>(),
      utt["joint3"].as<double_t>(),
      utt["joint4"].as<double_t>(),
      utt["joint5"].as<double_t>(),
      utt["joint6"].as<double_t>(),
      utt["joint7"].as<double_t>(),
    }},
    {{
      lft["x"].as<double_t>(),
      lft["y"].as<double_t>(),
      lft["z"].as<double_t>(),
      lft["R"].as<double_t>(),
      lft["P"].as<double_t>(),
      lft["Y"].as<double_t>(),
    }},
    {{
      uft["x"].as<double_t>(),
      uft["y"].as<double_t>(),
      uft["z"].as<double_t>(),
      uft["R"].as<double_t>(),
      uft["P"].as<double_t>(),
      uft["Y"].as<double_t>(),
    }});
  YAML::Node ji = config["robot"]["joint_impedance"];
  robot.setJointImpedance({{
    ji["joint1"].as<double_t>(),
    ji["joint2"].as<double_t>(),
    ji["joint3"].as<double_t>(),
    ji["joint4"].as<double_t>(),
    ji["joint5"].as<double_t>(),
    ji["joint6"].as<double_t>(),
    ji["joint7"].as<double_t>(),
  }});
  YAML::Node ci = config["robot"]["cartesian_impedance"];
  robot.setCartesianImpedance({{
    ci["x"].as<double_t>(),
    ci["y"].as<double_t>(),
    ci["z"].as<double_t>(),
    ci["R"].as<double_t>(),
    ci["P"].as<double_t>(),
    ci["Y"].as<double_t>(),
  }});
}
