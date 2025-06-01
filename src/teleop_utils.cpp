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
      ltt["joint1"].as<double>(),
      ltt["joint2"].as<double>(),
      ltt["joint3"].as<double>(),
      ltt["joint4"].as<double>(),
      ltt["joint5"].as<double>(),
      ltt["joint6"].as<double>(),
      ltt["joint7"].as<double>(),
    }},
    {{
      utt["joint1"].as<double>(),
      utt["joint2"].as<double>(),
      utt["joint3"].as<double>(),
      utt["joint4"].as<double>(),
      utt["joint5"].as<double>(),
      utt["joint6"].as<double>(),
      utt["joint7"].as<double>(),
    }},
    {{
      lft["x"].as<double>(),
      lft["y"].as<double>(),
      lft["z"].as<double>(),
      lft["R"].as<double>(),
      lft["P"].as<double>(),
      lft["Y"].as<double>(),
    }},
    {{
      uft["x"].as<double>(),
      uft["y"].as<double>(),
      uft["z"].as<double>(),
      uft["R"].as<double>(),
      uft["P"].as<double>(),
      uft["Y"].as<double>(),
    }});
  YAML::Node ji = config["robot"]["joint_impedance"];
  robot.setJointImpedance({{
    ji["joint1"].as<double>(),
    ji["joint2"].as<double>(),
    ji["joint3"].as<double>(),
    ji["joint4"].as<double>(),
    ji["joint5"].as<double>(),
    ji["joint6"].as<double>(),
    ji["joint7"].as<double>(),
  }});
  YAML::Node ci = config["robot"]["cartesian_impedance"];
  robot.setCartesianImpedance({{
    ci["x"].as<double>(),
    ci["y"].as<double>(),
    ci["z"].as<double>(),
    ci["R"].as<double>(),
    ci["P"].as<double>(),
    ci["Y"].as<double>(),
  }});
}
