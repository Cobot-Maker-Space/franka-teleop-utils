#include <iostream>

#include <franka/exception.h>
#include <franka/gripper.h>

#include "teleop_utils.h"

int main(int argc, const char** argv) {

  YAML::Node config = parse_options(argc, argv);

  try {
    franka::Gripper gripper(config["robot"]["host"].as<std::string>());
    double width = 0.024;
    gripper.homing();
    gripper.grasp(width, 0.05, 70);
  }
  catch (franka::Exception const& ex) {
    std::cerr << "Error: " << std::endl << ex.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
