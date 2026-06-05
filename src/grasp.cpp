#include <iostream>

#include <cxxopts.hpp>

#include <franka/exception.h>
#include <franka/gripper.h>

#include "teleop_utils.h"

int main(int argc, const char** argv) {

  YAML::Node config = parse_options(argc, argv);

  cxxopts::Options options(argv[0]);
  options.add_options()
    ("w,width", "Grasp width", cxxopts::value<double>());
  options.add_options()
    ("s,speed", "Grasp speed", cxxopts::value<double>());
  options.add_options()
    ("f,force", "Grasp force", cxxopts::value<double>());
  auto poptions = options.parse(argc, argv);

  try {
    franka::Gripper gripper(config["robot"]["host"].as<std::string>());
    double width = poptions["width"].as<double>();
    double speed = poptions["speed"].as<double>();
    double force = poptions["force"].as<double>();
    gripper.homing();
    gripper.grasp(width, speed, force);
  }
  catch (franka::Exception const& ex) {
    std::cerr << "Error: " << std::endl << ex.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
