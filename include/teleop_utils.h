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
#include <asio.hpp>

#include <capnp/serialize.h>

#include <franka/robot.h>

#include <yaml-cpp/yaml.h>

/*
 * Message size in bytes. Calculated by producing a message and checking size
 * in sample code.
 */
const size_t MESSAGE_SIZE = 136;

struct thread_data {
  std::mutex lock;
  std::atomic_bool running{ true };
  bool updated = false;
#ifdef REPORT_RATE
  int counter = 0;
#endif
};

void configure_robot(YAML::Node& config, franka::Robot& robot);

YAML::Node parse_options(int, const char**);

class PublishThread {
public:
  PublishThread(
    asio::ip::udp::socket& socket,
    asio::ip::udp::endpoint& endpoint,
    double_t publish_rate,
    capnp::MallocMessageBuilder& message,
    struct thread_data& thread_data) :
    socket(socket),
    endpoint(endpoint),
    publish_rate(publish_rate),
    message(message),
    thread_data(thread_data) {
  };
  void operator()() const;
private:
  asio::ip::udp::socket& socket;
  asio::ip::udp::endpoint& endpoint;
  double_t publish_rate;
  capnp::MallocMessageBuilder& message;
  struct thread_data& thread_data;
};

class SubscribeThread {
public:
  SubscribeThread(
    std::array<double, 7>& leader_pos,
    std::array<double, 7>& leader_vel,
    asio::ip::udp::socket& socket,
    struct thread_data& thread_data) :
    leader_pos(leader_pos),
    leader_vel(leader_vel),
    socket(socket),
    thread_data(thread_data) {
  };
  void operator()() const;
private:
  std::array<double, 7>& leader_pos;
  std::array<double, 7>& leader_vel;
  asio::ip::udp::socket& socket;
  struct thread_data& thread_data;
};

#ifdef REPORT_RATE
class ReportThread {
public:
  ReportThread(std::string name, struct thread_data& thread_data)
    : name(name), thread_data(thread_data) {
  };
  void operator()() const;
private:
  std::string name;
  struct thread_data& thread_data;
};
#endif
