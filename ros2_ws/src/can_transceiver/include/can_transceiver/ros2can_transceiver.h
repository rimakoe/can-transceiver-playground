#ifndef _TEST_NODE_H_
#define _TEST_NODE_H_

#include <can-transceiver-lib/can1.h>
#include <can-transceiver-lib/transceiver.h>

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

class TestNode : public rclcpp::Node, canlib::Transceiver {
 public:
  TestNode();
  TestNode(std::string device_name, std::vector<can_filter> filters);
  ~TestNode();

 private:
  bool is_receiver_running = false;
  bool is_transmitter_running = false;
};

#endif  // _TEST_NODE_H_