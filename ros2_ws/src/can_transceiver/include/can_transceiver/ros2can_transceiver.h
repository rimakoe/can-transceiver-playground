#ifndef _TEST_NODE_H_
#define _TEST_NODE_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <can-transceiver-lib/transceiver.h>

class TestNode : public rclcpp::Node, canlib::Transceiver
{
  public:
    TestNode();
    ~TestNode();
  
  private:
    bool is_receiver_running = false;
    bool is_transmitter_running = false;
};

#endif // _TEST_NODE_H_