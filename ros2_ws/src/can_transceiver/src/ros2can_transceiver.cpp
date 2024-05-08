#include "can_transceiver/ros2can_transceiver.h"

using namespace std::chrono_literals;
TestNode::TestNode() : rclcpp::Node("test_node_canlib"), canlib::Transceiver() {}

TestNode::TestNode(std::string device_name, std::vector<can_filter> filters)
    : rclcpp::Node("test_node_canlib"), canlib::Transceiver(device_name, filters) {
  m_brake_ratio = 0.0;
  m_lap_count = 0.0;
  // Use callbacks to customize the decoding of the incoming data
  canlib::callback::can1::jetson_commands = [&](can1_jetson_commands_t /*frame_encoded*/,
                                                canlib::frame::decoded::can1::jetson_commands_t frame_decoded) {
    RCLCPP_INFO(this->get_logger(), "received jetson_commands: brake ratio = %lf", frame_decoded.jetson_brake_ratio);
    m_brake_ratio = frame_decoded.jetson_brake_ratio;
  };
  canlib::callback::can1::jetson_tx = [&](can1_jetson_tx_t /*frame_encoded*/, canlib::frame::decoded::can1::jetson_tx_t frame_decoded) {
    RCLCPP_INFO(this->get_logger(), "received jetson_tx: lap count = %lf", frame_decoded.jetson_lap_count);
    m_lap_count = frame_decoded.jetson_lap_count;
  };

  // Use a thread to continuously update the can messages
  std::thread receiver([this]() {
    RCLCPP_INFO(this->get_logger(), "starting CAN receiver ...");
    this->is_receiver_running = true;
    while (rclcpp::ok()) {
      mtx.lock();
      if (receive()) {
        RCLCPP_INFO(this->get_logger(), "received data");
      }
      mtx.unlock();
    }
    RCLCPP_INFO(this->get_logger(), "shutdown CAN receiver ...");
    this->is_receiver_running = false;
  });

  // Just for an example send contnuosly data
  std::thread transmitter([this]() {
    RCLCPP_INFO(this->get_logger(), "starting CAN transmitter ...");
    this->is_transmitter_running = true;
    while (rclcpp::ok()) {
      transmit(canlib::frame::decoded::can1::jetson_commands_t(0.1, 0.2, 0.3, 0.4, 0.5));
      RCLCPP_INFO(this->get_logger(), "current lap count: %lf", m_lap_count);
      sleep(1);
    }
    RCLCPP_INFO(this->get_logger(), "shutdown CAN transmitter ...");
    this->is_transmitter_running = false;
  });

  receiver.detach();
  transmitter.detach();
  RCLCPP_INFO(this->get_logger(), "init success");
}

TestNode::~TestNode() {
  RCLCPP_INFO(this->get_logger(), "destructor called");
  while (is_receiver_running || is_transmitter_running) {
    if (is_receiver_running) RCLCPP_INFO(this->get_logger(), "CAN receiver is still running");
    if (is_transmitter_running) RCLCPP_INFO(this->get_logger(), "CAN transmitter is still running");
    sleep(1);
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  // Setup filters
  std::vector<can_filter> filters;
  can_filter filter;
  filter.can_id = 0x0E2;
  filter.can_mask = CAN_SFF_MASK;
  filters.push_back(filter);
  filter.can_id = 0x0E1;
  filter.can_mask = CAN_SFF_MASK;
  filters.push_back(filter);
  // Set device name
  std::string device_name = "vcan0";
  // Start ROS Node
  rclcpp::spin(std::make_shared<TestNode>(device_name, filters));
  rclcpp::shutdown();
  return 0;
}