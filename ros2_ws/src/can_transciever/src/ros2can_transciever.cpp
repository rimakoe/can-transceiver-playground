#include "can_transciever/ros2can_transciever.h"

using namespace std::chrono_literals;

TestNode::TestNode(): rclcpp::Node("test_node_canlib"), canlib::Transciever() {
    // Use callbacks to customize the decoding of the incoming data
    canlib::callback::rcv::can1::jetson_commands = [&](can1_jetson_commands_t /*frame_encoded*/){
      RCLCPP_INFO(this->get_logger(), "Hello from callback %lf\n", canlib::data.can1.jetson_commands.jetson_speed_target_left);
    };

    // Use a thread to continuously update the can messages
    std::thread receiver([this](){
      RCLCPP_INFO(this->get_logger(), "starting CAN receiver ...");
      this->is_receiver_running = true;
      while(rclcpp::ok()){
        if(receive()){
          RCLCPP_INFO(this->get_logger(), "received data");
        }
      }
      RCLCPP_INFO(this->get_logger(), "shutdown CAN receiver ...");
      this->is_receiver_running = false;
    });

    // Just for an example send contnuosly data
    std::thread transmitter([this](){
      RCLCPP_INFO(this->get_logger(), "starting CAN transmitter ...");
      this->is_transmitter_running = true;
      while(rclcpp::ok()){
        transmit(canlib::frame::decoded::can1::jetson_commands_t(
          0.1, 
          0.2, 
          0.3, 
          0.4, 
          0.5
        ));
        sleep(1);   
      }
      RCLCPP_INFO(this->get_logger(), "shutdown CAN transmitter ...");
      this->is_transmitter_running = false;
    });

    receiver.detach();
    transmitter.detach();
    RCLCPP_INFO(this->get_logger(), "init success");
}

TestNode::~TestNode(){
  RCLCPP_INFO(this->get_logger(), "destructor called");
  while(is_receiver_running || is_transmitter_running){
    if(is_receiver_running) RCLCPP_INFO(this->get_logger(), "CAN receiver is still running");
    if(is_transmitter_running) RCLCPP_INFO(this->get_logger(), "CAN transmitter is still running");
    sleep(1);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}