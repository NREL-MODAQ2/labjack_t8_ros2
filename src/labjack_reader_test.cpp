#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "modaq_messages/msg/t8ainstream.hpp"
#include <iostream>
#include <vector>

using std::placeholders::_1;

class LabJackReaderTest : public rclcpp::Node
{

public:
  LabJackReaderTest()
      : Node("labjack_reader_test")
  {
    subscription_ = this->create_subscription<modaq_messages::msg::T8ainstream>(
        "labjack_ain", 10, std::bind(&LabJackReaderTest::topic_callback, this, _1));
  }

private:
  void topic_callback(const modaq_messages::msg::T8ainstream &msg)
  {

    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.ain0[0]);
    RCLCPP_INFO(this->get_logger(), "I heard timestamp: '%lu'", (double) msg.header.stamp.sec);
    t = (double) msg.header.stamp.sec + (double) msg.header.stamp.nanosec / 1e9;
    delt = (double) (t - last_t);
    
    dt = delt /multiplier;
    RCLCPP_INFO(this->get_logger(), "dt: '%f'", dt);
    last_t = t;


  }

  rclcpp::Subscription<modaq_messages::msg::T8ainstream>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t last_t, t;
  double dt, delt;
  double multiplier = 10^9;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LabJackReaderTest>());
  rclcpp::shutdown();
  return 0;
}