// Publisher definition

#ifndef PUB_HPP
#define PUB_HPP

#include <rclcpp/rclcpp.hpp>
#include <traxxas1_interfaces/msg/air_quality.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <asio.hpp>

#define PUB_PERIOD 300

class Pub : public rclcpp::Node
{
public:
  Pub();

private:
  // rclcpp::Publisher<INTERFACE_TYPE>::SharedPtr OBJ;
  rclcpp::Publisher<traxxas1_interfaces::msg::AirQuality>::SharedPtr air_quality_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;

  std::shared_ptr<asio::serial_port> port_;

  // rclcpp::TimerBase::SharedPtr OBJ;
  rclcpp::TimerBase::SharedPtr pub_timer_; // timer to publish messages
  void pub_timer_callback(void);
  unsigned long pub_cnt_;
};

#endif