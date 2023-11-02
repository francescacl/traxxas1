// Publisher definition and implementation

#include <iostream>

#include "../include/topic_air_quality_sensor/pub.hpp"


// Creates a node
Pub::Pub()
: Node("publisher_node"),
  pub_cnt_(0) // counter
{

  air_quality_pub_ = this->create_publisher<traxxas1_interfaces::msg::AirQuality>(
    "/car1/air_quality_sensor", // TODO: update topic name
    rclcpp::QoS(10));

  gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/car1/gps/position",
    rclcpp::QoS(10));

  // std::chrono::duration (period)
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(PUB_PERIOD), // 300 milliseconds, period
    std::bind(
      &Pub::pub_timer_callback, // pointer to pub_timer_callback, Pub class method
      this)); // 'this' because it has no arguments, but it must have access to this node

  RCLCPP_INFO(this->get_logger(), "Publisher initialized");
}


// Publishes a message on timer occurrence
void Pub::pub_timer_callback(void)
{
  
  sensor_msgs::msg::NavSatFix new_msg_gps{};
  // TODO: set new_msg_gps's information using the following syntax
  // new_msg_gps.set__data(new_data_gps);
  // gps_pub_->publish(new_msg_gps); // TODO: uncomment this line when new_msg_gps is not empty

  traxxas1_interfaces::msg::AirQuality new_msg_air{};
  // new_msg_air.set__gps_data(new_msg_gps); // TODO: uncomment this line when new_msg_gps is not empty
  // TODO: set other new_msg_air's information
  // air_quality_pub_->publish(new_msg_air); // TODO: uncomment this line when new_msg_air is not empty

  pub_cnt_++;
  RCLCPP_INFO(this->get_logger(), "Published message %lu", pub_cnt_);
}


int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  auto pub_node = std::make_shared<Pub>();

  rclcpp::spin(pub_node);

  rclcpp::shutdown();
  std::cout << "Publisher terminated" << std::endl;
  exit(EXIT_SUCCESS);

}