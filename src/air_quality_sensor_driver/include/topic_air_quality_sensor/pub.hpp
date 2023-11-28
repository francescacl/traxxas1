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


class SerialComm
{
public:
  SerialComm(std::string port, unsigned int baud_rate)
  : io(), serial(io, port)
  {
    serial.set_option(asio::serial_port_base::baud_rate(baud_rate));
  }

  bool get_data(std::vector<std::string>& str_vector)
  {
    asio::streambuf buf;
    asio::read_until(serial, buf, '\n');
    std::istream stream(&buf);

    std::string received_data;
    std::getline(stream, received_data);

    // Save all the acquired tokens in a vector
    std::stringstream str_stream(received_data);
    std::string token;
    bool none = false;
    while (std::getline(str_stream, token, ',')) {
      str_vector.push_back(token);
      if (token == std::string(" None")){
        none = true;
      }
    }

    return none;
  }

private:
  asio::io_service io;
  asio::serial_port serial;
};


class Pub : public rclcpp::Node
{
public:
  Pub();

private:
  // rclcpp::Publisher<INTERFACE_TYPE>::SharedPtr OBJ;
  rclcpp::Publisher<traxxas1_interfaces::msg::AirQuality>::SharedPtr air_quality_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;

  std::unique_ptr<SerialComm> serial;

  // rclcpp::TimerBase::SharedPtr OBJ;
  rclcpp::TimerBase::SharedPtr pub_timer_; // timer to publish messages
  void pub_timer_callback(void);
  unsigned long pub_cnt_;

  bool connected = false;

  int device_id;
  std::string acquisition_date;
  float latitude;
  float longitude;
  float altitude;
  int gps_precision;
  float pm25;
  float pm10;
  float humidity;
  float temperature;
  float no2;
  float co2;
  float nh3;
  float co;
};


#endif