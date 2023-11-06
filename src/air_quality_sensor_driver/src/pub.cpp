// Publisher definition and implementation

#include <iostream>
#include <string>

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

  try {

    int device_id;
    std::string acquisition_date;
    float latitude;
    float longitude;
    float altitude
    int gps_precision;
    float pm25;
    float pm10;
    float humidity;
    float temperature;
    float no2;
    float co2;
    float nh3;
    float co;

    // Read from serial port  TODO AGGIUNGERE ALTITUDE
    
    RCLCPP_INFO(get_logger(), "New data acquisition");

    asio::io_context io;
    asio::serial_port port(io, "/dev/ttyACM0"); // TODO: add udev rule

    asio::streambuf buf;
    asio::read_until(port, buf, '\n');

    std::istream stream(&buf);

    std::string received_data;
    std::getline(stream, received_data);

    // Save all the acquired tokens in a vector
    std::stringstream str_stream(received_data);
    std::string token;
    std::vector<std::string> str_vector;
    while (std::getline(str_stream, token, ',')) {
      str_vector.push_back(token);
    }
    
    if(str_vector.size() == 14) {

      // Save the acquired values into the appropriate variables
      device_id = std::stoi(str_vector[0]);
      acquisition_date = str_vector[1];
      latitude = std::stof(str_vector[2]);
      longitude = std::stof(str_vector[3]);
      altitude = std::stof(str_vector[4]);
      gps_precision= std::stoi(str_vector[5]);
      pm25 = std::stof(str_vector[6]);
      pm10 = std::stof(str_vector[7]);
      humidity = std::stof(str_vector[8]);
      temperature = std::stof(str_vector[9]);
      no2 = std::stof(str_vector[10]);
      co2 = std::stof(str_vector[11]);
      nh3 = std::stof(str_vector[12]);
      co = std::stof(str_vector[13]);

      std::cout << "device_id: " << device_id << std::endl;
      std::cout << "acquisition_date: " << acquisition_date << std::endl;
      std::cout << "latitude: " << latitude << std::endl;
      std::cout << "longitude: " << longitude << std::endl;
      std::cout << "altitude: " << altitude << std::endl;
      std::cout << "gps_precision: " << gps_precision << std::endl;
      std::cout << "pm25: " << pm25 << std::endl;
      std::cout << "pm10: " << pm10 << std::endl;
      std::cout << "humidity: " << humidity << std::endl;
      std::cout << "temperature: " << temperature << std::endl;
      std::cout << "no2: " << no2 << std::endl;
      std::cout << "co2: " << co2 << std::endl;
      std::cout << "nh3: " << nh3 << std::endl;
      std::cout << "co: " << co << std::endl;

      // Messages

      sensor_msgs::msg::NavSatFix new_msg_gps{};
      new_msg_gps.set__longitude(longitude);
      new_msg_gps.set__latitude(latitude);
      new_msg_gps.set__altitude(altitude);
      gps_pub_->publish(new_msg_gps);

      traxxas1_interfaces::msg::AirQuality new_msg_air{};
      new_msg_air.set__gps_data(new_msg_gps);
      new_msg_air.set__device_id(device_id);
      new_msg_air.set__pm25(pm25);
      new_msg_air.set__pm10(pm10);
      new_msg_air.set__humidity(humidity);
      new_msg_air.set__temperature(temperature);
      new_msg_air.set__no2(no2);
      new_msg_air.set__co2(co2);
      new_msg_air.set__nh3(nh3);
      new_msg_air.set__co(co);
      air_quality_pub_->publish(new_msg_air);

      pub_cnt_++;
      RCLCPP_INFO(this->get_logger(), "Published message %lu", pub_cnt_);

    }
  } catch (std::exception &exception) {
    RCLCPP_ERROR(get_logger(), "Error in reading from serial port: %s", exception.what());
  }
  
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
