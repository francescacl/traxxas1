# Air quality sensor message

# ROS 2 message header (with timestamp from ROS 2 real-time clock)
std_msgs/Header header

int8 device_id
string acquisition_date

# GPS data: latitude, longitude
sensor_msgs/NavSatFix gps_data
int8 gps_precision

# Air quality data
float32 pm25
float32 pm10
float32 humidity
float32 temperature
float32 no2
float32 co2
float32 nh3
float32 co