#include "quaternions/quaternions.h"
#include "eskf/eskf.h"
#include "gnss2ned/gnss2ned.h"

#include <Eigen/Dense>
#include <iostream> 

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ESKFNode : public rclcpp::Node
{
  public:
    ESKFNode()
    : Node("boaty_eskf_node"), gnssConverter {GNSS2NED(40.0, 3.0, 0.0)}, eskf(makeEskfParams())
    {
      publisher_eskf = this->create_publisher<nav_msgs::msg::Odometry>("/boaty/odom_filtered", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&ESKFNode::timer_callback, this));

      subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
      "/boaty/imu", 10, std::bind(&ESKFNode::imu_callback, this, _1));

      subscription_gnss = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/boaty/gnss", 10, std::bind(&ESKFNode::gnss_callback, this, _1));

    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      //message.data = "Hello, world! " + std::to_string(count_++);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      //publisher_->publish(message);
    }
    
    void imu_callback(const sensor_msgs::msg::Imu & msg)
    {
      //RCLCPP_INFO(this->get_logger(), "Received IMU!");
      //std::cout << msg.linear_acceleration.z << std::endl;
      
      const rclcpp::Time measurement_time(msg.header.stamp);

      if (first_measurement) {
        // initialize from the IMU measurement
        last_filter_time_ = measurement_time;
        first_measurement = 0;
        return;
      }

      float ts = (measurement_time - last_filter_time_).seconds();

      if (ts <= 0.0) {
        RCLCPP_INFO(get_logger(), "Ignoring duplicate or out of sequence measurement");
        return;
      }

      Eigen::Vector3d acc;
      Eigen::Vector3d avel;

      acc << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
      avel << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
      
      IMUMeasurement zIMU{acc, avel, ts};

      eskf.predictFromIMU(zIMU);

    }

    void gnss_callback(const sensor_msgs::msg::NavSatFix & msg)
    {
      //RCLCPP_INFO(this->get_logger(), "Received gnss!");
      //std::cout << msg.latitude << std::endl;

      Eigen::Vector3d gnssNED = gnssConverter.toNED(msg.latitude, msg.longitude, msg.altitude); 
      //RCLCPP_INFO(this->get_logger(), "Converted to NED: !");
      //std::cout << gnssNED << std::endl;

      const rclcpp::Time measurement_time(msg.header.stamp);

      if (first_measurement) {
        // initialize from the IMU measurement
        last_filter_time_ = measurement_time;
        first_measurement = 0;
        return;
      }

      float ts = (measurement_time - last_filter_time_).seconds();

      if (ts <= 0.0) {
        RCLCPP_INFO(get_logger(), "Ignoring duplicate or out of sequence measurement");
        return;
      }

      GNSSMeasurement zGNSS{gnssNED, ts};

      eskf.updateFromGNSS(zGNSS);

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_eskf;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gnss;

    rclcpp::Time last_filter_time_; // for calculating dt
    bool first_measurement{1};
    
    GNSS2NED gnssConverter;
    
    static ESKFParams makeEskfParams(){
      Eigen::Vector3d gnss_lever;
      gnss_lever << 0.5, 0.25, -0.4;
      
      return ESKFParams{
      0.10, // accm_std
      0.25, // accm_bias_std
      0.0005, // accm_bias_p
      0.10, // gyro_std
      0.25, // gyro_bias_std
      0.0005, // gyro_bias_p
      0.5, // gnss_std_n
      0.5, // gnss_std_e
      0.5, // gnss_std_d
      Eigen::MatrixXd::Identity(3, 3), // accm_correction
      Eigen::MatrixXd::Identity(3, 3), // gyro_correction
      gnss_lever // GNSS lever arm
      };
    }
    
    ESKF eskf;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ESKFNode>());
  rclcpp::shutdown();
  return 0;
}
