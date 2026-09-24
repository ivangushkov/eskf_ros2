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

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ESKFNode : public rclcpp::Node
{
  public:
    ESKFNode()
    : Node("boaty_eskf_node"), count_(0), gnssConverter {GNSS2NED(40.0, 3.0, 0.0)}
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&ESKFNode::timer_callback, this));

      subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
      "/boaty/imu", 10, std::bind(&ESKFNode::imu_callback, this, _1));

      subscription_gnss = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/boaty/gnss", 10, std::bind(&ESKFNode::gnss_callback, this, _1));

      Eigen::Vector3d gnss_lever;
      gnss_lever << 1.0, 0.0, 2.0;

      ESKFParams p{
        2.0,                              // accm_std
        4.0,                              // accm_bias_std
        6.0,                              // accm_bias_p
        8.0,                              // gyro_std
        10.0,                             // gyro_bias_std
        12.0,                             // gyro_bias_p
        20.0,                             // gnss_std_n;
        20.0,                             // gnss_std_e;
        20.0,                             // gnss_std_d;
        Eigen::MatrixXd::Identity(3, 3),  // accm_correction
        Eigen::MatrixXd::Identity(3, 3),  // gyro_correction 
        gnss_lever                        // gnss lever arm
      };

      ESKF eskf(p);

      //gnssConverter = GNSS2NED(40.0, 3.0, 0.0);

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

    }

    void gnss_callback(const sensor_msgs::msg::NavSatFix & msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received gnss!");
      std::cout << msg.latitude << std::endl;

      Eigen::Vector3d gnssNED = gnssConverter.toNED(msg.latitude, msg.longitude, msg.altitude); 
      RCLCPP_INFO(this->get_logger(), "Converted to NED: !");
      std::cout << gnssNED << std::endl;

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gnss;

    GNSS2NED gnssConverter;

    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ESKFNode>());
  rclcpp::shutdown();
  return 0;
}
