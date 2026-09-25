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

class ESKFNode : public rclcpp::Node
{
  public:
    ESKFNode()
    : Node("boaty_eskf_node"), gnssConverter {GNSS2NED(40.0, 3.0, 0.0)}, eskf(makeEskfParams())
    {
      publisher_eskf = this->create_publisher<nav_msgs::msg::Odometry>("/boaty/odom_filtered", 10);

      subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
      "/boaty/imu", 10, std::bind(&ESKFNode::imu_callback, this, _1));

      subscription_gnss = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/boaty/gnss", 10, std::bind(&ESKFNode::gnss_callback, this, _1));

    }

  private:
    
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

      NominalState xEst = eskf.getNomState();

      auto msgOdom = nav_msgs::msg::Odometry();
      
      msgOdom.header = msg.header;
      msgOdom.child_frame_id = "Boaty/base_link";
      
      msgOdom.pose.pose.position.x = xEst.pos(0);
      msgOdom.pose.pose.position.y = xEst.pos(1);
      msgOdom.pose.pose.position.z = xEst.pos(2);
      
      msgOdom.pose.pose.orientation.x = xEst.ori.vec_part(0);
      msgOdom.pose.pose.orientation.y = xEst.ori.vec_part(1);
      msgOdom.pose.pose.orientation.z = xEst.ori.vec_part(2);
      msgOdom.pose.pose.orientation.w = xEst.ori.real_part;

      msgOdom.twist.twist.linear.x = xEst.vel(0);
      msgOdom.twist.twist.linear.y = xEst.vel(1);
      msgOdom.twist.twist.linear.z = xEst.vel(2);

      // Use the IMU angular velocity measurement
      msgOdom.twist.twist.angular.x = avel(0);
      msgOdom.twist.twist.angular.y = avel(1);
      msgOdom.twist.twist.angular.z = avel(2);

      have_avel = 1;
      avelIMU = avel;
      
      publisher_eskf->publish(msgOdom);

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


      NominalState xEst = eskf.getNomState();

      auto msgOdom = nav_msgs::msg::Odometry();
      
      msgOdom.header = msg.header;
      msgOdom.child_frame_id = "Boaty/base_link";
      
      msgOdom.pose.pose.position.x = xEst.pos(0);
      msgOdom.pose.pose.position.y = xEst.pos(1);
      msgOdom.pose.pose.position.z = xEst.pos(2);
      
      msgOdom.pose.pose.orientation.x = xEst.ori.vec_part(0);
      msgOdom.pose.pose.orientation.y = xEst.ori.vec_part(1);
      msgOdom.pose.pose.orientation.z = xEst.ori.vec_part(2);
      msgOdom.pose.pose.orientation.w = xEst.ori.real_part;

      msgOdom.twist.twist.linear.x = xEst.vel(0);
      msgOdom.twist.twist.linear.y = xEst.vel(1);
      msgOdom.twist.twist.linear.z = xEst.vel(2);

      // Use the IMU angular velocity measurement
      if (have_avel) {
        msgOdom.twist.twist.angular.x = avelIMU(0);
        msgOdom.twist.twist.angular.y = avelIMU(1);
        msgOdom.twist.twist.angular.z = avelIMU(2);
      }
      
      publisher_eskf->publish(msgOdom);

    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_eskf;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gnss;

    rclcpp::Time last_filter_time_; // for calculating dt
    bool first_measurement{1};
    bool have_avel{0};
    Eigen::Vector3d avelIMU; // saved from the IMU measurements
    
    GNSS2NED gnssConverter;
    
    static ESKFParams makeEskfParams(){
      Eigen::Vector3d gnss_lever;
      gnss_lever << 0.5, 0.25, -0.4;

      NominalState x0;
      ErrorStateGauss xErr0;

      x0.pos << 0.0, 0.0, 0.0;
      x0.vel << 0.0, 0.0, 0.0;
      x0.ori = RotationQuaternion(); // initializes with zero rotation
      x0.accm_bias << 0.0, 0.0, 0.0;
      x0.gyro_bias << 0.0, 0.0, 0.0;

      xErr0.cov = 0.25 * Eigen::MatrixXd::Identity(15, 15);

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
      gnss_lever, // GNSS lever arm
      x0, // initial nominal state
      xErr0 // initial error state (for covariance)
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
