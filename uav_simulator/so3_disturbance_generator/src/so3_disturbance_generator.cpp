#include <iostream>
#include <string.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pose_utils.h>

using namespace arma;
using namespace std;

#define CORRECTION_RATE 1

// Replaces the ROS1 dynamic_reconfigure DisturbanceUIConfig; values are
// declared as ROS2 parameters below and kept live via a parameter callback.
struct DisturbanceConfig
{
  double fxy = 0.0, stdfxy = 0.0;
  double fz = 0.0, stdfz = 0.0;
  double mrp = 0.0, stdmrp = 0.0;
  double myaw = 0.0, stdmyaw = 0.0;

  bool enable_noisy_odom = false;
  double stdxyz = 0.0, stdvxyz = 0.0, stdrp = 0.0, stdyaw = 0.0;

  bool enable_drift_odom = true;
  double stdvdriftxyz = 0.0, stdvdriftyaw = 0.0;
  double vdriftx = 0.0, vdrifty = 0.0, vdriftz = 0.0, vdriftyaw = 0.0;
};

class So3DisturbanceGenerator : public rclcpp::Node
{
public:
  So3DisturbanceGenerator() : Node("so3_disturbance_generator")
  {
    declare_parameter("fxy", config_.fxy);
    declare_parameter("stdfxy", config_.stdfxy);
    declare_parameter("fz", config_.fz);
    declare_parameter("stdfz", config_.stdfz);
    declare_parameter("mrp", config_.mrp);
    declare_parameter("stdmrp", config_.stdmrp);
    declare_parameter("myaw", config_.myaw);
    declare_parameter("stdmyaw", config_.stdmyaw);

    declare_parameter("enable_noisy_odom", config_.enable_noisy_odom);
    declare_parameter("stdxyz", config_.stdxyz);
    declare_parameter("stdvxyz", config_.stdvxyz);
    declare_parameter("stdrp", config_.stdrp);
    declare_parameter("stdyaw", config_.stdyaw);

    declare_parameter("enable_drift_odom", config_.enable_drift_odom);
    declare_parameter("stdvdriftxyz", config_.stdvdriftxyz);
    declare_parameter("stdvdriftyaw", config_.stdvdriftyaw);
    declare_parameter("vdriftx", config_.vdriftx);
    declare_parameter("vdrifty", config_.vdrifty);
    declare_parameter("vdriftz", config_.vdriftz);
    declare_parameter("vdriftyaw", config_.vdriftyaw);

    read_config();
    param_cb_handle_ = add_on_set_parameters_callback(
        std::bind(&So3DisturbanceGenerator::on_set_parameters, this, std::placeholders::_1));

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&So3DisturbanceGenerator::odom_callback, this, std::placeholders::_1));
    pubo_ = create_publisher<nav_msgs::msg::Odometry>("noisy_odom", 10);
    pubc_ = create_publisher<geometry_msgs::msg::PoseStamped>("correction", 10);
    pubf_ = create_publisher<geometry_msgs::msg::Vector3>("force_disturbance", 10);
    pubm_ = create_publisher<geometry_msgs::msg::Vector3>("moment_disturbance", 10);

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / 100.0),
                                std::bind(&So3DisturbanceGenerator::set_disturbance, this));
  }

private:
  void read_config()
  {
    config_.fxy = get_parameter("fxy").as_double();
    config_.stdfxy = get_parameter("stdfxy").as_double();
    config_.fz = get_parameter("fz").as_double();
    config_.stdfz = get_parameter("stdfz").as_double();
    config_.mrp = get_parameter("mrp").as_double();
    config_.stdmrp = get_parameter("stdmrp").as_double();
    config_.myaw = get_parameter("myaw").as_double();
    config_.stdmyaw = get_parameter("stdmyaw").as_double();

    config_.enable_noisy_odom = get_parameter("enable_noisy_odom").as_bool();
    config_.stdxyz = get_parameter("stdxyz").as_double();
    config_.stdvxyz = get_parameter("stdvxyz").as_double();
    config_.stdrp = get_parameter("stdrp").as_double();
    config_.stdyaw = get_parameter("stdyaw").as_double();

    config_.enable_drift_odom = get_parameter("enable_drift_odom").as_bool();
    config_.stdvdriftxyz = get_parameter("stdvdriftxyz").as_double();
    config_.stdvdriftyaw = get_parameter("stdvdriftyaw").as_double();
    config_.vdriftx = get_parameter("vdriftx").as_double();
    config_.vdrifty = get_parameter("vdrifty").as_double();
    config_.vdriftz = get_parameter("vdriftz").as_double();
    config_.vdriftyaw = get_parameter("vdriftyaw").as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
      const std::vector<rclcpp::Parameter> &)
  {
    read_config();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    noisy_odom_.header = msg->header;
    correction_.header = msg->header;
    // Get odom
    colvec pose(6);
    colvec vel(3);
    pose(0)        = msg->pose.pose.position.x;
    pose(1)        = msg->pose.pose.position.y;
    pose(2)        = msg->pose.pose.position.z;
    colvec q       = zeros<colvec>(4);
    q(0)           = msg->pose.pose.orientation.w;
    q(1)           = msg->pose.pose.orientation.x;
    q(2)           = msg->pose.pose.orientation.y;
    q(3)           = msg->pose.pose.orientation.z;
    pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
    vel(0)         = msg->twist.twist.linear.x;
    vel(1)         = msg->twist.twist.linear.y;
    vel(2)         = msg->twist.twist.linear.z;
    // Drift Odom
    static colvec drift_pose      = pose;
    static colvec drift_vel       = vel;
    static colvec correction_pose = zeros<colvec>(6);
    static colvec prev_pose       = pose;
    static rclcpp::Time prev_pose_t(msg->header.stamp);
    if (config_.enable_drift_odom)
    {
      double dt       = (rclcpp::Time(msg->header.stamp) - prev_pose_t).seconds();
      prev_pose_t     = rclcpp::Time(msg->header.stamp);
      colvec d        = pose_update(pose_inverse(prev_pose), pose);
      prev_pose       = pose;
      d(0)           += (config_.vdriftx   + config_.stdvdriftxyz * as_scalar(randn(1))) * dt;
      d(1)           += (config_.vdrifty   + config_.stdvdriftxyz * as_scalar(randn(1))) * dt;
      d(2)           += (config_.vdriftz   + config_.stdvdriftxyz * as_scalar(randn(1))) * dt;
      d(3)           += (config_.vdriftyaw + config_.stdvdriftyaw * as_scalar(randn(1))) * dt;
      drift_pose      = pose_update(drift_pose, d);
      drift_vel       = ypr_to_R(drift_pose.rows(3,5)) * trans(ypr_to_R(pose.rows(3,5))) * vel;
      correction_pose = pose_update(pose, pose_inverse(drift_pose));
    }
    else
    {
      drift_pose      = pose;
      drift_vel       = vel;
      correction_pose = zeros<colvec>(6);
    }
    // Noisy Odom
    static colvec noisy_pose = drift_pose;
    static colvec noisy_vel  = drift_vel;
    if (config_.enable_noisy_odom)
    {
      colvec noise_pose = zeros<colvec>(6);
      colvec noise_vel  = zeros<colvec>(3);
      noise_pose(0) = config_.stdxyz  * as_scalar(randn(1));
      noise_pose(1) = config_.stdxyz  * as_scalar(randn(1));
      noise_pose(2) = config_.stdxyz  * as_scalar(randn(1));
      noise_pose(3) = config_.stdyaw  * as_scalar(randn(1));
      noise_pose(4) = config_.stdrp   * as_scalar(randn(1));
      noise_pose(5) = config_.stdrp   * as_scalar(randn(1));
      noise_vel(0)  = config_.stdvxyz * as_scalar(randn(1));
      noise_vel(1)  = config_.stdvxyz * as_scalar(randn(1));
      noise_vel(2)  = config_.stdvxyz * as_scalar(randn(1));
      noisy_pose = drift_pose + noise_pose;
      noisy_vel  = drift_vel  + noise_vel;
      noisy_odom_.pose.covariance[0+0*6]         = config_.stdxyz * config_.stdxyz;
      noisy_odom_.pose.covariance[1+1*6]         = config_.stdxyz * config_.stdxyz;
      noisy_odom_.pose.covariance[2+2*6]         = config_.stdxyz * config_.stdxyz;
      noisy_odom_.pose.covariance[(0+3)+(0+3)*6] = config_.stdyaw * config_.stdyaw;
      noisy_odom_.pose.covariance[(1+3)+(1+3)*6] = config_.stdrp  * config_.stdrp;
      noisy_odom_.pose.covariance[(2+3)+(2+3)*6] = config_.stdrp  * config_.stdrp;
      noisy_odom_.twist.covariance[0+0*6]        = config_.stdvxyz * config_.stdvxyz;
      noisy_odom_.twist.covariance[1+1*6]        = config_.stdvxyz * config_.stdvxyz;
      noisy_odom_.twist.covariance[2+2*6]        = config_.stdvxyz * config_.stdvxyz;
    }
    else
    {
      noisy_pose = drift_pose;
      noisy_vel  = drift_vel;
      noisy_odom_.pose.covariance[0+0*6]         = 0;
      noisy_odom_.pose.covariance[1+1*6]         = 0;
      noisy_odom_.pose.covariance[2+2*6]         = 0;
      noisy_odom_.pose.covariance[(0+3)+(0+3)*6] = 0;
      noisy_odom_.pose.covariance[(1+3)+(1+3)*6] = 0;
      noisy_odom_.pose.covariance[(2+3)+(2+3)*6] = 0;
      noisy_odom_.twist.covariance[0+0*6]        = 0;
      noisy_odom_.twist.covariance[1+1*6]        = 0;
      noisy_odom_.twist.covariance[2+2*6]        = 0;
    }
    // Assemble and publish odom
    noisy_odom_.pose.pose.position.x    = noisy_pose(0);
    noisy_odom_.pose.pose.position.y    = noisy_pose(1);
    noisy_odom_.pose.pose.position.z    = noisy_pose(2);
    noisy_odom_.twist.twist.linear.x    = noisy_vel(0);
    noisy_odom_.twist.twist.linear.y    = noisy_vel(1);
    noisy_odom_.twist.twist.linear.z    = noisy_vel(2);
    colvec noisy_q                      = R_to_quaternion(ypr_to_R(noisy_pose.rows(3,5)));
    noisy_odom_.pose.pose.orientation.w = noisy_q(0);
    noisy_odom_.pose.pose.orientation.x = noisy_q(1);
    noisy_odom_.pose.pose.orientation.y = noisy_q(2);
    noisy_odom_.pose.pose.orientation.z = noisy_q(3);
    pubo_->publish(noisy_odom_);
    // Check time interval and publish correction
    static rclcpp::Time prev_correction_t(msg->header.stamp);
    if ((rclcpp::Time(msg->header.stamp) - prev_correction_t).seconds() > 1.0 / CORRECTION_RATE)
    {
      prev_correction_t              = rclcpp::Time(msg->header.stamp);
      correction_.pose.position.x    = correction_pose(0);
      correction_.pose.position.y    = correction_pose(1);
      correction_.pose.position.z    = correction_pose(2);
      colvec correction_q            = R_to_quaternion(ypr_to_R(correction_pose.rows(3,5)));
      correction_.pose.orientation.w = correction_q(0);
      correction_.pose.orientation.x = correction_q(1);
      correction_.pose.orientation.y = correction_q(2);
      correction_.pose.orientation.z = correction_q(3);
      pubc_->publish(correction_);
    }
  }

  void set_disturbance()
  {
    geometry_msgs::msg::Vector3 f;
    geometry_msgs::msg::Vector3 m;
    f.x = config_.fxy  + config_.stdfxy  * as_scalar(randn(1));
    f.y = config_.fxy  + config_.stdfxy  * as_scalar(randn(1));
    f.z = config_.fz   + config_.stdfz   * as_scalar(randn(1));
    m.x = config_.mrp  + config_.stdmrp  * as_scalar(randn(1));
    m.y = config_.mrp  + config_.stdmrp  * as_scalar(randn(1));
    m.z = config_.myaw + config_.stdmyaw * as_scalar(randn(1));
    pubf_->publish(f);
    pubm_->publish(m);
  }

  DisturbanceConfig config_;
  nav_msgs::msg::Odometry noisy_odom_;
  geometry_msgs::msg::PoseStamped correction_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubo_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubc_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubf_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubm_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<So3DisturbanceGenerator>());
  rclcpp::shutdown();
  return 0;
}
