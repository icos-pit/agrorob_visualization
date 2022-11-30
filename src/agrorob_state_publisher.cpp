#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"


#include "agrorob_msgs/msg/robot_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/path.hpp"

#include "agrorob_visualization/agrorob_state_publisher.hpp"
#include "agrorob_visualization/kinematics.hpp"

using namespace std;
using std::placeholders::_1;
using namespace agrorob_visualization;
using namespace agrorob_kinematics;

AgrorobStatePublisher::AgrorobStatePublisher(): Node("agrorob_state_publisher"), 
call_back_duration(50ms), kinematics(std::chrono::duration<double>(call_back_duration).count())
{   

    call_back_duration_s = std::chrono::duration<double>(call_back_duration).count(); //call back duration in seconds for kinematics

    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10); 

    estimated_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/agrorob/estimated_path", 10);

    robot_state_sub_ = this->create_subscription<agrorob_msgs::msg::RobotState>("/agrorob/robot_state", 10, std::bind(&AgrorobStatePublisher::robot_state_callback, this, _1));
    
    timer_ = this->create_wall_timer(call_back_duration, std::bind(&AgrorobStatePublisher::timer_callback, this));
    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    joint_states_msg = std::make_shared<sensor_msgs::msg::JointState>();
    estimated_path_msg = std::make_shared<nav_msgs::msg::Path>();
    estimated_path_msg->header.frame_id = "odom";
    odom_trasform_msg = initialize_tf("odom", "base_link");

    // joint_states_msg->name = {"body_shin_FR", "shin_wheel_FR", "body_shin_FL", "shin_wheel_FL", 
    //   "body_shin_RR", "shin_wheel_RR", "body_shin_RL", "shin_wheel_RL"};
    joint_states_msg->name.push_back("body_shin_FR");
    joint_states_msg->name.push_back("shin_wheel_FR");
    joint_states_msg->name.push_back("body_shin_FL");
    joint_states_msg->name.push_back("shin_wheel_FL");

    joint_states_msg->name.push_back("body_shin_RR");
    joint_states_msg->name.push_back("shin_wheel_RR");
    joint_states_msg->name.push_back("body_shin_RL");
    joint_states_msg->name.push_back("shin_wheel_RL");

    for(int i = 0; i < 8 ; i++)
    {
      joint_states_msg->position.push_back(0.0);
      joint_states_msg->velocity.push_back(0.0);
    }
}

void AgrorobStatePublisher::timer_callback()
{


  kinematics.calculate_odom_pose();
  auto time_now = this->get_clock()->now();

  odom_trasform_msg->transform.translation.x = kinematics.odom_pose->position.x;
  odom_trasform_msg->transform.translation.y = kinematics.odom_pose->position.y;
  odom_trasform_msg->transform.translation.z = 1.60;
  odom_trasform_msg->transform.rotation = kinematics.odom_pose->orientation;

  std::shared_ptr<geometry_msgs::msg::PoseStamped> pose = std::make_shared<geometry_msgs::msg::PoseStamped>();

  pose->header.stamp = time_now;
  pose->header.frame_id = "odom";
  pose->pose.position = kinematics.odom_pose->position;


  estimated_path_msg->poses.push_back(*pose); //TODO: create function
  if(estimated_path_msg->poses.size() > 10000)
    estimated_path_msg->poses.erase(estimated_path_msg->poses.begin(), estimated_path_msg->poses.begin() + 1000);

  
  odom_trasform_msg->header.stamp = time_now;
  joint_states_msg->header.stamp = time_now;
  estimated_path_msg->header.stamp = time_now;

  odom_tf_broadcaster_->sendTransform(*odom_trasform_msg);
  joint_states_pub_->publish(*joint_states_msg);
  estimated_path_pub_->publish(*estimated_path_msg);

  // RCLCPP_INFO(this->get_logger(), "duration: %f", call_back_duration_s);
  
}

void AgrorobStatePublisher::robot_state_callback(const agrorob_msgs::msg::RobotState& robot_state_msg)
{
  joint_states_msg->position[0] = robot_state_msg.right_front_wheel_turn_angle_rad; 
  joint_states_msg->position[2] = robot_state_msg.left_front_wheel_turn_angle_rad;
  joint_states_msg->position[4] = robot_state_msg.right_rear_wheel_turn_angle_rad;
  joint_states_msg->position[6] = robot_state_msg.left_rear_wheel_turn_angle_rad;

  joint_states_msg->velocity[1] = robot_state_msg.right_front_wheel_rotational_speed_rad_s;
  joint_states_msg->velocity[3] = robot_state_msg.left_front_wheel_rotational_speed_rad_s;
  joint_states_msg->velocity[5] = robot_state_msg.right_rear_wheel_rotational_speed_rad_s;
  joint_states_msg->velocity[7] = robot_state_msg.left_rear_wheel_rotational_speed_rad_s;

  

  update_ucar();

}    

void AgrorobStatePublisher::update_ucar()
{
  vec2 ucar_;

  ucar_[0] = ( joint_states_msg->position[0] + joint_states_msg->position[2] ) / 2;
  
  double rear_wheels_mean_vel_rad_s = ( joint_states_msg->velocity[5] + joint_states_msg->velocity[7] ) / 2;
  ucar_[1] = (rear_wheels_mean_vel_rad_s * kinematics.get_wheel_diameter());

  kinematics.set_ucar(ucar_);
  
}

geometry_msgs::msg::TransformStamped::SharedPtr AgrorobStatePublisher::initialize_tf(const string& parent_link , const string& child_link )
{
  auto tf_ = std::make_shared<geometry_msgs::msg::TransformStamped>();
  tf_->header.frame_id = parent_link.c_str();
  tf_->child_frame_id = child_link.c_str();

  return tf_;
}


   

