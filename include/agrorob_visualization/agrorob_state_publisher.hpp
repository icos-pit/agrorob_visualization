#ifndef __AGROROB_STATE_PUBLISHER_HPP__
#define __AGROROB_STATE_PUBLISHER_HPP__

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "agrorob_msgs/msg/robot_state.hpp"
#include "agrorob_visualization/kinematics.hpp"





namespace agrorob_visualization
{
    using namespace std;
    using namespace agrorob_kinematics;
    using duration_ms = std::chrono::duration<int, std::milli>;
    
    

    class AgrorobStatePublisher : public rclcpp::Node
    {
    public:
        AgrorobStatePublisher();

    private:

        geometry_msgs::msg::TransformStamped::SharedPtr initialize_tf(const string& parent_link , const string& child_link );

        void robot_state_callback(const agrorob_msgs::msg::RobotState& robot_state_ms);
        void timer_callback();
        void update_ucar();
        

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
        rclcpp::Subscription<agrorob_msgs::msg::RobotState>::SharedPtr robot_state_sub_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::shared_ptr<sensor_msgs::msg::JointState> joint_states_msg;
        std::shared_ptr<geometry_msgs::msg::TransformStamped> odom_trasform_msg;

        duration_ms call_back_duration;
        double call_back_duration_s;

        Kinematics kinematics;
        
    };

   


}


#endif // __AGROROB_STATE_PUBLISHER_NODE_HPP__