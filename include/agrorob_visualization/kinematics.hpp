#ifndef __KINEMATICS_HPP__
#define __KINEMATICS_HPP__

#include <boost/array.hpp>
#include "geometry_msgs/msg/pose.hpp"

namespace agrorob_kinematics
{
    using vec3 = boost::array<double, 3>; //heading, x, y 
    using vec2 = boost::array<double, 2>; //steering angle, velocity 
    

    class Kinematics
    {
    public:
        Kinematics();
        void calculate_odom_pose();
    private:

        // auto simFunc = [&](const vec& q, vec3& dq, const double call_back_duration_s)
        // {
        //     dq = RDCarKinematicsGPRear(1.0, q, {0.0, 0.0});
        // };

        
        
        std::shared_ptr<sensor_msgs::msg::Pose> odom_pose;
        double wheel_base;
        vec3 qcar;
    };


}




#endif // __KINEMATICS_HPP__
