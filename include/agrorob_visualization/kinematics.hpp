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
        Kinematics(const double& dt);
        void calculate_odom_pose();
        double get_wheel_diameter();
        void set_ucar(const vec2& u);
        std::shared_ptr<geometry_msgs::msg::Pose> odom_pose; //TODO: change to pose stamped

    private:
    
        inline vec3 RDCarKinematicsGPRear(double wheel_base, vec3 q, vec2& ucar);
        vec3 qcar;
        vec2 ucar;
        double wheel_base;
        double wheel_diameter;
        double dt_;
        vec2 ucar_fake;

    };


}




#endif // __KINEMATICS_HPP__
