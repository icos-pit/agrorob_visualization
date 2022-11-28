
#include <boost/array.hpp>
#include <boost/numeric/odeint/integrate/integrate.hpp>
#include "agrorob_visualization/kinematics.hpp"

using namespace boost::numeric::odeint;
using namespace agrorob_kinematics;

Kinematics::Kinematics(): wheel_base(3.0)
{
    qcar = {0.0, 0.0, 0.0};
    odom_pose = std::make_shared<sensor_msgs::msg::Pose>();
}

void Kinematics::calculate_odom_pose(){


    integrate(simFunc, qCar, 0.0, 1/(Real)controlLoopRate, 1/(Real)controlLoopRate);
    odom_pose->position 
    odom_pose->orientation 
}

// inline vec3 RDCarKinematicsGPRear(double wheel_base, vec3 q, vec2& input);
// {
//     vec3 dq;
//     // dq[0] = input.longitudinalVelocity*(1/params.L)*tan(input.steeringAngle);
//     // dq[1] = input.longitudinalVelocity*cos(q[0]);
//     // dq[2] = input.longitudinalVelocity*sin(q[0]);
//     return dq;
// }

auto simFunc = [&](const vec& q, vec3& dq, const double call_back_duration_s)
{
    dq = RDCarKinematicsGPRear(wheel_base, q, {0.0, 0.0});
};

integrate(simFunc, qCar, 0.0, 1/(Real)controlLoopRate, 1/(Real)controlLoopRate);



