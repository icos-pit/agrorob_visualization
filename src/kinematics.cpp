
#include <boost/array.hpp>
#include <boost/numeric/odeint/integrate/integrate.hpp>
#include "agrorob_visualization/kinematics.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace boost::numeric::odeint;
using namespace agrorob_kinematics;

Kinematics::Kinematics(const double& dt): dt_(dt)
{
    wheel_base = 3.0;
    wheel_diameter = 0.78;
    qcar = {0.0, 0.0, 0.0};
    ucar = {0.0, 0.0};
    odom_pose = std::make_shared<geometry_msgs::msg::Pose>();
}

inline vec3 Kinematics::RDCarKinematicsGPRear(double wheel_base, vec3 q, vec2& ucar)
{
    vec3 dq;
    dq[0] = ucar[1]*(1/wheel_base)*tan(ucar[0]);
    dq[1] = ucar[1]*cos(q[0]);
    dq[2] = ucar[1]*sin(q[0]);
    return dq;
}

void Kinematics::calculate_odom_pose()
{
    auto simFunc = [&](const vec3& q, vec3& dq, const double call_back_duration_s)
    {
        dq = this->RDCarKinematicsGPRear(wheel_base, q, ucar);
    };

    integrate(simFunc, qcar, 0.0, dt_, dt_);

    odom_pose->position.x =  qcar[1];
    odom_pose->position.y =  qcar[2];
    odom_pose->position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, qcar[0]);
    odom_pose->orientation.x = q.x();
    odom_pose->orientation.y = q.y();
    odom_pose->orientation.z = q.z();
    odom_pose->orientation.w = q.w();
}

double Kinematics::get_wheel_diameter()
{
    return wheel_diameter;
}

void Kinematics::set_ucar(const vec2& u)
{
    ucar = u;
}




