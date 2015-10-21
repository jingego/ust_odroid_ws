#include "ros/ros.h"
#include <iostream>
#include "mavlink_message/set_att_offboard.h"
#include "mavlink_message/PositionCommand.h"
#include "mavlink_message/att_onboard.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "nav_msgs/Odometry.h"
#include "Eigen/Geometry"
#include "rotation.h"
#include <iostream>
#include <cmath>
#include <string.h>
#include <time.h>
#include "control_function.h"

using namespace std;
using namespace std;
using namespace Eigen;

//ros topic
ros::Publisher      set_att_pub;


Eigen::Vector3d     des_pos, des_vel, des_acc;
double  des_yaw, Mass, ThrustGain;
bool    setp_state;
void des_pos_callback(const mavlink_message::PositionCommand &cmd)
{
    if (cmd.header.frame_id != string("null"))// || cmd.header.frame_id == string("hover"))
        setp_state  = true;
    else
        setp_state = false;

    des_pos(0) = cmd.position.x;
    des_pos(1) = cmd.position.y;
    des_pos(2) = cmd.position.z;

    des_vel(0) = cmd.velocity.x;
    des_vel(1) = cmd.velocity.y;
    des_vel(2) = cmd.velocity.z;

    des_acc(0) = cmd.accelerate.x;
    des_acc(1) = cmd.accelerate.y;
    des_acc(2) = cmd.accelerate.z;

    des_yaw = cmd.yaw;
}

Quaterniond q_px4;
Matrix3d R_px4;
void att_px4_callback(const mavlink_message::att_onboard::ConstPtr att_px4)
{
    q_px4.w() = att_px4->Q_b2w.w;
    q_px4.x() = att_px4->Q_b2w.x;
    q_px4.y() = att_px4->Q_b2w.y;
    q_px4.z() = att_px4->Q_b2w.z;
    R_px4 = q_px4.normalized().toRotationMatrix();
}
Eigen::Vector3d Kp, Kd;
void local_pos_callback(const nav_msgs::Odometry &pos)
{
    Matrix3d R_ukf;
    Quaterniond Q_ukf;
    Vector3d now_pos, now_vel;
    Q_ukf.w() = pos.pose.pose.orientation.w;
    Q_ukf.x() = pos.pose.pose.orientation.x;
    Q_ukf.y() = pos.pose.pose.orientation.y;
    Q_ukf.z() = pos.pose.pose.orientation.z;
    R_ukf = Q_ukf.normalized().toRotationMatrix();
    now_pos(0) = pos.pose.pose.position.x;
    now_pos(1) = pos.pose.pose.position.y;
    now_pos(2) = pos.pose.pose.position.z;
    now_vel(0) = pos.twist.twist.linear.x;
    now_vel(1) = pos.twist.twist.linear.y;
    now_vel(2) = pos.twist.twist.linear.z;
    mavlink_message::set_att_offboard att_offboard;
    att_offboard.header.stamp = pos.header.stamp;
    att_offboard.type_mask = 7;
    att_offboard.body_yaw_rate  = 0;
    att_offboard.body_roll_rate = 0;
    att_offboard.body_pitch_rate = 0;
    att_offboard.target_system = 0;
    att_offboard.target_component = 0;
    Quaterniond  target_q;
    double thrust = 0;
    if (setp_state)
        SO3Control_function( des_pos,
                des_vel,
                des_acc,
                des_yaw,
                R_ukf,
                R_px4,
                now_pos,
                now_vel,
                Kp,
                Kd,
                Mass,
                ThrustGain,
                target_q,
                thrust
                );
    att_offboard.q1 = target_q.w();
    att_offboard.q2 = target_q.x();
    att_offboard.q3 = target_q.y();
    att_offboard.q4 = target_q.z();
    att_offboard.thrust = thrust;
    set_att_pub.publish(att_offboard);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "so3_control");
    ros::NodeHandle pos2att("~");
    set_att_pub     = pos2att.advertise<mavlink_message::set_att_offboard>("/mavlink/set_att", 1000);
    ros::Subscriber local_pos_sub   = pos2att.subscribe("odom",   1000,   local_pos_callback);
    ros::Subscriber des_pos_sub     = pos2att.subscribe("des_pos", 1000, des_pos_callback);
    ros::Subscriber att_px4_sub     = pos2att.subscribe("q_pixhawk", 100, att_px4_callback); 
    double Px, Py, Pz, Dx, Dy, Dz;
    pos2att.param("Px", Px,  5.0);
    pos2att.param("Py", Py,  5.0);
    pos2att.param("Pz", Pz,  5.0);
    pos2att.param("Dx", Dx,  3.0);
    pos2att.param("Dy", Dy,  3.0);
    pos2att.param("Dz", Dz,  3.0);
    pos2att.param("Mass", Mass, 1.2);
    pos2att.param("ThrustGain", ThrustGain, 36.0);
    Kp << Px, Py, Pz;
    Kd << Dx, Dy, Dz;
    ros::spin();
    return 0;
}
