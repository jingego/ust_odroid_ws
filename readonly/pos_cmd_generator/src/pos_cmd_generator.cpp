#include    "ros/ros.h"
#include    "ros/time.h"
#include    "mavlink_message/PositionCommand.h"
#include    "mavlink_message/quad_state.h"
#include    "nav_msgs/Odometry.h"
#include    <iostream>
#include    "Eigen/Eigen"
#include    <cmath>
#include    <time.h>
#include    <cstdlib>
#include    <string.h>
#include    "rotation.h"
#include    "trajectory.h"
using namespace     std;
using namespace     Eigen;

ros::Publisher      poscmd_pub;
ros::Subscriber     pose_sub;
ros::Subscriber     state_sub;

Eigen::Vector3d     now_position, now_velocity;
Eigen::Quaterniond  q_ukf;
Eigen::Vector3d  rpy;
ros::Subscriber     auto_pos_cmd;
ros::Time  now_t;
void pose_callback( const nav_msgs::Odometry &pos )
{
    now_t   = pos.header.stamp;
    now_position(0)     = pos.pose.pose.position.x;
    now_position(1)     = pos.pose.pose.position.y;
    now_position(2)     = pos.pose.pose.position.z;
    q_ukf.w()     = pos.pose.pose.orientation.w;
    q_ukf.x()     = pos.pose.pose.orientation.x;
    q_ukf.y()     = pos.pose.pose.orientation.y;
    q_ukf.z()     = pos.pose.pose.orientation.z;
    now_velocity(0)     = pos.twist.twist.linear.x;
    now_velocity(1)     = pos.twist.twist.linear.y;
    now_velocity(2)     = pos.twist.twist.linear.z;
    rpy = Qbw2RPY(q_ukf);
}

uint8_t    now_state = 0xff;
Eigen::Vector3d  init_pos, init_vel;
ros::Time init_t;
bool poscmd_init, auto_init, trajectory_ok;
double yaw_tar;

void quad_state_callback(const mavlink_message::quad_state &state)
{
    if (state.offboard_state & 0x01)
        now_state   |= 0x01;
    else if (state.offboard_state & 0x02 )
        now_state   |= 0x02;
    if (((now_state & 0x0f ) == 0x01) || ((now_state & 0x0f) == 0x09)) // 0x01 from manal to hover; 0x09 from auto to hover
    {
        cout << "hover" << endl;
        //get the init position and yaw as the init information
        init_pos(0) = now_position.x();
        init_pos(1) = now_position.y();
        init_pos(2) = now_position.z();
        init_vel(0) = now_velocity.x();
        init_vel(1) = now_velocity.y();
        init_vel(2) = now_velocity.z();
        yaw_tar     = rpy.z();
        cout << "now position: " << now_position.transpose()  << " \t yaw: " << yaw_tar << endl;
        poscmd_init = true;
        auto_init   = false;
        trajectory_ok = false;
    }
    else if ( (now_state & 0x0f) == 0x06) // 0x06 from hover to auto
    {
        cout << "auto" << endl;
        init_pos(0) = now_position.x();
        init_pos(1) = now_position.y();
        init_pos(2) = now_position.z();
        init_vel(0) = now_velocity.x();
        init_vel(1) = now_velocity.y();
        init_vel(2) = now_velocity.z();
        if(trajectory_generator(init_pos, init_vel))
            trajectory_ok = true;
        init_t      = now_t;
        auto_init   = true;
    }
    else if ( (now_state & 0x0f) == 0x04) //from hover to manual
    {
        cout << "manual" << endl;
        poscmd_init = false;
        trajectory_ok = false;
    }
    now_state <<= 2;
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "pos_cmd");
    ros::NodeHandle     set_pos("~");
    poscmd_pub          =  set_pos.advertise<mavlink_message::PositionCommand>("des_pos", 1000);
    pose_sub       =  set_pos.subscribe("odom", 1000, pose_callback);
    state_sub           =  set_pos.subscribe("/mavlink/quad_state", 1000, quad_state_callback);
    poscmd_init    = false;
    auto_init      = false;
    ros::NodeHandle  nh;
    ros::NodeHandle     n("~");

    printf("start setting position\n");
    ros::Rate loop_rate(100);

    mavlink_message::PositionCommand    set_p;
    while (ros::ok())
    {
        set_p.header.stamp = now_t;
        if ( poscmd_init )
        {
            if ( !auto_init )
            {
                set_p.header.frame_id = (string)"hover";
                set_p.position.x = init_pos.x();
                set_p.position.y = init_pos.y();
                set_p.position.z = init_pos.z();
                set_p.velocity.x = 0;
                set_p.velocity.y = 0;
                set_p.velocity.z = 0;
                set_p.accelerate.x = 0;
                set_p.accelerate.y = 0;
                set_p.accelerate.z = 0;
            }
            else if (trajectory_ok)
            {
                //use dT to get the desired position, velocity and acceleration
                double dT    = (now_t - init_t).toSec();
                Vector3d des_pos, des_vel, des_acc;
                trajectory_control(dT, now_position, now_velocity, des_pos, des_vel, des_acc);
                set_p.position.x = des_pos.x();
                set_p.position.y = des_pos.y();
                set_p.position.z = des_pos.z();
                set_p.velocity.x = des_vel.x();
                set_p.velocity.y = des_vel.y();
                set_p.velocity.z = des_vel.z();
                set_p.accelerate.x = des_acc.x();
                set_p.accelerate.y = des_acc.y();
                set_p.accelerate.z = des_acc.z();
            }
        }
        else
        {
            set_p.header.frame_id = (string)"null";
        }

        set_p.yaw = yaw_tar;
        poscmd_pub.publish(set_p);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

