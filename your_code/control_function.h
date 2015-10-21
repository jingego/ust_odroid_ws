#ifndef control_funtion_h
#define control_funtion_h

using namespace Eigen;
/*please fill this controller function
 * input: 
 * des_pos -> desired position
 * des_vel -> desired velocity
 * des_acc -> desired acceleration
 * des_yaw -> desired yaw angle of the offboard frame
 * R_ukf   -> the body attitude in the offboard frame
 * R_px4   -> the body attitude in the onboard frame
 * now_pos -> body position, of course, in the offboard frame
 * now_vel -> body velocity in the offboard frame
 * Kp      -> P gain for position loop
 * Kd      -> P gain for velocity loop
 * Mass    -> quality of the quadrotor
 * ThrustGain -> the coeffient that tranfers force (Unit: N) to target thrust (Range: 0 - 1)
 *
 * output:
 * target_quaternion -> target attitude for autopilot (pixhawk), of course, in the onboard frame
 * target_thrust     -> target thrust of the quadrotor, Range: 0.0 - 1.0
 * */
void SO3Control_function( const Eigen::Vector3d des_pos,
        const Eigen::Vector3d des_vel,
        const Eigen::Vector3d des_acc,
        double des_yaw,
        const Eigen::Matrix3d R_ukf,
        const Eigen::Matrix3d R_px4,
        const Eigen::Vector3d now_pos,
        const Eigen::Vector3d now_vel,
        const Eigen::Vector3d Kp,
        const Eigen::Vector3d Kd,
        const double Mass,
        const double ThrustGain,
        Eigen::Quaterniond &target_quaternion,
        double &target_thrust
        )
{

}

#endif
