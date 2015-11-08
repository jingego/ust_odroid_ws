#ifndef control_funtion_h
#define control_funtion_h

#include <math.h>
/*please fill this controller function
 * input:
 * des_pos -> desired position
 * des_vel -> desired velocity
 * des_acc -> desired acceleration
 * des_yaw -> desired yaw angle
 * now_pos -> now psition
 * now_vel -> body velocity
 * Kp      -> P gain for position loop
 * Kd      -> P gain for velocity loop
 * Mass    -> quality of the quadrotor
 *
 
 * output:
 * rpy -> target attitude for autopilot
 * target_thrust     -> target thrust of the quadrotor
 * */
void SO3Control_function( const double des_pos[3],
                          const double des_vel[3],
                          const double des_acc[3],
                          const double des_yaw,
                          const double now_pos[3],
                          const double now_vel[3],
                          const double now_yaw,
                          const double Kp[3],
                          const double Kd[3],
                          const double Mass,
                          const double Gravity,
                          double rpy[3],
                          double &target_thrust
                        )
{
<<<<<<< HEAD
    double err_pos[3], err_vel[3], rc2dot[3], F;
    //get the desired acceleration
    for (int i = 0; i < 3; ++i)
    {
       err_pos[i] = des_pos[i] - now_pos[i];
       err_vel[i] = des_vel[i] - now_vel[i];
       rc2dot[i] = Kp[i] * err_pos[i] + Kd[i] * err_vel[i];
    }

    target_thrust = Mass * (Gravity + rc2dot[2]);
    rpy[0] = (rc2dot[0]*sin(now_yaw)-rc2dot[1]*cos(now_yaw))/Gravity;
    rpy[1] = (rc2dot[0]*cos(now_yaw)+rc2dot[1]*sin(now_yaw))/Gravity;
    rpy[2] = des_yaw;    //des_yaw  
=======
	
>>>>>>> c3a1f09c21b9c6800015c59e084ed2f3b93c905f
}
#endif
