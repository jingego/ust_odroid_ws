#ifndef trajectory_h
#define trajectory_h

using namespace Eigen;
/*
 * this function is used to generate trajectory coefficients.
 * input: 
 * init_pos -> the initial position for trajectory
 * init_vel -> the initial velocity for trajectory
 */
bool trajectory_generator(Vector3d init_pos, Vector3d init_vel)
{
    double v_abs[3], des_pos[3];
    v_abs={0.1,0,0};
   	des_pos[0]=init_pos[0]+v_abs[0]*dT;/*how to dT?*/
	des_pos[1]=init_pos[1]+v_abs[1]*dT;
	des_pos[2]=init_pos[2]+v_abs[2]*dT;
    return true;
}
/*
 * this function is to get desired states for specific trajectory, just generated, at time dt.
 * input:
 * dT   -> the time
 * now_pos, now_vel -> maybe useless
 *
 * output:
 * desired_pos -> desired position at dT
 * desired_vel -> desired velocity at dT
 * desired_acc -> desired acceleration at dT
 */
void trajectory_control(const double dT, 
        const Vector3d now_pos,
        const Vector3d now_vel,
        Vector3d & desired_pos,
        Vector3d & desired_vel,
        Vector3d & desired_acc
        )
{

}
#endif
