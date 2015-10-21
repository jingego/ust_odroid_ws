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
