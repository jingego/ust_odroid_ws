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
	/* line trajectory	*/
	v_abs={0.1,0,0};
 	des_pos[0]=init_pos[0]+v_abs[0]*dT;/*how to dT?*/
	des_pos[1]=init_pos[1]+v_abs[1]*dT;
	des_pos[2]=init_pos[2]+v_abs[2]*dT;
	/* trajectory 2 with matlab path1*/
	double ax[5],ay[5],az[5];
	/* trajectory 3 with matlab path2 
	 double ax[5],ay[5],az[5];
	if (t<=3)
		ax={0.0124,-0.1076,0.2622,0,0,0.5};
		ay={0,0,0,0,0.5};
		az=(0,0,0,0,1};
	else if (t<=6)
		ax={-0.0124,0.3092,-2.9713,13.6331,-29.5385,26};
		ay={0.0124,-0.3092,2.9713,-13.6331,30,-25};
		az={0,0,0,0,1};
	else if (t<=9)
		ax={-0.0124,0.5109,-8.3022,66.4615,-262.1538,410};
		ay={-0.0124,0.5109,-8.3022,66.4615,-262.1538,407};
		az={0,0,0,0,1};
	else if (t<=12)
		ax=1000*{0,-0.0009,0.0268,-0.3920,2.8505,-8.2555};
		ay=1000*{0,0,-0.0163,0.184,-1.0343,2.3105};
		az={0,0,0,0,1};
	else if (t<=15)
		ax=	1000*{0,0,0.0268,-0.392,2.8505,8.2555};
		ay=1000*{0,-0.0009,0.0268,-0.3920,2.85,-8.2495};
		az=az={0,0,0,0,1};
	else if (t<=18)
		ax=10000*{0,0.0001,-0.004,0.0716,-0.638,2.2682};
		ay=10000*{0,-0.0001,0.004,-0.0716,0.6381,-2.2687};
		az={0,0,0,0,1};
	else if (t<=21)
		ax=10000*{0,0.0001,-0.0056,0.1181,-1.2462,5.2490};
		ay=10000*{0,0.0001,-0.0056,0.1181,-1.2461,5.2481};
		az={0,0,0,0,1};
	else
		ax=100000*{0,0,0.0007,-0.0181,0.2209,-1.0752};
		ay=100000*{0,0,-0.0007,0.0181,-0.221,1.0753};
		az={0,0,0,0,1};
	path 2 finished*/
	
	
		
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
