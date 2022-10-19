#include "include/CRocketsimulation.h"

void CRocketsimulation::Controller_Module(struct st_Rocket Rocket, struct st_Guidance Guidance, struct st_Ctr* Ctr)
{

	double M_delT = Guidance.Guidance_CMD[4] * Rocket.Moment_Arm[1] / Rocket.MOI[1][1];

	double val1 = Ctr->sigma1 * (Rocket.Att[0] - Guidance.Guidance_CMD[1]) * 180 / Pi + Ctr->C1;
	double val2 = Ctr->sigma2 * (Rocket.Att[0] - Guidance.Guidance_CMD[1]) * 180 / Pi + Ctr->C2;

	double d_euler[3][3];
	d_euler[0][0] = 1;
	d_euler[0][1] = -cos(Rocket.Att[0]) * tan(Rocket.Att[2]);
	d_euler[0][2] = sin(Rocket.Att[0]) * tan(Rocket.Att[2]);
	d_euler[1][0] = 0;
	d_euler[1][1] = cos(Rocket.Att[0]) / cos(Rocket.Att[2]);
	d_euler[1][2] = -sin(Rocket.Att[0]) / cos(Rocket.Att[2]);
	d_euler[2][0] = 0;
	d_euler[2][1] = sin(Rocket.Att[0]);
	d_euler[2][2] = cos(Rocket.Att[0]);

	double euler_derivatives[3];
	m33v31(d_euler, Rocket.Angular_Vel, euler_derivatives);

	if (Rocket.Angular_Vel[0] * 180 / Pi > val1)
	{
		Ctr->T2_cmd = 0;
		Ctr->T3_cmd = 40 / 2;
		Ctr->T4_cmd = 0;
		Ctr->T5_cmd = 40 / 2;
	}
	else if (Rocket.Angular_Vel[0] * 180 / Pi < val2)
	{
		Ctr->T2_cmd = 40 / 2;
		Ctr->T3_cmd = 0;
		Ctr->T4_cmd = 40 / 2;
		Ctr->T5_cmd = 0;
	}
	else
	{
		Ctr->T2_cmd = 0;
		Ctr->T3_cmd = 0;
		Ctr->T4_cmd = 0;
		Ctr->T5_cmd = 0;
	}

	double kr = -2 * Ctr->zeta_Ctr * Ctr->wn_Ctr / M_delT;
	double kp = -pow(Ctr->wn_Ctr,2) / M_delT;

	double dp_cmd = kr * euler_derivatives[1] - (kp * (Guidance.Guidance_CMD[2] - Rocket.Att[1]));
	double dy_cmd = kr * euler_derivatives[2] - (kp * (Guidance.Guidance_CMD[3] - Rocket.Att[2]));

	Ctr->dp_cmd = dp_cmd * cos(Rocket.Att[0]) + dy_cmd * sin(Rocket.Att[0]);
	Ctr->dy_cmd = -dp_cmd * sin(Rocket.Att[0]) + dy_cmd * cos(Rocket.Att[0]);

}