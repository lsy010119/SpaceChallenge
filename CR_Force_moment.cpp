#include "include/CRocketsimulation.h"

void CRocketsimulation::Sum_Force_Moment(struct st_Force_Moment* F_M, struct st_Rocket Rocket)
{

	F_M->Gravity_I[0] = 0;
	F_M->Gravity_I[1] = 0;
	F_M->Gravity_I[2] = Rocket.Mass * 9.81;

	m33v31(Rocket.DCM_N2B, F_M->Gravity_I, F_M->Gravity_B);

	F_M->Total_Force_B[0] = F_M->Gravity_B[0] + F_M->Actuator_Force_Moment[0];
	F_M->Total_Force_B[1] = F_M->Gravity_B[1] + F_M->Actuator_Force_Moment[1];
	F_M->Total_Force_B[2] = F_M->Gravity_B[2] + F_M->Actuator_Force_Moment[2];

	F_M->Total_Moment[0] = F_M->Actuator_Force_Moment[3];
	F_M->Total_Moment[1] = F_M->Actuator_Force_Moment[4];
	F_M->Total_Moment[2] = F_M->Actuator_Force_Moment[5];

}

void CRocketsimulation::Cal_Actuator_force_moment(struct st_Rocket Rocket, struct st_Force_Moment* F_M, struct st_Actuator* Act, struct st_Guidance* Guidance, struct st_Sim Sim)
{

	if (Guidance->Guidance_phase == 0 && Sim.t < Guidance->Opnloop_time)
	{
		Act->T3 = 0;
		Act->T4 = 0;
		Act->T2 = 0;
		Act->T5 = 0;
		Act->dp = 0;
		Act->dy = 0;
		Guidance->Guidance_CMD[4] = 8300;
		Guidance->Pre_Thrust = Guidance->Guidance_CMD[4];
	}

	F_M->Actuator_Force_Moment[0] = Guidance->Guidance_CMD[4] * cos(Act->dp) * cos(Act->dy);
	F_M->Actuator_Force_Moment[1] = -Guidance->Guidance_CMD[4] * sin(Act->dy);
	F_M->Actuator_Force_Moment[2] = Guidance->Guidance_CMD[4] * sin(Act->dp) * cos(Act->dy) - Act->T2 + Act->T3 + Act->T4 - Act->T5;
	F_M->Actuator_Force_Moment[3] = Rocket.Moment_Arm[2] * (-Act->T3 + Act->T2 - Act->T5 + Act->T4);
	F_M->Actuator_Force_Moment[4] = Rocket.Moment_Arm[1] * F_M->Actuator_Force_Moment[2] + Rocket.Moment_Arm[0] * (Act->T3 - Act->T2 - Act->T5 + Act->T4);
	F_M->Actuator_Force_Moment[5] = Rocket.Moment_Arm[1] * -F_M->Actuator_Force_Moment[1];
}