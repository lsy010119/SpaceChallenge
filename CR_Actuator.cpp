#include "include/CRocketsimulation.h"

void CRocketsimulation::Actuator_Module(struct st_Ctr Ctr, struct st_Actuator* Act, struct st_Force_Moment* F_M)
{

	Act->T2 = Ctr.T2_cmd;
	Act->T3 = Ctr.T3_cmd;
	Act->T4 = Ctr.T4_cmd;
	Act->T5 = Ctr.T5_cmd;


	if (Act->Mode == 0)
	{
		Act->dp = Ctr.dp_cmd;
		Act->dy = Ctr.dy_cmd;
	}
	else if (Act->Mode == 1)
	{
		double wn_act = Act->wn_act * 2 * Pi;
		double zeta_act = Act->zeta_act;

		Act->d_dp = Act->dp1;
		Act->d_dp1 = -(pow(wn_act, 2) * Act->dp) - (2 * wn_act * zeta_act * Act->d_dp) + pow(wn_act,2) * Ctr.dp_cmd;

		Act->d_dy = Act->dy1;
		Act->d_dy1 = -(pow(wn_act, 2) * Act->dy) - (2 * wn_act * zeta_act * Act->d_dy) + pow(wn_act, 2) * Ctr.dy_cmd;
	}
	else
	{
		Act->dp = Ctr.dp_cmd;
		Act->dy = Ctr.dy_cmd;
	}


	Saturation(&Act->dp, Act->TVC_saturation);
	Saturation(&Act->dy, Act->TVC_saturation);

}

