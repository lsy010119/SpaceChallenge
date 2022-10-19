#include "include/CRocketsimulation.h"

void CRocketsimulation::Initialize(struct st_Rocket* Rocket, struct st_Guidance* Guidance, struct st_Actuator* Act, struct st_Thrust* Thruster, struct st_Sim Sim)
{
	// �ʱⰪ
	Rocket->Pos_N[0] = 0;
	Rocket->Pos_N[1] = 0;
	Rocket->Pos_N[2] = 0;

	Rocket->Vel_N[0] = 0;
	Rocket->Vel_N[1] = 0;
	Rocket->Vel_N[2] = 0;

	DCM_N2B(*Rocket, Rocket->DCM_N2B);
	m33v31(Rocket->DCM_N2B, Rocket->Vel_N, Rocket->Vel_B);

	Rocket->Att[0] = 90 * D2R;
	Rocket->Att[1] = 90 * D2R;
	Rocket->Att[2] = 0;
	Att2Quat(Rocket->Att, Rocket->Quaternion);

	Rocket->Mass = 775;
	Rocket->MOI[0][0] = 1.4136928e+02;
	Rocket->MOI[1][1] = 5.2392295e+02;
	Rocket->MOI[2][2] = 5.9472133e+02;

	Cal_CG(Rocket, Thruster, Guidance, Act, Sim);

	Rocket->Moment_Arm[0] = Thruster->Lateral_gas_x - Rocket->CG[0];
	Rocket->Moment_Arm[1] = Thruster->Main_Engine_x - Rocket->CG[0];
	Rocket->Moment_Arm[2] = 1.5;

	Guidance->Opnloop_time = 0.5;
	Guidance->vrtAsc_time = 3 * 2;
	Guidance->ascent_time = 10;
	Guidance->hovering_pos[0] = 5 * 2;
	Guidance->hovering_pos[1] = 0 * 2;
	Guidance->hovering_pos[2] = -10 * 2;
	Guidance->hover_time = 5 * 2;
	Guidance->divert_dist[0] = 40 * 2;
	Guidance->divert_dist[1] = 0 * 2;
	Guidance->d_alt = 9 * 2;
	Guidance->end_vel = 0.05 * 2;
	Guidance->Descent_Time = 5 * 2;
	Guidance->Target_pos[0] = Guidance->hovering_pos[0] + Guidance->divert_dist[0];
	Guidance->Target_pos[1] = Guidance->hovering_pos[1] + Guidance->divert_dist[1];
	Guidance->Target_pos[2] = Guidance->hovering_pos[2] + Guidance->d_alt;
	Guidance->time1 = Guidance->vrtAsc_time;
	Guidance->time2 = Guidance->ascent_time;
	Guidance->time3 = Guidance->time2 + Guidance->hover_time;
	Guidance->time4 = Guidance->time3 + Guidance->Descent_Time;
	Guidance->time5 = Guidance->time4 - (Guidance->hovering_pos[2] - Guidance->d_alt) / Guidance->end_vel;

	Guidance->c5_p1[0] = 6 * Guidance->hovering_pos[0] / pow((Guidance->ascent_time - Guidance->vrtAsc_time), 5);
	Guidance->c5_p1[1] = 6 * Guidance->hovering_pos[1] / pow((Guidance->ascent_time - Guidance->vrtAsc_time), 5);
	Guidance->c4_p1[0] = -15 * Guidance->hovering_pos[0] / pow((Guidance->ascent_time - Guidance->vrtAsc_time), 4);
	Guidance->c4_p1[1] = -15 * Guidance->hovering_pos[1] / pow((Guidance->ascent_time - Guidance->vrtAsc_time), 4);
	Guidance->c3_p1[0] = 10 * Guidance->hovering_pos[0] / pow((Guidance->ascent_time - Guidance->vrtAsc_time), 3);
	Guidance->c3_p1[1] = 10 * Guidance->hovering_pos[1] / pow((Guidance->ascent_time - Guidance->vrtAsc_time), 3);

	Guidance->c5xy_p3[0] = 6 * Guidance->divert_dist[0] / pow(Guidance->Descent_Time, 5);
	Guidance->c5xy_p3[1] = 6 * Guidance->divert_dist[1] / pow(Guidance->Descent_Time, 5);
	Guidance->c4xy_p3[0] = -15 * Guidance->divert_dist[0] / pow(Guidance->Descent_Time, 4);
	Guidance->c4xy_p3[1] = -15 * Guidance->divert_dist[1] / pow(Guidance->Descent_Time, 4);
	Guidance->c3xy_p3[0] = 10 * Guidance->divert_dist[0] / pow(Guidance->Descent_Time, 3);
	Guidance->c3xy_p3[1] = 10 * Guidance->divert_dist[1] / pow(Guidance->Descent_Time, 3);

	Guidance->c5z_p3 = (6 * Guidance->d_alt - 3 * Guidance->end_vel * Guidance->Descent_Time) / pow(Guidance->Descent_Time, 5);
	Guidance->c4z_p3 = (-15 * Guidance->d_alt + 7 * Guidance->end_vel * Guidance->Descent_Time) / pow(Guidance->Descent_Time, 4);
	Guidance->c3z_p3 = (10 * Guidance->d_alt - 4 * Guidance->end_vel * Guidance->Descent_Time) / pow(Guidance->Descent_Time, 3);


}