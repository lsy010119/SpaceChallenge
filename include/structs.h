#pragma once
#include "base.h"

struct st_Rocket
{
	double Pos_N[3] = { 0.0 };
	double Vel_N[3] = { 0.0 };
	double Vel_B[3] = { 0.0 };
	double Acc_N[3] = { 0.0 };
	double Acc_B[3] = { 0.0 };
	double d_Vel_B[3] = { 0.0 };
	double Quaternion[4] = { 0.0 };
	double dQuaternion[4] = { 0.0 };
	double Att[3] = { 0.0 };
	double Angular_Vel[3] = { 0.0 };
	double Angular_acc[3] = { 0.0 };
	double Mass;
	double CG[3];
	double MOI[3][3];
	double Moment_Arm[3];
	double m_dot = 0;
	double DCM_N2B[3][3];
	double DCM_B2N[3][3];
	double Diameter = 2;
	double Length = 4.1;
	double Electronic_System = 68;
	double Electronic_System_x = 1.8;
	double Electronic_System_y = 0;
	double Electronic_System_z = 0;
	double Structure = 329.3;
	double Structure_x = 1.8;
	double Structure_y = 0;
	double Structure_z = 0;
};

struct st_Guidance
{
	double Moment_CMD[3];
	double Guidance_phase = 0;
	double Guidance_CMD[5];
	double Pre_Thrust = 8300;;

	double Opnloop_time;
	double vrtAsc_time;
	double ascent_time;
	double hover_time;
	double Descent_Time;

	double hovering_pos[3];
	double Target_position[3];
	double divert_dist[2];
	double d_alt;
	double end_vel;
	double Target_pos[3];
	double Target_velocity[3];

	double time1;
	double time2;
	double time3;
	double time4;
	double time5;

	double c5_p1[2];
	double c4_p1[2];
	double c3_p1[2];
	double c5_p3[2];
	double c4_p3[2];
	double c3_p3[2];

	double c5xy_p3[2];
	double c4xy_p3[2];
	double c3xy_p3[2];
	double c5z_p3;
	double c4z_p3;
	double c3z_p3;
};

struct st_Actuator
{
	double Mode = 0;
	double TVC_saturation = 10 * D2R;
	double dp =0;
	double dp1 = 0;
	double d_dp = 0;
	double d_dp1 = 0;
	double dy = 0;
	double dy1 = 0;
	double d_dy = 0;
	double d_dy1 = 0;
	double wn_act = 17;
	double zeta_act = 0.8;
	double T2 = 0;
	double T3 = 0;
	double T4 = 0;
	double T5 = 0;
};

struct st_Ctr
{
	double dy_cmd = 0;
	double dp_cmd = 0;
	double T2_cmd = 0;
	double T3_cmd = 0;
	double T4_cmd = 0;
	double T5_cmd = 0;

	double sigma1 = -3;
	double sigma2 = -3;
	double C1 = 5;
	double C2 = -5;
	double wn_Ctr = 4;
	double zeta_Ctr = 0.9;
};

struct st_Thrust
{
	// Oxidizer
	double mo = 194.7;
	double mdot_ox_per_thrust = 0.310843;
	double rho_o = 1155;
	double mo_r = 0.35;
	double Oxidizer_x_init = 1.1;
	double Oxidizer_y_init = 0;
	double Oxidizer_z_init = 0;
	double Volume_o;
	// Fuel
	double mf = 75.4;
	double mdot_fuel_per_thrust = 0.0915663;
	double rho_f = 442;
	double mf_r = 0.35;
	double fuel_x = 2;
	double fuel_x_init = 2;
	double fuel_y_init = 0;
	double fuel_z_init = 0;
	double Volume_f;
	// N gas 1
	double m_lg = 36.3;
	double Lateral_gas_x = 2;
	double Lateral_gas_lg_r = 0.15;
	double mout_lg = 1;
	double rho_lg = 1.16;
	double Lateral_gas_y = 0.7;
	double Lateral_gas_z = 0;
	// N gas 2
	double m_lg2 = 36.3;
	double Lateral_gas2_x = 2;
	double Lateral_gas_lg2_r = 0.15;
	double mout_lg2 = 2;
	double rho_lg2 = 1.16;
	double Lateral_gas2_y = -0.7;
	double Lateral_gas2_z = 0;
	double mdot_nitrogen_per_thrust = 0.40241;
	// Engine
	double Main_Engine = 35;
	double Main_Engine_x = 3.05;
	double Main_Engine_y = 0;
	double Main_Engine_z = 0;
};

struct st_Sim
{
	double t = 0;
	double dt = 0.01;
	double X[17] = { 0.0 };
	double dX[17] = { 0.0 };
};

struct st_Force_Moment
{
	double Total_Force_B[3];
	double Gravity_I[3];
	double Gravity_B[3];
	double Total_Moment[3];
	double Actuator_Force_Moment[6]; // �ű��
};