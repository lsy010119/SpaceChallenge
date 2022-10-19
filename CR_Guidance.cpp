#include "include/CRocketsimulation.h"

void CRocketsimulation::Guidance_Module(struct st_Rocket* Rocket, struct st_Guidance* Guidance, struct st_Sim Sim)
{
	Calc_Guidancephase_and_target(Rocket, Guidance, Sim);
	Calc_ThrustCMD_and_Acc_cmd(Rocket, Guidance, Sim);
}

void CRocketsimulation::Calc_Guidancephase_and_target(struct st_Rocket* Rocket, struct st_Guidance* Guidance, struct st_Sim Sim)
{
	double d0 = 0;

	if (Sim.t >= 0 && Sim.t < Guidance->time1)
	{
		Guidance->Guidance_phase = 0;
	}
	else if (Sim.t >= Guidance->time1 && Sim.t < Guidance->time2 && Guidance->Guidance_phase == 0)
	{
		Guidance->Guidance_phase = 1;
	}
	else if (Sim.t >= Guidance->time2 && Sim.t < Guidance->time3 && Guidance->Guidance_phase == 1)
	{
		Guidance->Guidance_phase = 2;
	}
	else if (Sim.t >= Guidance->time3 && Sim.t < Guidance->time4 && Guidance->Guidance_phase == 2)
	{
		Guidance->Guidance_phase = 3;
	}
	else if (Sim.t >= Guidance->time4 && Guidance->Guidance_phase == 3)
	{
		Guidance->Guidance_phase = 4;
	}

	if (Guidance->Guidance_phase == 0)
	{
		Guidance->Target_position[0] = 0;
		Guidance->Target_position[1] = 0;
		Guidance->Target_position[2] = (Guidance->hovering_pos[2] - d0) / Guidance->ascent_time * Sim.t + d0;
		Guidance->Target_velocity[0] = 0;
		Guidance->Target_velocity[1] = 0;
		Guidance->Target_velocity[2] = (Guidance->hovering_pos[2] - d0) / Guidance->ascent_time;
	}
	else if (Guidance->Guidance_phase == 1)
	{
		Guidance->Target_position[0] = Guidance->c5_p1[0] * pow((Sim.t - Guidance->time1), 5) + Guidance->c4_p1[0] * pow((Sim.t - Guidance->time1), 4) + Guidance->c3_p1[0] * pow((Sim.t - Guidance->time1), 3);
		Guidance->Target_position[1] = Guidance->c5_p1[1] * pow((Sim.t - Guidance->time1), 5) + Guidance->c4_p1[1] * pow((Sim.t - Guidance->time1), 4) + Guidance->c3_p1[1] * pow((Sim.t - Guidance->time1), 3);
		Guidance->Target_position[2] = (Guidance->hovering_pos[2] - d0) / Guidance->ascent_time * Sim.t + d0;
		Guidance->Target_velocity[0] = 5 * Guidance->c5_p1[0] * pow((Sim.t - Guidance->time1), 4) + 4 * Guidance->c4_p1[0] * pow((Sim.t - Guidance->time1), 3) + 3 * Guidance->c3_p1[0] * pow((Sim.t - Guidance->time1), 2);
		Guidance->Target_velocity[1] = 5 * Guidance->c5_p1[1] * pow((Sim.t - Guidance->time1), 4) + 4 * Guidance->c4_p1[1] * pow((Sim.t - Guidance->time1), 3) + 3 * Guidance->c3_p1[1] * pow((Sim.t - Guidance->time1), 2);
		Guidance->Target_velocity[2] = (Guidance->hovering_pos[2] - d0) / Guidance->ascent_time;
	}
	else if (Guidance->Guidance_phase == 2)
	{
		Guidance->Target_position[0] = Guidance->hovering_pos[0];
		Guidance->Target_position[1] = Guidance->hovering_pos[1];
		Guidance->Target_position[2] = Guidance->hovering_pos[2];
		Guidance->Target_velocity[0] = 0;
		Guidance->Target_velocity[1] = 0;
		Guidance->Target_velocity[2] = 0;
	}
	else if (Guidance->Guidance_phase == 3)
	{
		Guidance->Target_position[0] = Guidance->c5xy_p3[0] * pow((Sim.t - Guidance->time3), 5) + Guidance->c4xy_p3[0] * pow((Sim.t - Guidance->time3), 4) + Guidance->c3xy_p3[0] * pow((Sim.t - Guidance->time3), 3) + Guidance->hovering_pos[0];
		Guidance->Target_position[1] = Guidance->c5xy_p3[1] * pow((Sim.t - Guidance->time3), 5) + Guidance->c4xy_p3[1] * pow((Sim.t - Guidance->time3), 4) + Guidance->c3xy_p3[1] * pow((Sim.t - Guidance->time3), 3) + Guidance->hovering_pos[1];
		Guidance->Target_position[2] = Guidance->c5z_p3 * pow((Sim.t - Guidance->time3), 5) + Guidance->c4z_p3 * pow((Sim.t - Guidance->time3), 4) + Guidance->c3z_p3 * pow((Sim.t - Guidance->time3), 3) + Guidance->hovering_pos[2];
		Guidance->Target_velocity[0] = 5 * Guidance->c5xy_p3[0] * pow((Sim.t - Guidance->time3), 4) + 4 * Guidance->c4xy_p3[0] * pow((Sim.t - Guidance->time3), 3) + 3 * Guidance->c3xy_p3[0] * pow((Sim.t - Guidance->time3), 2);
		Guidance->Target_velocity[1] = 5 * Guidance->c5xy_p3[1] * pow((Sim.t - Guidance->time3), 4) + 4 * Guidance->c4xy_p3[1] * pow((Sim.t - Guidance->time3), 3) + 3 * Guidance->c3xy_p3[1] * pow((Sim.t - Guidance->time3), 2);
		Guidance->Target_velocity[2] = 5 * Guidance->c5z_p3 * pow((Sim.t - Guidance->time3), 4) + 4 * Guidance->c4z_p3 * pow((Sim.t - Guidance->time3), 3) + 3 * Guidance->c3z_p3 * pow((Sim.t - Guidance->time3), 2);
	}
	else if (Guidance->Guidance_phase == 4)
	{
		Guidance->Target_position[0] = Guidance->hovering_pos[0] + Guidance->divert_dist[0];
		Guidance->Target_position[1] = Guidance->hovering_pos[1] + Guidance->divert_dist[1];
		Guidance->Target_position[2] = Guidance->end_vel * Sim.t + Guidance->hovering_pos[2] + Guidance->d_alt - Guidance->end_vel * Guidance->time4;
		Guidance->Target_velocity[0] = 0;
		Guidance->Target_velocity[1] = 0;
		Guidance->Target_velocity[2] = Guidance->end_vel;
	}
}

void CRocketsimulation::Calc_ThrustCMD_and_Acc_cmd(struct st_Rocket* Rocket, struct st_Guidance* Guidance, struct st_Sim Sim)
{
	double A_cmd[3] = { 0.0 };
	double Desired_Thrust_temp[3];
	double max_thrust = 8300;
	double thrust_ratio = 0.7;
	double min_thrust = thrust_ratio * max_thrust;
	double traj_w = 1;
	double traj_z = 1;
	double traj_Kp = pow(traj_w, 2);
	double traj_Kv = 2 * traj_z * traj_w;
	double Guidance_CMD[4];

	for (int A_cmd_idx = 0; A_cmd_idx < 3; ++A_cmd_idx)
	{
		A_cmd[A_cmd_idx] = traj_Kp * (Guidance->Target_position[A_cmd_idx] - Rocket->Pos_N[A_cmd_idx]) - traj_Kv * Rocket->Vel_N[A_cmd_idx];
	}
	A_cmd[2] = A_cmd[2] - 9.81;
	double A_cmd_mag = sqrt(pow(A_cmd[0], 2) + pow(A_cmd[1], 2) + pow(A_cmd[2], 2)); // �߷� ���� ũ��

	for (int Desired_Thrust_temp_idx = 0; Desired_Thrust_temp_idx < 3; ++Desired_Thrust_temp_idx)
	{
		Desired_Thrust_temp[Desired_Thrust_temp_idx] = A_cmd[Desired_Thrust_temp_idx] * Rocket->Mass;
	}
	double Desired_Thrust = sqrt(pow(Desired_Thrust_temp[0], 2) + pow(Desired_Thrust_temp[1], 2) + pow(Desired_Thrust_temp[2], 2)); // �߷� ���� ũ��
	double Desired_Thrust_temp2 = Desired_Thrust; // �߷� ���� ũ�� ���� ����

	// Saturation
	if (! Sim.t == 0)
	{
		if (Desired_Thrust > Guidance->Pre_Thrust + 10)
		{
			Desired_Thrust = Guidance->Pre_Thrust + 10;
		}
		else if (Desired_Thrust < Guidance->Pre_Thrust - 10)
		{
			Desired_Thrust = Guidance->Pre_Thrust - 10;
		}
		if (Desired_Thrust > max_thrust)
		{
			Desired_Thrust = max_thrust;
		}
		else if (Desired_Thrust < min_thrust)
		{
			Desired_Thrust = min_thrust;
		}
	}

	if (Desired_Thrust_temp2 >= Desired_Thrust)
	{
		for (int A_cmd_saturation_idx = 0; A_cmd_saturation_idx < 3; ++A_cmd_saturation_idx)
		{
			A_cmd[A_cmd_saturation_idx] = Desired_Thrust / Rocket->Mass * A_cmd[A_cmd_saturation_idx] / A_cmd_mag;
		}
	}

	Guidance_CMD[0] = 90 * Pi/180;
	Guidance_CMD[1] = atan2(-A_cmd[2], A_cmd[0]);
	Guidance_CMD[2] = asin(A_cmd[1] * Rocket->Mass / Desired_Thrust);
	Guidance_CMD[3] = Desired_Thrust;

	Guidance->Pre_Thrust = Guidance_CMD[3];
	Guidance->Guidance_CMD[0] = Guidance->Guidance_phase; // ���� ������
	Guidance->Guidance_CMD[1] = Guidance_CMD[0]; // �� �ڼ��� ����
	Guidance->Guidance_CMD[2] = Guidance_CMD[1]; // ��ġ �ڼ��� ����
	Guidance->Guidance_CMD[3] = Guidance_CMD[2]; // �� �ڼ��� ����
	Guidance->Guidance_CMD[4] = Guidance_CMD[3]; // �߷� ����
}