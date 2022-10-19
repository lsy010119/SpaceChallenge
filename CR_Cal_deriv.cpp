#include "include/CRocketsimulation.h"

void CRocketsimulation::Calc_deriv(struct st_Sim* Sim, struct st_Rocket Rocket, struct st_Force_Moment F_M, struct st_Actuator Act)
{
	double Inv_Inertia[3][3];
	double buffer1[3] = { 0.0 };
	double buffer2[3] = { 0.0 };
	double buffer3[3] = { 0.0 };

	m33v31(Rocket.MOI, Rocket.Angular_Vel, buffer1);
	CrossProduct(Rocket.Angular_Vel, buffer1, buffer2);
	int buffer3_idx;
	for (buffer3_idx = 0; buffer3_idx < 3; ++buffer3_idx)
	{
		buffer3[buffer3_idx] = F_M.Total_Moment[buffer3_idx] - buffer2[buffer3_idx];
	}
	m33_Inverse(Rocket.MOI, Inv_Inertia); // �������Ʈ �����
	m33v31(Inv_Inertia, buffer3, Rocket.Angular_acc); // �����ӵ� // d_w_IB_Body
	d_Quaternion(Rocket, Rocket.dQuaternion); // ���ʹϾ� ��ȭ��

	mtrp(Rocket.DCM_N2B, Rocket.DCM_B2N);
	m33v31(Rocket.DCM_B2N, Rocket.Vel_B, Rocket.Vel_N);

	Sim->X[0] = Rocket.Pos_N[0];
	Sim->X[1] = Rocket.Pos_N[1];
	Sim->X[2] = Rocket.Pos_N[2];
	Sim->X[3] = Rocket.Vel_B[0];
	Sim->X[4] = Rocket.Vel_B[1];
	Sim->X[5] = Rocket.Vel_B[2];
	Sim->X[6] = Rocket.Quaternion[0];
	Sim->X[7] = Rocket.Quaternion[1];
	Sim->X[8] = Rocket.Quaternion[2];
	Sim->X[9] = Rocket.Quaternion[3];
	Sim->X[10] = Rocket.Angular_Vel[0];
	Sim->X[11] = Rocket.Angular_Vel[1];
	Sim->X[12] = Rocket.Angular_Vel[2];
	Sim->X[13] = Act.dp;
	Sim->X[14] = Act.dp1;
	Sim->X[15] = Act.dy;
	Sim->X[16] = Act.dy1;

	Sim->dX[0] = Rocket.Vel_N[0];
	Sim->dX[1] = Rocket.Vel_N[1];
	Sim->dX[2] = Rocket.Vel_N[2];
	Sim->dX[3] = -(Rocket.Angular_Vel[1] * Rocket.Vel_B[2] - Rocket.Angular_Vel[2] * Rocket.Vel_B[1]) + F_M.Total_Force_B[0] / Rocket.Mass;
	Sim->dX[4] = -(Rocket.Angular_Vel[2] * Rocket.Vel_B[0] - Rocket.Angular_Vel[0] * Rocket.Vel_B[2]) + F_M.Total_Force_B[1] / Rocket.Mass;
	Sim->dX[5] = -(Rocket.Angular_Vel[0] * Rocket.Vel_B[1] - Rocket.Angular_Vel[1] * Rocket.Vel_B[0]) + F_M.Total_Force_B[2] / Rocket.Mass;
	Sim->dX[6] = Rocket.dQuaternion[0];
	Sim->dX[7] = Rocket.dQuaternion[1];
	Sim->dX[8] = Rocket.dQuaternion[2];
	Sim->dX[9] = Rocket.dQuaternion[3];
	Sim->dX[10] = Rocket.Angular_acc[0];
	Sim->dX[11] = Rocket.Angular_acc[1];
	Sim->dX[12] = Rocket.Angular_acc[2];
	Sim->dX[13] = Act.d_dp;
	Sim->dX[14] = Act.d_dp1;
	Sim->dX[15] = Act.d_dy;
	Sim->dX[16] = Act.d_dy1;

}

void CRocketsimulation::Update_state(struct st_Rocket* Rocket, struct st_Thrust* Thruster, struct st_Guidance* Guidance, struct st_Actuator* Act, struct st_Sim Sim)
{
	double Vel_B[3];
	double Temp_Att[3];

	DCM_N2B(*Rocket, Rocket->DCM_N2B);
	mtrp(Rocket->DCM_N2B, Rocket->DCM_B2N);

	Rocket->Pos_N[0] = Sim.X[0];
	Rocket->Pos_N[1] = Sim.X[1];
	Rocket->Pos_N[2] = Sim.X[2];
	Rocket->Vel_B[0] = Sim.X[3];
	Rocket->Vel_B[1] = Sim.X[4];
	Rocket->Vel_B[2] = Sim.X[5];
	Rocket->Quaternion[0] = Sim.X[6];
	Rocket->Quaternion[1] = Sim.X[7];
	Rocket->Quaternion[2] = Sim.X[8];
	Rocket->Quaternion[3] = Sim.X[9];
	Rocket->Angular_Vel[0] = Sim.X[10];
	Rocket->Angular_Vel[1] = Sim.X[11];
	Rocket->Angular_Vel[2] = Sim.X[12];
	Act->dp = Sim.X[13];
	Act->dp1 = Sim.X[14];
	Act->dy = Sim.X[15];
	Act->dy1 = Sim.X[16];

	Quaternion2Att(*Rocket, Temp_Att);
	Rocket->Att[0] = Temp_Att[2];
	Rocket->Att[1] = Temp_Att[0];
	Rocket->Att[2] = Temp_Att[1];

	Vel_B[0] = Rocket->Vel_B[0];
	Vel_B[1] = Rocket->Vel_B[1];
	Vel_B[2] = Rocket->Vel_B[2];
	m33v31(Rocket->DCM_B2N, Vel_B, Rocket->Vel_N);

	Cal_CG(Rocket, Thruster, Guidance, Act, Sim);

	Rocket->Moment_Arm[0] = Thruster->Lateral_gas_x - Rocket->CG[0];
	Rocket->Moment_Arm[1] = Thruster->Main_Engine_x - Rocket->CG[0];
	Rocket->Moment_Arm[2] = 1.5;
}

void CRocketsimulation::d_Quaternion(struct st_Rocket Rocket, double* res)
{
	double Angular_Vel[3];

	int Angular_Vel_idx;
	for (Angular_Vel_idx = 0; Angular_Vel_idx < 3; ++Angular_Vel_idx)
	{
		Angular_Vel[Angular_Vel_idx] = Rocket.Angular_Vel[Angular_Vel_idx];
	}
	res[0] = (-0.5) * ((Angular_Vel[0] * Rocket.Quaternion[1]) + (Angular_Vel[1] * Rocket.Quaternion[2]) + (Angular_Vel[2] * Rocket.Quaternion[3]));		/* Input1_dot = 1/2*(-P*Input2 - Q*q2 - R*q3) */
	res[1] = (0.5) * ((Angular_Vel[0] * Rocket.Quaternion[0]) - (Angular_Vel[1] * Rocket.Quaternion[3]) + (Angular_Vel[2] * Rocket.Quaternion[2]));		/* Input2_dot = 1/2*(P*Input1 - Q*q3 + R*q2) */
	res[2] = (0.5) * ((Angular_Vel[0] * Rocket.Quaternion[3]) + (Angular_Vel[1] * Rocket.Quaternion[0]) - (Angular_Vel[2] * Rocket.Quaternion[1]));		/* q2_dot = 1/2*(P*q3 + Q*Input1 - R*Input2) */
	res[3] = (0.5) * ((-1.0 * Angular_Vel[0] * Rocket.Quaternion[2]) + (Angular_Vel[1] * Rocket.Quaternion[1]) + (Angular_Vel[2] * Rocket.Quaternion[0]));	/* q3_dot = 1/2*(-P*q2 + Q*Input2 + R*Input1) */

}

void CRocketsimulation::Cal_CG(struct st_Rocket* Rocket, struct st_Thrust* Thruster, struct st_Guidance* Guidance, struct st_Actuator* Act, struct st_Sim Sim)
{
	double a, b, c;            // 3 ���������ǰ��
	double p, q;
	double D;
	double x1, x2, x3;  // ����
	double Oxidizer_x;
	double Oxidizer_y;
	double Oxidizer_z;
	double fuel_x;
	double fuel_y;
	double fuel_z;
	double Input[3];
	double r_solve[3];
	double r_exact = 0;

	if (Sim.t == 0)
	{
		Thruster->mo = Thruster->mo;
		Thruster->Volume_o = Thruster->mo / Thruster->rho_o;

		Thruster->mf = Thruster->mf;
		Thruster->Volume_f = Thruster->mf / Thruster->rho_f;

		Input[0] = 0;
		Input[1] = -3 * pow(Thruster->mo_r, 2);
		Input[2] = -2 * pow(Thruster->mo_r, 3) + 3 * Thruster->Volume_o / Pi;

		Cal_3rd_Equation(Input, r_solve);

		for (int count = 0; count < 3; ++count)
			if (-Thruster->mo_r <= r_solve[count] && Thruster->mo_r >= r_solve[count])
			{
				r_exact = r_solve[count];
				break;
			}

		Oxidizer_x = Thruster->Oxidizer_x_init - r_exact;
		Oxidizer_y = 0;
		Oxidizer_z = 0;

		Input[0] = 0;
		Input[1] = -3 * pow(Thruster->mf_r, 2);
		Input[2] = -2 * pow(Thruster->mf_r, 3) + 3 * Thruster->Volume_f / Pi;

		Cal_3rd_Equation(Input, r_solve);

		for (int count = 0; count < 3; ++count)
			if (-Thruster->mf_r <= r_solve[count] && Thruster->mf_r >= r_solve[count])
			{
				r_exact = r_solve[count];
				break;
			}

		fuel_x = Thruster->fuel_x_init - r_exact;
		fuel_y = 0;
		fuel_z = 0;

		Thruster->m_lg = Thruster->m_lg;
		Thruster->Lateral_gas_x = Thruster->Lateral_gas_x;

		Thruster->m_lg2 = Thruster->m_lg2;
		Thruster->Lateral_gas2_x = Thruster->Lateral_gas_x;
	}
	else
	{
		Thruster->mo = Thruster->mo - Guidance->Guidance_CMD[4] / 1000 * Thruster->mdot_ox_per_thrust * Sim.dt;
		Thruster->Volume_o = Thruster->mo / Thruster->rho_o;

		Thruster->mf = Thruster->mf - Guidance->Guidance_CMD[4] / 1000 * Thruster->mdot_fuel_per_thrust * Sim.dt;
		Thruster->Volume_f = Thruster->mf / Thruster->rho_f;

		Input[0] = 0;
		Input[1] = -3 * pow(Thruster->mo_r, 2);
		Input[2] = -2 * pow(Thruster->mo_r, 3) + 3 * Thruster->Volume_o / Pi;

		Cal_3rd_Equation(Input, r_solve);

		for (int count = 0; count < 3; ++count)
			if (-Thruster->mo_r <= r_solve[count] && Thruster->mo_r >= r_solve[count])
			{
				r_exact = r_solve[count];
				break;
			}

		Oxidizer_x = Thruster->Oxidizer_x_init - r_exact;
		Oxidizer_y = 0;
		Oxidizer_z = 0;

		Input[0] = 0;
		Input[1] = -3 * pow(Thruster->mf_r, 2);
		Input[2] = -2 * pow(Thruster->mf_r, 3) + 3 * Thruster->Volume_f / Pi;

		Cal_3rd_Equation(Input, r_solve);

		for (int count = 0; count < 3; ++count)
			if (-Thruster->mf_r <= r_solve[count] && Thruster->mf_r >= r_solve[count])
			{
				r_exact = r_solve[count];
				break;
			}

		fuel_x = Thruster->fuel_x_init - r_exact;
		fuel_y = 0;
		fuel_z = 0;

		Thruster->m_lg = Thruster->m_lg - (sqrt(pow(Act->T2, 2)) + sqrt(pow(Act->T3, 2))) / 1000 * Thruster->mdot_nitrogen_per_thrust * Sim.dt;
		Thruster->Lateral_gas_x = Thruster->Lateral_gas_x;

		Thruster->m_lg2 = Thruster->m_lg2 - (sqrt(pow(Act->T4, 2)) + sqrt(pow(Act->T5, 2))) / 1000 * Thruster->mdot_nitrogen_per_thrust * Sim.dt;
		Thruster->Lateral_gas2_x = Thruster->Lateral_gas_x;
	}

	double Total_Mass = Thruster->mo + Thruster->mf + Thruster->m_lg + Thruster->m_lg2 + Thruster->Main_Engine + Rocket->Electronic_System + Rocket->Structure;

	double Oxidizer_part_x = Thruster->mo * Oxidizer_x / Total_Mass;
	double fuel_part_x = Thruster->mf * fuel_x / Total_Mass;
	double lg1_part_x = Thruster->m_lg * Thruster->Lateral_gas_x / Total_Mass;
	double lg2_part_x = Thruster->m_lg2 * Thruster->Lateral_gas2_x / Total_Mass;
	double Main_engine_part_x = Thruster->Main_Engine * Thruster->Main_Engine_x / Total_Mass;
	double 	Electronic_system_part_x = Rocket->Electronic_System * Rocket->Electronic_System_x / Total_Mass;
	double structure_part_x = Rocket->Structure * Rocket->Structure_x / Total_Mass;

	double Oxidizer_part_y = Thruster->mo * Oxidizer_y / Total_Mass;
	double fuel_part_y = Thruster->mf * fuel_y / Total_Mass;
	double lg1_part_y = Thruster->m_lg * Thruster->Lateral_gas_y / Total_Mass;
	double lg2_part_y = Thruster->m_lg2 * Thruster->Lateral_gas2_y / Total_Mass;
	double 	Main_engine_part_y = Thruster->Main_Engine * Thruster->Main_Engine_y / Total_Mass;
	double Electronic_system_part_y = Rocket->Electronic_System * Rocket->Electronic_System_y / Total_Mass;
	double structure_part_y = Rocket->Structure * Rocket->Structure_y / Total_Mass;

	double Oxidizer_part_z = Thruster->mo * Oxidizer_z / Total_Mass;
	double fuel_part_z = Thruster->mf * fuel_z / Total_Mass;
	double lg1_part_z = Thruster->m_lg * Thruster->Lateral_gas_z / Total_Mass;
	double lg2_part_z = Thruster->m_lg2 * Thruster->Lateral_gas2_z / Total_Mass;
	double Main_engine_part_z = Thruster->Main_Engine * Thruster->Main_Engine_z / Total_Mass;
	double Electronic_system_part_z = Rocket->Electronic_System * Rocket->Electronic_System_z / Total_Mass;
	double structure_part_z = Rocket->Structure * Rocket->Structure_z / Total_Mass;

	Rocket->Mass = Total_Mass;
	Rocket->CG[0] = Oxidizer_part_x + fuel_part_x + lg1_part_x + lg2_part_x + Main_engine_part_x + Main_engine_part_x + structure_part_x + Electronic_system_part_x;
	Rocket->CG[1] = Oxidizer_part_y + fuel_part_y + lg1_part_y + lg2_part_y + Main_engine_part_y + Main_engine_part_y + structure_part_y + Electronic_system_part_y;
	Rocket->CG[2] = Oxidizer_part_z + fuel_part_z + lg1_part_z + lg2_part_z + Main_engine_part_z + Main_engine_part_z + structure_part_z + Electronic_system_part_z;

}

void CRocketsimulation::Cal_3rd_Equation(double Input[3], double res[3])
{

	double p, q;
	double D;
	double x1, x2, x3;  // ����

	p = (-Input[0] * Input[0] / 3.0 + Input[1]) / 3.0;
	q = (2.0 * Input[0] * Input[0] * Input[0] / 27.0 - Input[0] * Input[1] / 3.0 + Input[2]) / 2.0;
	D = p * p * p + q * q;

	if (D < 0)
		x1 = -2.0 * sqrt(-p) * cos(1.0 / 3.0 * atan(sqrt(-D) / q)) - Input[0] / 3.0;
	else if (D == 0)
		x1 = -2.0 * pow(q, 1.0 / 3.0) - Input[0] / 3.0;
	else
	{
		if (p < 0)
			x1 = -pow(q + sqrt(D), 1.0 / 3.0) - pow(q - sqrt(D), 1.0 / 3.0) - Input[0] / 3.0;
		else if (p == 0)
			x1 = -pow(2.0 * q, 1.0 / 3.0) - Input[0] / 3.0;
		else
			x1 = pow(-q + sqrt(D), 1.0 / 3.0) - pow(q + sqrt(D), 1.0 / 3.0) - Input[0] / 3.0;
	}

	Input[2] = (Input[0] + x1) * x1 + Input[1];
	Input[1] = Input[0] + x1;
	Input[0] = 1.0;
	D = Input[1] * Input[1] - 4.0 * Input[0] * Input[2];
	if (D > 0)
	{
		x2 = (-Input[1] - sqrt(D)) / (2.0 * Input[0]);
		x3 = (-Input[1] + sqrt(D)) / (2.0 * Input[0]);
	}
	else if (D == 0)
	{
		x2 = -Input[1] / (2.0 * Input[0]);
		printf("�ϳ��ǽǱ�%lf ���߱�%lf �Դϴ�.\n", x1, x2);
	}
	else
	{
		x2 = (-Input[1] / (2.0 * Input[0]));
		x3 = sqrt(abs(D)) / (2.0 * Input[0]);
		if (x2 == 0)
			printf("�ϳ��ǽǱ�%lf �͵����%lfi, %lfi �Դϴ�.\n", x1, x3, -x3);
		else
			printf("�ϳ��ǽǱ�%lf �͵����%lf%+lfi, %lf%+lfi �Դϴ�.\n", x1, x2, x3, x2, -x3);
	}

	res[0] = x1;
	res[1] = x2;
	res[2] = x3;


}