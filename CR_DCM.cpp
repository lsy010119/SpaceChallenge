#include "include/CRocketsimulation.h"

void CRocketsimulation::DCM_N2B(struct st_Rocket Rocket, double res[][3])
{
	res[0][0] = pow(Rocket.Quaternion[0], 2) + pow(Rocket.Quaternion[1], 2) - pow(Rocket.Quaternion[2], 2) - pow(Rocket.Quaternion[3], 2);
	res[0][1] = 2 * (Rocket.Quaternion[0] * Rocket.Quaternion[3] + Rocket.Quaternion[1] * Rocket.Quaternion[2]);
	res[0][2] = 2 * (-Rocket.Quaternion[0] * Rocket.Quaternion[2] + Rocket.Quaternion[1] * Rocket.Quaternion[3]);

	res[1][0] = 2 * (-Rocket.Quaternion[0] * Rocket.Quaternion[3] + Rocket.Quaternion[1] * Rocket.Quaternion[2]);
	res[1][1] = pow(Rocket.Quaternion[0], 2) - pow(Rocket.Quaternion[1], 2) + pow(Rocket.Quaternion[2], 2) - pow(Rocket.Quaternion[3], 2);
	res[1][2] = 2 * (Rocket.Quaternion[0] * Rocket.Quaternion[1] + Rocket.Quaternion[2] * Rocket.Quaternion[3]);

	res[2][0] = 2 * (Rocket.Quaternion[0] * Rocket.Quaternion[2] + Rocket.Quaternion[1] * Rocket.Quaternion[3]);
	res[2][1] = 2 * (-Rocket.Quaternion[0] * Rocket.Quaternion[1] + Rocket.Quaternion[2] * Rocket.Quaternion[3]);
	res[2][2] = pow(Rocket.Quaternion[0], 2) - pow(Rocket.Quaternion[1], 2) - pow(Rocket.Quaternion[2], 2) + pow(Rocket.Quaternion[3], 2);

}

/* Att -> Quat */
void CRocketsimulation::Att2Quat(double* Attitude, double* Quaternion)
{
	CRocketsimulation Roketsimulation;
	double Roll_Angle = Attitude[0];
	double Pitch_Angle = Attitude[1];
	double Yaw_Angle = Attitude[2];

	double Qx[4] = { 0.0 };
	Qx[0] = cos(Roll_Angle / 2);
	Qx[1] = sin(Roll_Angle / 2);

	double Qy[4] = { 0.0 };
	Qy[0] = cos(Pitch_Angle / 2);
	Qy[2] = sin(Pitch_Angle / 2);

	double Qz[4] = { 0.0 };
	Qz[0] = cos(Yaw_Angle / 2);
	Qz[3] = sin(Yaw_Angle / 2);

	double Quat_norm = 0;
	double Quaternion1[4];
	double Quaternion2[4];

	Roketsimulation.quaternionmul(Qz, Qx, Quaternion1);
	Roketsimulation.quaternionmul(Qy, Quaternion1, Quaternion2);


	for (int i = 0; i < 4; i++)
	{
		Quat_norm += pow(Quaternion2[i], 2);
	}
	Quat_norm = sqrt(Quat_norm);

	for (int i = 0; i < 4; i++)
	{
		Quaternion[i] = Quaternion2[i] / Quat_norm;
	}
}

void CRocketsimulation::quaternionmul(double Input1[4], double Input2[4], double* Quaternion)
{
	double w0 = Input1[0]; double x0 = Input1[1]; double y0 = Input1[2]; double z0 = Input1[3];
	double w1 = Input2[0]; double x1 = Input2[1]; double y1 = Input2[2]; double z1 = Input2[3];

	double wr = (w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1);
	double xr = (w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1);
	double yr = (w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1);
	double zr = (w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1);

	Quaternion[0] = wr;// wr is scalar part, xr, yr, zr are vector part
	Quaternion[1] = xr;
	Quaternion[2] = yr;
	Quaternion[3] = zr;
}

/* Quat -> Att */
void CRocketsimulation::Quaternion2Att(struct st_Rocket Rocket, double res[3])
{
	double qin[4];
	qin[0] = Rocket.Quaternion[0];
	qin[1] = Rocket.Quaternion[1];
	qin[2] = Rocket.Quaternion[2];
	qin[3] = Rocket.Quaternion[3];

	double r11 = -2 * (qin[1] * qin[3] - qin[0] * qin[2]);
	double r12 = pow(qin[0], 2) + pow(qin[1], 2) - pow(qin[2], 2) - pow(qin[3], 2);
	double r21 = 2 * (qin[1] * qin[2] + qin[0] * qin[3]);
	double r31 = -2 * (qin[2] * qin[3] - qin[0] * qin[1]);
	double r32 = pow(qin[0], 2) - pow(qin[1], 2) + pow(qin[2], 2) - pow(qin[3], 2);

	res[0] = atan2(r11, r12);
	res[1] = asin(r21);
	res[2] = atan2(r31, r32);

}
