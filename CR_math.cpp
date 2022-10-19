#include "include/CRocketsimulation.h"

//..3x3 Matrix X 3x3 Matrix
void CRocketsimulation::m33m33(double x[][3], double y[][3], double res[][3])
{
	for (unsigned int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			res[i][j] = x[i][0] * y[0][j] + x[i][1] * y[1][j] + x[i][2] * y[2][j];
		}
	}
}

//3x3 Matrix X 3x1 Vector
void CRocketsimulation::m33v31(double x[][3], double* y, double* res)
{
	res[0] = x[0][0] * y[0] + x[0][1] * y[1] + x[0][2] * y[2];
	res[1] = x[1][0] * y[0] + x[1][1] * y[1] + x[1][2] * y[2];
	res[2] = x[2][0] * y[0] + x[2][1] * y[1] + x[2][2] * y[2];
}

//3x3 Matrix inversion
void CRocketsimulation::m33_Inverse(double rG[3][3], double Result[3][3])
{
	double(*api)[3] = (double(*)[3])rG;
	double(*res)[3] = (double(*)[3])Result;
	double det_rG;
	det_rG = api[0][0] * api[1][1] * api[2][2] + api[0][1] * api[1][2] * api[2][0] + api[0][2] * api[1][0] * api[2][1]
		- api[0][0] * api[1][2] * api[2][1] - api[0][1] * api[1][0] * api[2][2] - api[0][2] * api[1][1] * api[2][0];
	// khs 131211 coding rule
	if (fabs(det_rG) < 1e-308)
	{
		det_rG = 1e-308;
	}
	else
	{
		// coding rule
	}
	res[0][0] = (api[1][1] * api[2][2] - api[1][2] * api[2][1]) / det_rG;
	res[0][1] = (api[2][1] * api[0][2] - api[2][2] * api[0][1]) / det_rG;
	res[0][2] = (api[0][1] * api[1][2] - api[0][2] * api[1][1]) / det_rG;
	res[1][0] = (api[1][2] * api[2][0] - api[1][0] * api[2][2]) / det_rG;
	res[1][1] = (api[2][2] * api[0][0] - api[2][0] * api[0][2]) / det_rG;
	res[1][2] = (api[0][2] * api[1][0] - api[0][0] * api[1][2]) / det_rG;
	res[2][0] = (api[1][0] * api[2][1] - api[1][1] * api[2][0]) / det_rG;
	res[2][1] = (api[2][0] * api[0][1] - api[0][0] * api[2][1]) / det_rG;
	res[2][2] = (api[0][0] * api[1][1] - api[0][1] * api[1][0]) / det_rG;
}

//4x4 Matrix X 4x1 Vector
void CRocketsimulation::m44v41(double x[][4], double* y, double* res)
{
	res[0] = x[0][0] * y[0] + x[0][1] * y[1] + x[0][2] * y[2] + x[0][3] * y[3];
	res[1] = x[1][0] * y[0] + x[1][1] * y[1] + x[1][2] * y[2] + x[1][3] * y[3];
	res[2] = x[2][0] * y[0] + x[2][1] * y[1] + x[2][2] * y[2] + x[2][3] * y[3];
	res[3] = x[3][0] * y[0] + x[3][1] * y[1] + x[3][2] * y[2] + x[3][3] * y[3];
}

//..3x3 Matrix Transpose
void CRocketsimulation::mtrp(double x[][3], double res[][3])
{
	for (unsigned int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			res[i][j] = x[j][i];
		}
	}
}

// ����
void CRocketsimulation::CrossProduct(double* Vec1, double* Vec2, double* Result)
{
	Result[0] = Vec1[1] * Vec2[2] - Vec1[2] * Vec2[1];
	Result[1] = -(Vec1[0] * Vec2[2] - Vec1[2] * Vec2[0]);
	Result[2] = Vec1[0] * Vec2[1] - Vec1[1] * Vec2[0];

}

// �����Ÿ
void CRocketsimulation::Runge4th(double* Time, double StepSize, double* X, double* dX)
{
	double X_old[17] = { 0., };
	double K1[17] = { 0., };
	double K2[17] = { 0., };
	double K3[17] = { 0., };
	double K4[17] = { 0., };

	for (unsigned int n_Loop1 = 0U; n_Loop1 < 4U; n_Loop1++)
	{
		if (n_Loop1 == 1U || n_Loop1 == 3U)
		{
			*Time = *Time + 0.5 * StepSize;
		}
		for (unsigned int n_Loop2 = 0U; n_Loop2 < 17; n_Loop2++)
		{
			switch (n_Loop1)
			{
			case 0U:
				X_old[n_Loop2] = X[n_Loop2];
				K1[n_Loop2] = dX[n_Loop2];
				X[n_Loop2] = X_old[n_Loop2] + StepSize * K1[n_Loop2] / 2;
				break;

			case 1U:
				K2[n_Loop2] = dX[n_Loop2];
				X[n_Loop2] = X_old[n_Loop2] + StepSize * K2[n_Loop2] / 2;
				break;

			case 2U:
				K3[n_Loop2] = dX[n_Loop2];
				X[n_Loop2] = X_old[n_Loop2] + StepSize * K3[n_Loop2];
				break;

			case 3U:
				K4[n_Loop2] = dX[n_Loop2];
				X[n_Loop2] = X_old[n_Loop2] + StepSize / 6 * (K1[n_Loop2] + 2 * K2[n_Loop2] + 2 * K3[n_Loop2] + K4[n_Loop2]);
				break;

			default:
				break;
			}
		}
	}
}

// saturatoin
void CRocketsimulation::Saturation(double* A, double B)
{
	int sign;

	if (abs(*A) >= B)
	{
		if (*A > 0)
		{
			sign = 1;
		}
		else
		{
			sign = -1;
		}

		*A = B * sign;
	}
}
