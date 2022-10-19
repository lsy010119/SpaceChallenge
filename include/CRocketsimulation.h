#pragma once
#include "base.h"

class CRocketsimulation
{
private:

public:
	//Initialize
	void Initialize(struct st_Rocket* Rocket, struct st_Guidance* Guidance, struct st_Actuator* Act, struct st_Thrust* Thruster, struct st_Sim Sim);

	//Dynamic
	void Sum_Force_Moment(struct st_Force_Moment* F_M, struct st_Rocket Rocket);
	void d_Quaternion(struct st_Rocket Rocket, double* res);
	void Calc_deriv(struct st_Sim* Sim, struct st_Rocket Rocket, struct st_Force_Moment F_M, struct st_Actuator Act);
	void Update_state(struct st_Rocket* Rocket, struct st_Thrust* Thruster, struct st_Guidance* Guidance, struct st_Actuator* Act, struct st_Sim Sim);
	void Cal_CG(struct st_Rocket* Rocket, struct st_Thrust* Thruster, struct st_Guidance* Guidance, struct st_Actuator* Act, struct st_Sim Sim);
	void Cal_3rd_Equation(double Input[3], double res[3]);

	//Guidnace
	void Guidance_Module(struct st_Rocket* Rocket, struct st_Guidance* Guidance, struct st_Sim Sim);
	void Calc_Guidancephase_and_target(struct st_Rocket* Rocket, struct st_Guidance* Guidance, struct st_Sim Sim);
	void Calc_ThrustCMD_and_Acc_cmd(struct st_Rocket* Rocket, struct st_Guidance* Guidance, struct st_Sim Sim);

	// Conotroller
	void Controller_Module(struct st_Rocket Rocket, struct st_Guidance Guidance, struct st_Ctr* Ctr);

	// Actuator
	void Actuator_Module(struct st_Ctr Ctr, struct st_Actuator* Act, struct st_Force_Moment* F_M);
	void Cal_Actuator_force_moment(struct st_Rocket Rocket, struct st_Force_Moment* F_M, struct st_Actuator* Act, struct st_Guidance* Guidance, struct st_Sim Sim);

	//DCM
	void DCM_N2B(struct st_Rocket Rocket, double res[][3]);
	void Att2Quat(double* Attitude, double* Quaternion);
	void Quaternion2Att(struct st_Rocket Rocket, double res[3]);
	void quaternionmul(double Input1[4], double Input2[4], double* Quaternion);

	// Math
	void Runge4th(double* Time, double StepSize, double* X, double* dX);
	void CrossProduct(double* Vec1, double* Vec2, double* Result);
	void m33m33(double x[][3], double y[][3], double res[][3]);
	void mtrp(double x[][3], double res[][3]);
	void m33v31(double x[][3], double* y, double* res);
	void m33_Inverse(double rG[3][3], double Result[3][3]);
	void m44v41(double x[][4], double* y, double* res);
	void Saturation(double* A, double B);
};