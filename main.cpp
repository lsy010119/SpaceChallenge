#define _CRT_SECURE_NO_WARNINGS

#include "include/CRocketsimulation.h"

CRocketsimulation Rocketsimulation;
st_Rocket Rocket;
st_Thrust Thruster;
st_Guidance Guidance;
st_Sim Sim;
st_Actuator Act;
st_Force_Moment F_M;
st_Ctr Ctr;

int main()
{
	FILE* Result = fopen("Result_Cpp.txt", "w");
	Rocketsimulation.Initialize(&Rocket, &Guidance, &Act, &Thruster, Sim);
	printf("initialized");

	while (round(Rocket.Pos_N[2] * 1e4) / 1e4 <= 0)
	{
		printf("%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\n", Sim.t, Rocket.Pos_N[0], Rocket.Pos_N[1], Rocket.Pos_N[2], Rocket.Vel_N[0], Rocket.Vel_N[1], Rocket.Vel_N[2], Rocket.Att[0] * 180 / Pi, Rocket.Att[1] * 180 / Pi, Rocket.Att[2] * 180 / Pi, Guidance.Guidance_CMD[4], Act.dp*180/Pi, Act.dy*180/Pi);
		fprintf(Result, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\n", Sim.t, Rocket.Pos_N[0], Rocket.Pos_N[1], Rocket.Pos_N[2], Rocket.Vel_N[0], Rocket.Vel_N[1], Rocket.Vel_N[2], Rocket.Att[0] * 180 / Pi, Rocket.Att[1] * 180 / Pi, Rocket.Att[2] * 180 / Pi, Act.dp * 180 / Pi, Act.dy * 180 / Pi);
		Rocketsimulation.Guidance_Module(&Rocket, &Guidance, Sim);
		Rocketsimulation.Controller_Module(Rocket, Guidance, &Ctr);
		Rocketsimulation.Actuator_Module(Ctr, &Act, & F_M);
		Rocketsimulation.Cal_Actuator_force_moment(Rocket, &F_M, &Act, &Guidance, Sim);
		Rocketsimulation.Sum_Force_Moment(&F_M, Rocket);
		Rocketsimulation.Calc_deriv(&Sim, Rocket, F_M, Act);
		Rocketsimulation.Runge4th(&Sim.t, Sim.dt, Sim.X, Sim.dX);
		Rocketsimulation.Update_state(&Rocket, &Thruster, &Guidance, &Act, Sim);
	}
	fclose(Result);
}