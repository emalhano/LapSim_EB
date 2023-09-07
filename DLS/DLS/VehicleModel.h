#pragma once
#include <ap.h>
#include "Definitions.h"

class VehicleModel
{
	private:
		double x[N_STATES];
		double dx[N_STATES];
		double u[N_CTRLS];
		double outputs[N_OUTPUTS];
		double mCar;
		double Izz;
		double fWeightDist;
		double fWheelbase;
		double vWCV[FR][2];
		double xWCP [FR];
		double alpha[FR];
		double kappa[FR];
		double FTyre[FR][3];
		double rTyre[FR];
		double MDemand;
		double Power;
		double Iyy_wheel;
		double Cd;
		double fFrontalArea;
		double fAirDens;
		double cPac[FR][n_pac];
		double fSpeed;
		double fSideSlip;
		double fDragForce;
		

	public:
		VehicleModel();
		VehicleModel(double x_input[N_STATES], double u_input[N_CTRLS]);
		void updateStatesAndControls(double x_input[N_STATES], double u_input[N_CTRLS]);
		void updateTyreForces();
		void updateWheelVelocities();
		void updateSlipAngles();
		void updateSlipRatios();
		void updateWheelTorque();
		void updateStateDerivatives();
		void runVehicleModel();
		void checkTyreModel(double Fz);

		double getState(int id);
		double getStateDerivative(int id);
		double getOutput(int id);
		double getControl(int id);
		
};

