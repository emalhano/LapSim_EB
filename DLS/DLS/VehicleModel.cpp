#include "VehicleModel.h"
#include "Definitions.h"
#include <ap.h>
#include <math.h>

VehicleModel::VehicleModel()
{
	
	// Initialise local variables
	int ii = 0;
	Json::Reader reader;
	Json::Value SetupValues;


	fSideSlip = 0.0;
	fSpeed = 0.0;
	fDragForce = 0.0;

	// Initialise state derivatives
	for (ii = 0; ii < N_STATES; ii++)	x[ii]       = 0.0;
	for (ii = 0; ii < N_CTRLS; ii++)	u[ii]       = 0.0;
	for (ii = 0; ii < N_STATES; ii++)	dx[ii]      = 0.0;
	for (ii = 0; ii < N_OUTPUTS; ii++)	outputs[ii] = 0.0;

	// Load setup
	std::ifstream SetupJson("C:\\Temp\\setup.json");
	reader.parse(SetupJson, SetupValues);


	// Initialise car inertia properties and dimension
	mCar = SetupValues["mCar"].asDouble();
	Izz = SetupValues["MOI_Izz"].asDouble();
	fWeightDist = SetupValues["rWeightDist"].asDouble();
	fWheelbase = SetupValues["lWheelbase"].asDouble();
	Iyy_wheel = SetupValues["MOI_Iyy_wheel"].asDouble();

	// Initialise wheel centre position
	xWCP[FRONT] = fWheelbase * (1 - fWeightDist);
	xWCP[REAR] = -fWheelbase * fWeightDist;

	// Initialise vehicle wheel centre velocities
	vWCV[FRONT][X] = 0; vWCV[FRONT][Y] = 0;
	vWCV[REAR][X] = 0; vWCV[REAR][Y] = 0;

	// Initialise tyre radius
	rTyre[FRONT] = SetupValues["lTyreRadiusF"].asDouble();
	rTyre[REAR] = SetupValues["lTyreRadiusF"].asDouble();

	// Initialise tyre forces
	FTyre[FRONT][X] = 0; FTyre[FRONT][Y] = 0; FTyre[FRONT][Z] = 0;
	FTyre[REAR][X] = 0; FTyre[REAR][Y] = 0; FTyre[REAR][Z] = 0;

	// Initialise tyre slip angle and slip ratio
	alpha[FRONT] = 0; alpha[REAR] = 0;
	kappa[FRONT] = 0; kappa[REAR] = 0;

	// Initialise torque demand
	MDemand = 0;

	// Initialise power
	Power = SetupValues["PEnginePower"].asDouble();

	Cd = SetupValues["Cd"].asDouble();
	fAirDens = SetupValues["AirDens"].asDouble();
	fFrontalArea = SetupValues["AFrontalArea"].asDouble();
	fDragForce = 0;

	// Initialise tyre model coefficients
	cPac[FRONT][pac_a0] = SetupValues["TFM_F"]["a0"].asDouble();
	cPac[FRONT][pac_a1] = SetupValues["TFM_F"]["a1"].asDouble();
	cPac[FRONT][pac_a2] = SetupValues["TFM_F"]["a2"].asDouble();
	cPac[FRONT][pac_a3] = SetupValues["TFM_F"]["a3"].asDouble();
	cPac[FRONT][pac_a4] = SetupValues["TFM_F"]["a4"].asDouble();
	cPac[FRONT][pac_a5] = SetupValues["TFM_F"]["a5"].asDouble();
	cPac[FRONT][pac_a6] = SetupValues["TFM_F"]["a6"].asDouble();
	cPac[FRONT][pac_a7] = SetupValues["TFM_F"]["a7"].asDouble();
	cPac[FRONT][pac_a8] = SetupValues["TFM_F"]["a8"].asDouble();
	cPac[FRONT][pac_a9] = SetupValues["TFM_F"]["a9"].asDouble();
	cPac[FRONT][pac_a10] = SetupValues["TFM_F"]["a10"].asDouble();
	cPac[FRONT][pac_a11] = SetupValues["TFM_F"]["a11"].asDouble();
	cPac[FRONT][pac_a12] = SetupValues["TFM_F"]["a12"].asDouble();
	cPac[FRONT][pac_a13] = SetupValues["TFM_F"]["a13"].asDouble();
	cPac[FRONT][pac_a14] = SetupValues["TFM_F"]["a14"].asDouble();
	cPac[FRONT][pac_a15] = SetupValues["TFM_F"]["a15"].asDouble();
	cPac[FRONT][pac_a16] = SetupValues["TFM_F"]["a16"].asDouble();
	cPac[FRONT][pac_a17] = SetupValues["TFM_F"]["a17"].asDouble();

	cPac[REAR][pac_a0] = SetupValues["TFM_R"]["a0"].asDouble();
	cPac[REAR][pac_a1] = SetupValues["TFM_R"]["a1"].asDouble();
	cPac[REAR][pac_a2] = SetupValues["TFM_R"]["a2"].asDouble();
	cPac[REAR][pac_a3] = SetupValues["TFM_R"]["a3"].asDouble();
	cPac[REAR][pac_a4] = SetupValues["TFM_R"]["a4"].asDouble();
	cPac[REAR][pac_a5] = SetupValues["TFM_R"]["a5"].asDouble();
	cPac[REAR][pac_a6] = SetupValues["TFM_R"]["a6"].asDouble();
	cPac[REAR][pac_a7] = SetupValues["TFM_R"]["a7"].asDouble();
	cPac[REAR][pac_a8] = SetupValues["TFM_R"]["a8"].asDouble();
	cPac[REAR][pac_a9] = SetupValues["TFM_R"]["a9"].asDouble();
	cPac[REAR][pac_a10] = SetupValues["TFM_R"]["a10"].asDouble();
	cPac[REAR][pac_a11] = SetupValues["TFM_R"]["a11"].asDouble();
	cPac[REAR][pac_a12] = SetupValues["TFM_R"]["a12"].asDouble();
	cPac[REAR][pac_a13] = SetupValues["TFM_R"]["a13"].asDouble();
	cPac[REAR][pac_a14] = SetupValues["TFM_R"]["a14"].asDouble();
	cPac[REAR][pac_a15] = SetupValues["TFM_R"]["a15"].asDouble();
	cPac[REAR][pac_a16] = SetupValues["TFM_R"]["a16"].asDouble();
	cPac[REAR][pac_a17] = SetupValues["TFM_R"]["a17"].asDouble();

};

VehicleModel::VehicleModel(double x_input[N_STATES], double u_input[N_CTRLS])
{

	int ii = 0;
	Json::Reader reader;
	Json::Value SetupValues;

	fSideSlip = 0.0;
	fSpeed = 0.0;
	fDragForce = 0.0;

	// Initialise state derivatives
	for (ii = 0; ii < N_STATES; ii++)	x[ii]       = x_input[ii];
	for (ii = 0; ii < N_CTRLS; ii++)	u[ii]       = u_input[ii];
	for (ii = 0; ii < N_STATES; ii++)	dx[ii]      = 0.0;
	for (ii = 0; ii < N_OUTPUTS; ii++)	outputs[ii] = 0.0;

	// Load setup
	std::ifstream SetupJson("C:\\Temp\\setup.json");
	reader.parse(SetupJson, SetupValues);


	// Initialise car inertia properties and dimension
	mCar = SetupValues["mCar"].asDouble(); //900
	Izz = SetupValues["MOI_Izz"].asDouble(); //1000
	fWeightDist = SetupValues["rWeightDist"].asDouble(); //0.48
	fWheelbase = SetupValues["lWheelbase"].asDouble(); //3.6
	Iyy_wheel = SetupValues["MOI_Iyy_wheel"].asDouble(); //1.5

	// Initialise wheel centre position
	xWCP[FRONT] = fWheelbase * (1 - fWeightDist);
	xWCP[REAR] = -fWheelbase * fWeightDist;

	// Initialise vehicle wheel centre velocities
	vWCV[FRONT][X] = 0; vWCV[FRONT][Y] = 0;
	vWCV[REAR][X] = 0; vWCV[REAR][Y] = 0;

	// Initialise tyre radius
	rTyre[FRONT] = SetupValues["lTyreRadiusF"].asDouble(); //0.3
	rTyre[REAR] = SetupValues["lTyreRadiusF"].asDouble(); //0.3

	// Initialise tyre forces
	FTyre[FRONT][X] = 0; FTyre[FRONT][Y] = 0; FTyre[FRONT][Z] = 0;
	FTyre[REAR][X] = 0; FTyre[REAR][Y] = 0; FTyre[REAR][Z] = 0;

	// Initialise tyre slip angle and slip ratio
	alpha[FRONT] = 0; alpha[REAR] = 0;
	kappa[FRONT] = 0; kappa[REAR] = 0;

	// Initialise torque demand
	MDemand = 0;

	// Initialise power
	Power = SetupValues["PEnginePower"].asDouble();//300e3

	Cd = SetupValues["Cd"].asDouble(); //0.6;
	fAirDens = SetupValues["AirDens"].asDouble(); //1.216
	fFrontalArea = SetupValues["AFrontalArea"].asDouble(); //1.5
	fDragForce = 0;

	// Initialise tyre model coefficients
	cPac[FRONT][pac_a0] = SetupValues["TFM_F"]["a0"].asDouble(); //1.4
	cPac[FRONT][pac_a1] = SetupValues["TFM_F"]["a1"].asDouble(); //0;
	cPac[FRONT][pac_a2] = SetupValues["TFM_F"]["a2"].asDouble(); //1.100;
	cPac[FRONT][pac_a3] = SetupValues["TFM_F"]["a3"].asDouble(); //2500 * 180 / PI;
	cPac[FRONT][pac_a4] = SetupValues["TFM_F"]["a4"].asDouble(); //10000;
	cPac[FRONT][pac_a5] = SetupValues["TFM_F"]["a5"].asDouble(); //0;
	cPac[FRONT][pac_a6] = SetupValues["TFM_F"]["a6"].asDouble(); //0;
	cPac[FRONT][pac_a7] = SetupValues["TFM_F"]["a7"].asDouble(); //-2;
	cPac[FRONT][pac_a8] = SetupValues["TFM_F"]["a8"].asDouble(); //0;
	cPac[FRONT][pac_a9] = SetupValues["TFM_F"]["a9"].asDouble(); //0;
	cPac[FRONT][pac_a10] = SetupValues["TFM_F"]["a10"].asDouble(); //0;
	cPac[FRONT][pac_a11] = SetupValues["TFM_F"]["a11"].asDouble(); //0;
	cPac[FRONT][pac_a12] = SetupValues["TFM_F"]["a12"].asDouble(); //0;
	cPac[FRONT][pac_a13] = SetupValues["TFM_F"]["a13"].asDouble(); //0;
	cPac[FRONT][pac_a14] = SetupValues["TFM_F"]["a14"].asDouble(); //0;
	cPac[FRONT][pac_a15] = SetupValues["TFM_F"]["a15"].asDouble(); //0;
	cPac[FRONT][pac_a16] = SetupValues["TFM_F"]["a16"].asDouble(); //0;
	cPac[FRONT][pac_a17] = SetupValues["TFM_F"]["a17"].asDouble(); //0;

	cPac[REAR][pac_a0] = SetupValues["TFM_R"]["a0"].asDouble(); //1.4;
	cPac[REAR][pac_a1] = SetupValues["TFM_R"]["a1"].asDouble(); //0;
	cPac[REAR][pac_a2] = SetupValues["TFM_R"]["a2"].asDouble(); //1.1;
	cPac[REAR][pac_a3] = SetupValues["TFM_R"]["a3"].asDouble(); //3000 * 180 / PI;
	cPac[REAR][pac_a4] = SetupValues["TFM_R"]["a4"].asDouble(); //10000;
	cPac[REAR][pac_a5] = SetupValues["TFM_R"]["a5"].asDouble(); //0;
	cPac[REAR][pac_a6] = SetupValues["TFM_R"]["a6"].asDouble(); //0;
	cPac[REAR][pac_a7] = SetupValues["TFM_R"]["a7"].asDouble(); //-2;
	cPac[REAR][pac_a8] = SetupValues["TFM_R"]["a8"].asDouble(); //0;
	cPac[REAR][pac_a9] = SetupValues["TFM_R"]["a9"].asDouble(); //0;
	cPac[REAR][pac_a10] = SetupValues["TFM_R"]["a10"].asDouble(); //0;
	cPac[REAR][pac_a11] = SetupValues["TFM_R"]["a11"].asDouble(); //0;
	cPac[REAR][pac_a12] = SetupValues["TFM_R"]["a12"].asDouble(); //0;
	cPac[REAR][pac_a13] = SetupValues["TFM_R"]["a13"].asDouble(); //0;
	cPac[REAR][pac_a14] = SetupValues["TFM_R"]["a14"].asDouble(); //0;
	cPac[REAR][pac_a15] = SetupValues["TFM_R"]["a15"].asDouble(); //0;
	cPac[REAR][pac_a16] = SetupValues["TFM_R"]["a16"].asDouble(); //0;
	cPac[REAR][pac_a17] = SetupValues["TFM_R"]["a17"].asDouble(); //0;

};

void VehicleModel::updateStatesAndControls(double x_input[N_STATES], double u_input[N_CTRLS])
{
	for (int ii = 0; ii < N_STATES; ii++) x[ii] = x_input[ii];
	for (int ii = 0; ii < N_CTRLS ; ii++) u[ii] = u_input[ii];
}

void VehicleModel::updateTyreForces()
{

	double fTotalSlip[FR] = { 0.0 };
	double fSlipDir[FR][2] = { 0.0 };
	double fSlipDirNorm = 0;
	double B = 0;
	double C = 0;
	double D = 0;
	double BCD = 0;
	double H = 0;
	double E = 0;
	double V = 0;
	double Bx1 = 0;
	double Bx1Increment = 0;
	double fTyreForceSlipDir = 0;
	double fTyreForceSlipDirIncrement = 0;
	double fHorizontalStiffness[FR] = { 0.0 };


	FTyre[FRONT][Z] = mCar * fWeightDist * GRAV;
	FTyre[REAR ][Z] = mCar * (1-fWeightDist) * GRAV;

	for (int ii = 0; ii < FR; ii++)
	{
		fTotalSlip[ii] = sqrt(pow(alpha[ii], 2) + pow(kappa[ii], 2));

		// Avoid division by 0
		if (fTotalSlip[ii] == 0)
		{
			fSlipDir[ii][X] = 0;
			fSlipDir[ii][Y] = -1;

			FTyre[ii][X] = 0.0;
			FTyre[ii][Y] = 0.0;
		}
		else
		{

			fSlipDir[ii][X] = -kappa[ii] / fTotalSlip[ii];
			fSlipDir[ii][Y] =  alpha[ii] / fTotalSlip[ii];
			

			// Using the magic formula
			C = cPac[ii][pac_a0];
			D = FTyre[ii][Z] * (cPac[ii][pac_a1] * FTyre[ii][Z] + cPac[ii][pac_a2]);
			BCD = cPac[ii][pac_a3] * sin(atan(FTyre[ii][Z] / cPac[ii][pac_a4]) * 2);
			B = BCD / (C * D);
			H = cPac[ii][pac_a8] * FTyre[ii][Z] + cPac[ii][pac_a9];
			E = (cPac[ii][pac_a6] * FTyre[ii][Z] + cPac[ii][pac_a7]) * (1 - cPac[ii][pac_a17]) * signbit(fTotalSlip[ii] + H);
			V = cPac[ii][pac_a11] * FTyre[ii][Z] + cPac[ii][pac_a12];
			Bx1 = B * (fTotalSlip[ii] + H);
			Bx1Increment = Bx1 + 1e-6;

			// Tyre force in slip direction
			fTyreForceSlipDir = -D * sin(C * atan(Bx1 - E * (Bx1 - atan(Bx1)))) + V;
			fTyreForceSlipDirIncrement = -D * sin(C * atan(Bx1Increment - E * (Bx1 - atan(Bx1Increment)))) + V;

			fHorizontalStiffness[ii] = (fTyreForceSlipDirIncrement - fTyreForceSlipDir) / 1e-6;

			FTyre[ii][X] = fTyreForceSlipDir * fSlipDir[ii][X];
			FTyre[ii][Y] = fTyreForceSlipDir * fSlipDir[ii][Y];

		}

	}

	outputs[IDX_Output_AlphaFront] = alpha[FRONT];
	outputs[IDX_Output_AlphaRear] = alpha[REAR];
	outputs[IDX_Output_TyreGradientFront] = fHorizontalStiffness[FRONT];
	outputs[IDX_Output_TyreGradientRear] = fHorizontalStiffness[REAR];

}

void VehicleModel::updateWheelVelocities()
{

	double vWCV_VeFrame[2] = { 0.0 };

	vWCV_VeFrame[X] = x[IDX_States_v_VeVel_X];
	vWCV_VeFrame[Y] = x[IDX_States_v_VeVel_Y] + xWCP[FRONT] * x[IDX_States_w_VeAngVel_Z];

	vWCV[FRONT][X] =  vWCV_VeFrame[X] * cos(u[IDX_Controls_a_Steering])
					- vWCV_VeFrame[Y] * sin(u[IDX_Controls_a_Steering]);

	vWCV[FRONT][Y] = - vWCV_VeFrame[X] * sin(u[IDX_Controls_a_Steering])
					 + vWCV_VeFrame[Y] * cos(u[IDX_Controls_a_Steering]);

	vWCV[REAR][X] = x[IDX_States_v_VeVel_X];
	vWCV[REAR][Y] = x[IDX_States_v_VeVel_Y] + xWCP[REAR] * x[IDX_States_w_VeAngVel_Z];

}

void VehicleModel::updateSlipAngles()
{
	alpha[FRONT] = atan(vWCV[FRONT][Y] / vWCV[FRONT][X]);
	alpha[REAR ] = atan(vWCV[REAR ][Y] / vWCV[REAR ][X]);
}

void VehicleModel::updateSlipRatios()
{
	kappa[FRONT] = x[IDX_States_w_FrontWheelAngVel] * rTyre[FRONT] / vWCV[FRONT][X] - 1;
	kappa[REAR ] = x[IDX_States_w_RearWheelAngVel ] * rTyre[REAR ] / vWCV[REAR ][X] - 1;
}

void VehicleModel::updateWheelTorque()
{
	MDemand = u[IDX_Controls_a_TorqueDemand] * 2 * Power / (x[IDX_States_w_FrontWheelAngVel] + x[IDX_States_w_FrontWheelAngVel]);
}

void VehicleModel::updateStateDerivatives()
{
	fDragForce = fAirDens * fFrontalArea * Cd * pow(x[IDX_States_v_VeVel_X], 2) / 2;
	
	dx[IDX_States_v_VeVel_X] =  (  FTyre[FRONT][X] * cos(u[IDX_Controls_a_Steering])
								 - FTyre[FRONT][Y] * sin(u[IDX_Controls_a_Steering])
								 + FTyre[REAR ][X]
								 - fDragForce ) / mCar + x[IDX_States_v_VeVel_Y] * x[IDX_States_w_VeAngVel_Z];

	dx[IDX_States_v_VeVel_Y] =  (  FTyre[FRONT][X] * sin(u[IDX_Controls_a_Steering])
							     + FTyre[FRONT][Y] * cos(u[IDX_Controls_a_Steering])
							     + FTyre[REAR][Y] ) / mCar - x[IDX_States_v_VeVel_X] * x[IDX_States_w_VeAngVel_Z];

	dx[IDX_States_w_FrontWheelAngVel] = (MDemand / 2 - FTyre[FRONT][X] * rTyre[FRONT]) / Iyy_wheel;
	dx[IDX_States_w_RearWheelAngVel]  = (MDemand / 2 - FTyre[REAR ][X] * rTyre[REAR ]) / Iyy_wheel;

	dx[IDX_States_w_VeAngVel_Z] = ( ( FTyre[FRONT][X] * sin(u[IDX_Controls_a_Steering])
								    + FTyre[FRONT][Y] * cos(u[IDX_Controls_a_Steering]) ) * xWCP[FRONT]
								    + FTyre[REAR][Y] * xWCP[REAR] ) / Izz;

	outputs[IDX_Output_v_dVeVel_X]			= dx[IDX_States_v_VeVel_X];
	outputs[IDX_Output_v_dVeVel_Y]			= dx[IDX_States_v_VeVel_Y];
	outputs[IDX_Output_w_dVeAngVel_Z]		= dx[IDX_States_w_VeAngVel_Z]; 
	outputs[IDX_Output_w_dFrontWheelAngVel] = dx[IDX_States_w_FrontWheelAngVel];
	outputs[IDX_Output_w_dRearWheelAngVel]  = dx[IDX_States_w_RearWheelAngVel];

	outputs[IDX_Output_v_AbsdVxdt] =    dx[IDX_States_v_VeVel_X] * cos(fSideSlip) + dx[IDX_States_v_VeVel_Y] * sin(fSideSlip);
	outputs[IDX_Output_v_AbsdVydt] =  - dx[IDX_States_v_VeVel_X] * sin(fSideSlip) + dx[IDX_States_v_VeVel_Y] * cos(fSideSlip);

};

void VehicleModel::runVehicleModel()
{

	fSpeed = sqrt(x[IDX_States_v_VeVel_X] * x[IDX_States_v_VeVel_X] + x[IDX_States_v_VeVel_Y] * x[IDX_States_v_VeVel_Y]);
	fSideSlip = atan(x[IDX_States_v_VeVel_Y] / x[IDX_States_v_VeVel_X]);

	updateWheelVelocities();
	updateSlipAngles();
	updateSlipRatios();
	updateTyreForces();
	updateWheelTorque();
	updateStateDerivatives();

	outputs[IDX_Output_v_AbsVel] = fSpeed;

}

double VehicleModel::getState(int id)
{
	return x[id];
}

double VehicleModel::getControl(int id)
{
	return u[id];
}

double VehicleModel::getStateDerivative(int id)
{
	return dx[id];
}

double VehicleModel::getOutput(int id)
{
	return outputs[id];
}

void VehicleModel::checkTyreModel(double Fz)
{

	double fTotalSlip[FR] = { 0.0 };
	double fSlipDir[FR][2] = { 0.0 };
	double fSlipDirNorm = 0;
	double B = 0;
	double C = 0;
	double D = 0;
	double BCD = 0;
	double H = 0;
	double E = 0;
	double V = 0;
	double Bx1 = 0;
	double Bx1Increment = 0;
	double fTyreForceSlipDir = 0;
	double fTyreForceSlipDirIncrement = 0;
	double fHorizontalStiffness[FR] = { 0.0 };

	std::ofstream tyrecheck("C:\\Temp\\tyrecheck.csv", std::ios::app);

	for (double localalpha = 0; localalpha < 30; localalpha += 0.1)
	{

		for (int ii = 0; ii < FR; ii++)
		{
			fTotalSlip[ii] = localalpha * PI / 180;


			// Using the magic formula
			C = cPac[ii][pac_a0];
			D = Fz * (cPac[ii][pac_a1] * Fz + cPac[ii][pac_a2]);
			BCD = cPac[ii][pac_a3] * sin(atan(Fz / cPac[ii][pac_a4]) * 2);
			B = BCD / (C * D);
			H = cPac[ii][pac_a8] * Fz + cPac[ii][pac_a9];
			E = (cPac[ii][pac_a6] * Fz + cPac[ii][pac_a7]) * (1 - cPac[ii][pac_a17]) * signbit(fTotalSlip[ii] + H);
			V = cPac[ii][pac_a11] * Fz + cPac[ii][pac_a12];
			Bx1 = B * (fTotalSlip[ii] + H);
			Bx1Increment = Bx1 + 1e-6;

			// Tyre force in slip direction
			fTyreForceSlipDir = -D * sin(C * atan(Bx1 - E * (Bx1 - atan(Bx1)))) + V;
			fTyreForceSlipDirIncrement = -D * sin(C * atan(Bx1Increment - E * (Bx1 - atan(Bx1Increment)))) + V;

			fHorizontalStiffness[ii] = (fTyreForceSlipDirIncrement - fTyreForceSlipDir) / 1e-6;



			tyrecheck << localalpha << "," << fTyreForceSlipDir << ",";


		}
		tyrecheck << std::endl;
		
	}
	tyrecheck.close();

}