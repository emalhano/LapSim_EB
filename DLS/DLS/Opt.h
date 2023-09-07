#pragma once
#include <ap.h>
#include "Definitions.h"
#include "VehicleModel.h"


void getJac_MaxSpeed(const alglib::real_1d_array& x, alglib::real_1d_array& fi, alglib::real_2d_array& jac, void* params);
void getJac_MaxgLong(const alglib::real_1d_array& x, alglib::real_1d_array& fi, alglib::real_2d_array& jac, void* params);

struct MaxSpeedCfg
{

	int nFreeStates = 4;
	int nFreeControls = 2;
	int nNLEq = 4;
	int nNLNeq = 0;

	double rCurv = 0.0;

	int nStatesMap[4] = { IDX_States_v_VeVel_X,
						  IDX_States_v_VeVel_Y,
						  IDX_States_w_FrontWheelAngVel,
						  IDX_States_w_RearWheelAngVel };

	int nControlsMap[2] = { IDX_Controls_a_Steering,
							IDX_Controls_a_TorqueDemand };

	// First one is the objective function
	int nFixedOutputsMap[5] = { IDX_Output_v_AbsVel,
								IDX_Output_v_AbsdVydt,
							    IDX_Output_w_dVeAngVel_Z,
							    IDX_Output_w_dFrontWheelAngVel,
							    IDX_Output_w_dRearWheelAngVel };

	VehicleModel VeModel;

	double fPertSize = 1E-6;

};



struct MaxgLongCfg
{

	int nFreeStates = 3;
	int nFreeControls = 2;
	int nNLEq = 4;
	int nNLNeq = 0;

	double fSpeed = 0.0;
	double nDirection = 0.0;
	double rCurv = 0.0;

	int nStatesMap[3] = { IDX_States_v_VeVel_Y,
						  IDX_States_w_FrontWheelAngVel,
						  IDX_States_w_RearWheelAngVel };

	int nControlsMap[2] = { IDX_Controls_a_Steering,
							IDX_Controls_a_TorqueDemand };

	// First one is the objective function
	int nFixedOutputsMap[5] = { IDX_Output_v_AbsdVxdt,
								IDX_Output_v_AbsdVydt,
								IDX_Output_w_dVeAngVel_Z,
								IDX_Output_w_dFrontWheelAngVel,
								IDX_Output_w_dRearWheelAngVel };

	VehicleModel VeModel;

	double fPertSize = 1E-6;

};