#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include "linalg.h"
#include "json\json.h"


#define PI		3.14159265358979323846
#define GRAV	9.81


// States
enum
{
	IDX_States_v_VeVel_X,
	IDX_States_v_VeVel_Y,
	IDX_States_w_VeAngVel_Z,
	IDX_States_w_FrontWheelAngVel,
	IDX_States_w_RearWheelAngVel,
	N_STATES
};

// Controls
enum
{
	IDX_Controls_a_Steering,
	IDX_Controls_a_TorqueDemand,
	N_CTRLS
};

// Outputs
enum
{
	IDX_Output_v_AbsVel,
	IDX_Output_v_dVeVel_X,
	IDX_Output_v_dVeVel_Y,
	IDX_Output_w_dVeAngVel_Z,
	IDX_Output_w_dFrontWheelAngVel,
	IDX_Output_w_dRearWheelAngVel,
	IDX_Output_v_AbsdVxdt,
	IDX_Output_v_AbsdVydt,
	IDX_Output_AlphaFront,
	IDX_Output_AlphaRear,
	IDX_Output_TyreGradientFront,
	IDX_Output_TyreGradientRear,
	N_OUTPUTS
};

// Axles
enum
{
	FRONT,
	REAR,
	FR
};

// Axes
enum
{
	X,
	Y,
	Z,
	XX,
	YY,
	ZZ
};

// Pacejka coefficients
enum
{
	pac_a0,
	pac_a1,
	pac_a2,
	pac_a3,
	pac_a4,
	pac_a5,
	pac_a6,
	pac_a7,
	pac_a8,
	pac_a9,
	pac_a10,
	pac_a11,
	pac_a12,
	pac_a13,
	pac_a14,
	pac_a15,
	pac_a16,
	pac_a17,
	n_pac
};