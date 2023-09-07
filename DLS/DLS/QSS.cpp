#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "optimization.h"
#include "Definitions.h"
#include "VehicleModel.h"
#include "QSS.h"


QSS_RampSpeed::QSS_RampSpeed()
{

}

void QSS_RampSpeed::Solve(VehicleModel& VeModel, double x[N_STATES], double u[N_CTRLS], double fSpeed, double fYawRate)
{

	// Initialising
	int nFreeStates = 3;
	int nFreeControls = 2;
	int nStatesMap[3] = { 0 };
	int nControlsMap[2] = { 0 };
	int nFixedDerivativesMap[5] = { 0 };
	double x_update[5] = { 0.0 };
	double dx_Ref[N_STATES] = { 0.0 };
	double fErr = 0;
	double fPert = 0;
	double Jac_debug[5][5] = { 0.0 };

	int ii = 0;
	int jj = 0;

	// Mapping
	ii = 0;
	nStatesMap[ii++] = IDX_States_v_VeVel_Y;
	nStatesMap[ii++] = IDX_States_w_FrontWheelAngVel;
	nStatesMap[ii++] = IDX_States_w_RearWheelAngVel;

	ii = 0;
	nControlsMap[ii++] = IDX_Controls_a_Steering;
	nControlsMap[ii++] = IDX_Controls_a_TorqueDemand;

	ii = 0;
	nFixedDerivativesMap[ii++] = IDX_States_v_VeVel_X;
	nFixedDerivativesMap[ii++] = IDX_States_v_VeVel_Y;
	nFixedDerivativesMap[ii++] = IDX_States_w_VeAngVel_Z;
	nFixedDerivativesMap[ii++] = IDX_States_w_FrontWheelAngVel;
	nFixedDerivativesMap[ii++] = IDX_States_w_RearWheelAngVel;


	x[IDX_States_w_VeAngVel_Z] = fYawRate;


	// Initialising jacobian matrix and target vector
	alglib::real_2d_array Jac;
	Jac.setlength(5, 5);

	alglib::real_1d_array b;
	b.setlength(5);


	// Run vehicle model with initial guess
	VeModel.updateStatesAndControls(x, u);
	VeModel.runVehicleModel();

	// Store initial state derivatives and calculate error
	fErr = 0;
	for (ii = 0; ii < nFreeStates + nFreeControls; ii++)
	{
		dx_Ref[nFixedDerivativesMap[ii]] = VeModel.getStateDerivative(nFixedDerivativesMap[ii]);
		b[ii] = VeModel.getStateDerivative(nFixedDerivativesMap[ii]);
		fErr += pow(VeModel.getStateDerivative(nFixedDerivativesMap[ii]), 2);
	}
	fErr = fErr / 2;


	// Start iterating if error doesn't meet tolerance
	while (fErr > 1E-6)
	{

		// Build jacobian for states
		for (ii = 0; ii < nFreeStates; ii++)
		{
			// Perturb state
			fPert = (1 + x[nStatesMap[ii]]) * 1E-5;
			x[nStatesMap[ii] ] += fPert;

			// Run vehicle model with new states
			VeModel.updateStatesAndControls(x, u);
			VeModel.runVehicleModel();

			// Store elements in jacobian
			for (jj = 0; jj < nFreeStates + nFreeControls; jj++)
			{
				double dxlocal = VeModel.getStateDerivative(nFixedDerivativesMap[jj]);
				Jac[jj][ii] = ( VeModel.getStateDerivative(nFixedDerivativesMap[jj]) - dx_Ref[nFixedDerivativesMap[jj]] ) / fPert;
				Jac_debug[jj][ii] = Jac[jj][ii];
			}

			// Revert back to the original states for this iteration
			x[nStatesMap[ii]] -= fPert;

		}


		// Build jacobian for controls
		for (ii = 0; ii < nFreeControls; ii++)
		{

			// Perturb state
			fPert = (1 + u[nControlsMap[ii]]) * 1E-5;
			u[nControlsMap[ii]] += fPert;

			// Run vehicle model with new states
			VeModel.updateStatesAndControls(x, u);
			VeModel.runVehicleModel();

			// Store elements in jacobian
			for (jj = 0; jj < nFreeStates + nFreeControls; jj++)
			{
				Jac[jj][nFreeStates+ii] = (VeModel.getStateDerivative(nFixedDerivativesMap[jj]) - dx_Ref[nFixedDerivativesMap[jj]]) / fPert;
				Jac_debug[jj][nFreeStates + ii] = Jac[jj][nFreeStates + ii];
			}

			// Revert back to the original states for this iteration
			u[nControlsMap[ii]] -= fPert;

		}


		alglib::matinvreport rep;
		alglib::ae_int_t info;
		alglib::rmatrixinverse(Jac, Jac.rows(), info, rep);

		// Solve linear system A*x = b (Jac*x_update = -dx_Ref)
		for (ii = 0; ii < Jac.rows(); ii++)
		{
			x_update[ii] = 0;
			for (jj = 0; jj < Jac.cols(); jj++)
			{
				x_update[ii] += Jac[ii][jj] * (-dx_Ref[jj]);
			}
		}
		
		// Update states
		for (ii = 0; ii < nFreeStates; ii++)
		{
			x[nStatesMap[ii]] += x_update[ii];
		}

		// Update controls
		for (ii = 0; ii < nFreeControls; ii++)
		{
			u[nControlsMap[ii]] += x_update[nFreeStates+ii];
		}

		VeModel.updateStatesAndControls(x, u);
		VeModel.runVehicleModel();

		fErr = 0;
		for (ii = 0; ii < nFreeStates + nFreeControls; ii++)
		{
			dx_Ref[nFixedDerivativesMap[ii]] = VeModel.getStateDerivative(nFixedDerivativesMap[ii]);
			b[ii] = VeModel.getStateDerivative(nFixedDerivativesMap[ii]);
			fErr += pow(VeModel.getStateDerivative(nFixedDerivativesMap[ii]), 2);
		}
		fErr = fErr / 2;

	}

}