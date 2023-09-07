#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "optimization.h"
#include "Definitions.h"
#include "VehicleModel.h"
#include "Opt.h"



// Jacobian calculation for Max speed for a given curvature
void getJac_MaxSpeed(const alglib::real_1d_array& x, alglib::real_1d_array& fi, alglib::real_2d_array& jac, void* params)
{

    // Initalise local variables
    MaxSpeedCfg *OptConfig = (MaxSpeedCfg*) params;

    double rCurv = OptConfig->rCurv;
    double fPertSize = OptConfig->fPertSize;

    double  fSpeed = 0.0;
    double  xasdouble[N_STATES] = { 0.0 };
    double  uasdouble[N_CTRLS] = { 0.0 };
    double  fPert = 0.0;
    double  fSideSlip = 0.0;
    double  fLongAcc = 0.0;
    double  fLatAcc = 0.0;
    double* dx_Ref = 0;

    int ii, jj = 0;

    // Need to move this to the object
    dx_Ref = new double[1 + OptConfig->nNLEq + OptConfig->nNLNeq];

    // Pass states and controls to x and u variables
    for (ii = 0; ii < OptConfig->nFreeStates; ii++)   xasdouble[OptConfig->nStatesMap[ii]]   = x[ii];
    for (ii = 0; ii < OptConfig->nFreeControls; ii++) uasdouble[OptConfig->nControlsMap[ii]] = x[OptConfig->nFreeStates + ii];

    // Update dependent states
    fSpeed = sqrt( xasdouble[IDX_States_v_VeVel_X] * xasdouble[IDX_States_v_VeVel_X] + xasdouble[IDX_States_v_VeVel_Y] * xasdouble[IDX_States_v_VeVel_Y] );
    xasdouble[IDX_States_w_VeAngVel_Z] = fSpeed * rCurv;

    // Update vehicle model and run it
    OptConfig->VeModel.updateStatesAndControls(xasdouble, uasdouble);
    OptConfig->VeModel.runVehicleModel();



    // Compute function vector (objective + non linear equality + non linear inequality)
    for (ii = 0; ii < (1 + OptConfig->nNLEq + OptConfig->nNLNeq); ii++)
    {

        fi[ii] = OptConfig->VeModel.getOutput(OptConfig->nFixedOutputsMap[ii]);

        if (ii == 0) // if objective, multiply by -1 (we are trying to maximise speed -> minimise -speed)
        {
            fi[ii] *= -1;
        }
        // if wheel acceleration, make sure its congruent with vehicle acceleration
        else if (OptConfig->nFixedOutputsMap[ii] == IDX_Output_w_dFrontWheelAngVel ||
            OptConfig->nFixedOutputsMap[ii] == IDX_Output_w_dRearWheelAngVel)
        {
            fi[ii] -= OptConfig->VeModel.getOutput(IDX_Output_v_AbsdVxdt) * 0.3;
        }
    }




    // Get outputs prior to taking numerical derivatives
    for (ii = 0; ii < (1 + OptConfig->nNLEq + OptConfig->nNLNeq); ii++)
    {
        dx_Ref[ii] = OptConfig->VeModel.getOutput( OptConfig->nFixedOutputsMap[ii] );
    }




    // Build jacobian for states - top line of the jacobian are the objective fuction partial derivatives
    for (ii = 0; ii < OptConfig->nFreeStates; ii++)
    {
        // Perturb state
        fPert = (1 + xasdouble[OptConfig->nStatesMap[ii]]) * fPertSize;
        xasdouble[OptConfig->nStatesMap[ii]] += fPert;

        // Run vehicle model with new states
        OptConfig->VeModel.updateStatesAndControls(xasdouble, uasdouble);
        OptConfig->VeModel.runVehicleModel();

        // Store elements in jacobian
        for (jj = 0; jj < (1 + OptConfig->nNLEq + OptConfig->nNLNeq); jj++)
        {
            jac[jj][ii] = (OptConfig->VeModel.getOutput(OptConfig->nFixedOutputsMap[jj]) - dx_Ref[jj]) / fPert;
            if (jj == 0) jac[jj][ii] *= -1; // We multiply the derivative of the speed as our objective function is -ve speed
        }

        // Revert back to the original states for this iteration
        xasdouble[OptConfig->nStatesMap[ii]] -= fPert;
    }



    // Build jacobian for controls - top line of the jacobian are the objective fuction partial derivatives
    for (ii = 0; ii < OptConfig->nFreeControls; ii++)
    {
        // Perturb state
        fPert = (1 + uasdouble[OptConfig->nControlsMap[ii]]) * 1E-5;
        uasdouble[OptConfig->nControlsMap[ii]] += fPert;

        // Run vehicle model with new states
        OptConfig->VeModel.updateStatesAndControls(xasdouble, uasdouble);
        OptConfig->VeModel.runVehicleModel();

        // Store elements in jacobian
        for (jj = 0; jj < (1 + OptConfig->nNLEq + OptConfig->nNLNeq); jj++)
        {
            jac[jj][OptConfig->nFreeStates + ii] = (OptConfig->VeModel.getOutput(OptConfig->nFixedOutputsMap[jj]) - dx_Ref[jj]) / fPert;
            if (jj == 0) jac[jj][OptConfig->nFreeStates + ii] *= -1; // We multiply the derivative of the speed as our objective function is -ve speed
        }

        // Revert back to the original control for this iteration
        uasdouble[OptConfig->nControlsMap[ii]] -= fPert;
    }

    /*
    for (ii = 0; ii < 5; ii++)
    {
        for (jj = 0; jj < 6; jj++)
        {
            std::cout << jac[ii][jj] << " ";
        }
        std::cout << std::endl;
    }
    //*/


    // Clean dx_Ref
    delete[] dx_Ref; dx_Ref = 0;
}





// Jacobian calculation for MaxgLong for a given curvature and speed
void getJac_MaxgLong(const alglib::real_1d_array& x, alglib::real_1d_array& fi, alglib::real_2d_array& jac, void* params)
{

    // Initalise local variables
    MaxgLongCfg *OptConfig = (MaxgLongCfg*) params;

    double rCurv = OptConfig->rCurv;
    double fPertSize = OptConfig->fPertSize;
    double nDirection = OptConfig->nDirection;
    double fSpeed = OptConfig->fSpeed;

    double  xasdouble[N_STATES] = { 0.0 };
    double  uasdouble[N_CTRLS] = { 0.0 };
    double  fPert = 0.0;
    double* dx_Ref = 0;

    int ii, jj = 0;

    // Need to move this to the object
    dx_Ref = new double[1 + OptConfig->nNLEq + OptConfig->nNLNeq];

    // Pass states and controls to x and u variables
    for (ii = 0; ii < OptConfig->nFreeStates; ii++)   xasdouble[OptConfig->nStatesMap[ii]]   = x[ii];
    for (ii = 0; ii < OptConfig->nFreeControls; ii++) uasdouble[OptConfig->nControlsMap[ii]] = x[OptConfig->nFreeStates + ii];

    // Update dependent states
    xasdouble[IDX_States_v_VeVel_X] = sqrt(  fSpeed * fSpeed - xasdouble[IDX_States_v_VeVel_Y] * xasdouble[IDX_States_v_VeVel_Y] );
    xasdouble[IDX_States_w_VeAngVel_Z] = fSpeed * rCurv;

    // Update vehicle model and run it
    OptConfig->VeModel.updateStatesAndControls(xasdouble, uasdouble);
    OptConfig->VeModel.runVehicleModel();



    // Compute function vector (objective + non linear equality + non linear inequality)
    for (ii = 0; ii < (1 + OptConfig->nNLEq + OptConfig->nNLNeq); ii++)
    {

        fi[ii] = OptConfig->VeModel.getOutput(OptConfig->nFixedOutputsMap[ii]);

        if (ii == 0) // if objective, multiply by -1 (we are trying to maximise gLong -> minimise -gLong)
        {
            fi[ii] *= -1 * nDirection;
        }
        // if wheel acceleration, make sure its congruent with vehicle acceleration
        else if (OptConfig->nFixedOutputsMap[ii] == IDX_Output_w_dFrontWheelAngVel ||
                    OptConfig->nFixedOutputsMap[ii] == IDX_Output_w_dRearWheelAngVel)
        {
            fi[ii] -= OptConfig->VeModel.getOutput(IDX_Output_v_AbsdVxdt) * 0.3;
        }
    }




    // Get outputs prior to taking numerical derivatives
    for (ii = 0; ii < (1 + OptConfig->nNLEq + OptConfig->nNLNeq); ii++)
    {
        dx_Ref[ii] = OptConfig->VeModel.getOutput( OptConfig->nFixedOutputsMap[ii] );
    }




    // Build jacobian for states - top line of the jacobian are the partial derivatives of the objective fuction 
    for (ii = 0; ii < OptConfig->nFreeStates; ii++)
    {
        // Perturb state
        fPert = (1 + xasdouble[OptConfig->nStatesMap[ii]]) * fPertSize;
        xasdouble[OptConfig->nStatesMap[ii]] += fPert;

        xasdouble[IDX_States_v_VeVel_X] = sqrt(  fSpeed * fSpeed - xasdouble[IDX_States_v_VeVel_Y] * xasdouble[IDX_States_v_VeVel_Y] );

        // Run vehicle model with new states
        OptConfig->VeModel.updateStatesAndControls(xasdouble, uasdouble);
        OptConfig->VeModel.runVehicleModel();

        // Store elements in jacobian
        for (jj = 0; jj < (1 + OptConfig->nNLEq + OptConfig->nNLNeq); jj++)
        {
            jac[jj][ii] = (OptConfig->VeModel.getOutput(OptConfig->nFixedOutputsMap[jj]) - dx_Ref[jj]) / fPert;
            if (jj == 0) jac[jj][ii] *= -nDirection;; // We multiply the derivative of gLong as our objective function is -ve gLong
        }

        // Revert back to the original states for this iteration
        xasdouble[OptConfig->nStatesMap[ii]] -= fPert;
        xasdouble[IDX_States_v_VeVel_X] = sqrt(  fSpeed * fSpeed - xasdouble[IDX_States_v_VeVel_Y] * xasdouble[IDX_States_v_VeVel_Y] );
    }



    // Build jacobian for controls - top line of the jacobian are the objective fuction partial derivatives
    for (ii = 0; ii < OptConfig->nFreeControls; ii++)
    {
        // Perturb state
        fPert = (1 + uasdouble[OptConfig->nControlsMap[ii]]) * 1E-5;
        uasdouble[OptConfig->nControlsMap[ii]] += fPert;

        // Run vehicle model with new states
        OptConfig->VeModel.updateStatesAndControls(xasdouble, uasdouble);
        OptConfig->VeModel.runVehicleModel();

        // Store elements in jacobian
        for (jj = 0; jj < (1 + OptConfig->nNLEq + OptConfig->nNLNeq); jj++)
        {
            jac[jj][OptConfig->nFreeStates + ii] = (OptConfig->VeModel.getOutput(OptConfig->nFixedOutputsMap[jj]) - dx_Ref[jj]) / fPert;
            if (jj == 0) jac[jj][OptConfig->nFreeStates + ii] *= -nDirection;; // We multiply the derivative of gLong as our objective function is -ve gLong
        }

        // Revert back to the original control for this iteration
        uasdouble[OptConfig->nControlsMap[ii]] -= fPert;
    }

    /*
    for (ii = 0; ii < 5; ii++)
    {
        for (jj = 0; jj < 6; jj++)
        {
            std::cout << jac[ii][jj] << " ";
        }
        std::cout << std::endl;
    }
    //*/


    // Clean dx_Ref
    delete[] dx_Ref; dx_Ref = 0;
}