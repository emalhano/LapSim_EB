#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "optimization.h"
#include "Definitions.h"
#include "VehicleModel.h"
#include "TrackModel.h"
#include "QSS.h"
#include "Opt.h"


using namespace alglib;


int test_main()
{

    //MaxSpeedCfg OptConfig;
    MaxgLongCfg OptConfig;
    QSS_RampSpeed RampSpeed;
    TrackModel MyTrack;

    OptConfig.rCurv = 0.0;// 1 / 60.0;
    OptConfig.nDirection = -1.0;
    OptConfig.fSpeed = 30.0;



    double x[N_STATES] = { 0.0 };
    double u[N_CTRLS] = { 0.0 };

    x[IDX_States_v_VeVel_X] = 30;// sqrt(5 / OptConfig.rCurv);
    x[IDX_States_v_VeVel_Y] = 0;
    x[IDX_States_w_VeAngVel_Z] = 0;
    x[IDX_States_w_FrontWheelAngVel] = x[IDX_States_v_VeVel_X] / 0.3;
    x[IDX_States_w_RearWheelAngVel] = x[IDX_States_v_VeVel_X] / 0.3;

    u[IDX_Controls_a_Steering] = atan(3.6 * OptConfig.rCurv);
    u[IDX_Controls_a_TorqueDemand] = 0.0;

    OptConfig.VeModel.updateStatesAndControls(x, u);
    OptConfig.VeModel.runVehicleModel();

    RampSpeed.Solve(OptConfig.VeModel, x, u, 30.0/*sqrt(5 / OptConfig.rCurv)*/, x[IDX_States_v_VeVel_X] * OptConfig.rCurv);


    alglib::real_1d_array x0;
    alglib::real_1d_array s;
    double epsx = 0.000001;
    alglib::ae_int_t maxits = 0;
    alglib::minnlcstate state;
    alglib::minnlcreport rep;
    alglib::real_1d_array x1;

    x0.setlength(OptConfig.nFreeStates + OptConfig.nFreeControls);
    s.setlength(OptConfig.nFreeStates + OptConfig.nFreeControls);

    for (int ii = 0; ii < OptConfig.nFreeStates; ii++)   x0[ii] = OptConfig.VeModel.getState(OptConfig.nStatesMap[ii]);
    for (int ii = 0; ii < OptConfig.nFreeControls; ii++) x0[OptConfig.nFreeStates + ii] = OptConfig.VeModel.getControl(OptConfig.nControlsMap[ii]);
    for (int ii = 0; ii < OptConfig.nFreeStates + OptConfig.nFreeControls; ii++) s[ii] = 1.0;


    minnlccreate(OptConfig.nFreeStates + OptConfig.nFreeControls, x0, state);
    minnlcsetcond(state, epsx, maxits);
    minnlcsetscale(state, s);
    minnlcsetstpmax(state, 10);

    minnlcsetalgoslp(state);

    minnlcsetnlc(state, OptConfig.nNLEq, OptConfig.nNLNeq);


    alglib::minnlcoptimize(state, getJac_MaxgLong, NULL, &OptConfig);
    alglib::minnlcresults(state, x1, rep);


    //x[IDX_States_v_VeVel_X] = x1[0];
    //x[IDX_States_v_VeVel_Y] = x1[1];
    //x[IDX_States_w_VeAngVel_Z] = OptConfig.rCurv * sqrt( x1[0]* x1[0] + x1[1]*x1[1] );
    //x[IDX_States_w_FrontWheelAngVel] = x1[2];
    //x[IDX_States_w_RearWheelAngVel] = x1[3];

    //u[IDX_Controls_a_Steering] = x1[4];
    //u[IDX_Controls_a_TorqueDemand] = x1[5];


    x[IDX_States_v_VeVel_Y] = x1[0];
    x[IDX_States_v_VeVel_X] = sqrt(OptConfig.fSpeed * OptConfig.fSpeed - x[IDX_States_v_VeVel_Y] * x[IDX_States_v_VeVel_Y]);
    x[IDX_States_w_VeAngVel_Z] = OptConfig.rCurv * sqrt(x1[0] * x1[0] + x1[1] * x1[1]);
    x[IDX_States_w_FrontWheelAngVel] = x1[1];
    x[IDX_States_w_RearWheelAngVel] = x1[2];

    u[IDX_Controls_a_Steering] = x1[3];
    u[IDX_Controls_a_TorqueDemand] = x1[4];


    OptConfig.VeModel.updateStatesAndControls(x, u);
    OptConfig.VeModel.runVehicleModel();

    for (int ii = 0; ii < OptConfig.nFreeStates; ii++)   x0[ii] = OptConfig.VeModel.getState(OptConfig.nStatesMap[ii]);
    for (int ii = 0; ii < OptConfig.nFreeControls; ii++) x0[OptConfig.nFreeStates + ii] = OptConfig.VeModel.getControl(OptConfig.nControlsMap[ii]);


    //minnlccreate(OptConfig.nFreeStates + OptConfig.nFreeControls, x0, state);
    //minnlcsetcond(state, epsx, maxits);
    //minnlcsetscale(state, s);
    //minnlcsetstpmax(state, 10);
    //
    //minnlcsetalgoslp(state);
    //
    //minnlcsetnlc(state, OptConfig.nNLEq, OptConfig.nNLNeq);
    //alglib::minnlcoptimize(state, getJac_MaxSpeed, NULL, &OptConfig);
    //
    //std::cout << "--- MaxgLat ---" << std::endl << std::endl;
    //
    //std::cout << "States:" << std::endl;
    //for (int ii = 0; ii < N_STATES; ii++)
    //{
    //    std::cout << myVehicleModel.getState(ii) << std::endl;
    //}
    //std::cout <<  std::endl;
    //
    //std::cout << "Controls:" << std::endl;
    //for (int ii = 0; ii < N_CTRLS; ii++)
    //{
    //    std::cout << myVehicleModel.getControl(ii) << std::endl;
    //}
    //std::cout << std::endl;
    //
    //std::cout << "State derivatives:" << std::endl;
    //for (int ii = 0; ii < N_STATES; ii++)
    //{
    //    std::cout << myVehicleModel.getStateDerivative(ii) << std::endl;
    //}
    //std::cout << std::endl;
    //std::cout << "--- End ---" << std::endl << std::endl;



    // Run MaxgLat


}