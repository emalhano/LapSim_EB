#pragma once
#include <ap.h>
#include "Definitions.h"

class QSS_RampSpeed
{
	private:

	public:
		QSS_RampSpeed();
		void Solve(VehicleModel& VeModel, double x0[N_STATES], double u0[N_CTRLS], double fSpeed, double fYawRate);

};