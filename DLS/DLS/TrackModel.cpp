#include <ap.h>
#include "Definitions.h"
#include "TrackModel.h"

TrackModel::TrackModel()
{

	// Initialise local variables
	int ii = 0;
	Json::Reader reader;
	Json::Value SetupValues;

	// Load setup
	std::ifstream SetupJson("C:\\Temp\\track.json");
	reader.parse(SetupJson, SetupValues);

	afDistance = new double[SetupValues["DistanceMap"].size()];
	afCurv     = new double[SetupValues["DistanceMap"].size()];

	for (ii = 0; ii < SetupValues["DistanceMap"].size(); ii++)
	{
		afDistance[ii] = SetupValues["DistanceMap"][ii].asDouble();
		afCurv[ii] = SetupValues["CurvatureMap"][ii].asDouble();
	}

	for (ii = 1; ii < ( SetupValues["DistanceMap"].size() - 1 ); ii++)
	{
		if ( abs(afCurv[ii]) > abs(afCurv[ii-1]) && abs(afCurv[ii]) > abs(afCurv[ii+1]) && abs(afCurv[ii]) > 1E-3 )
		{
			anIndexApex[nNumberOfApexes++] = ii;
		}
	}

}