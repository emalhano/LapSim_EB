#pragma once



class TrackModel
{

	private:
		double* afDistance = 0;
		double* afCurv = 0;
		double anIndexApex[100] = { 0.0 };
		int    nNumberOfApexes = 0;

	public:
		TrackModel();
		void findApex();
		void reorderTrack();

};