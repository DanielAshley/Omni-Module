#ifndef OMNI_LIB_H
#define OMNI_LIB_H

#include <vector>

/* Sensable's includes */
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include "WmraTypes.h"

class omni{
public:
	omni();
	~omni();
	WMRA::Pose getDeltaPose();
	vector<double> getKinematicPose();
	static void omniThread(void *aArg);
//
	//HDSchedulerHandle gCallbackHandle;
//	HDCallbackCode HDCALLBACK omniCallback(void *pUserData);
	HHD hHD;

private:
	double gain;
};

#endif;