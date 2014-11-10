
#include "omni_lib.h"
#include "tinythread.h"

#include <windows.h>
#include <string>
#include <stdio.h>
#include <conio.h>
#include <iostream>
#include <math.h>
#include "WmraTypes.h"


using namespace std;
using namespace tthread;
using namespace WMRA;


WMRA::Pose p;
vector<double> tranMatrix(16);

tthread::thread* t;
int count_ms;
HDSchedulerHandle gCallbackHandle;

/******************************************************************************
 * Scheduler callback for reading the robot position.  Called every 1ms 
 *****************************************************************************/
HDCallbackCode HDCALLBACK omniCallback(void *pUserData)
{
    hdBeginFrame(hdGetCurrentDevice());

	// Get the position of the device.
    hduVector3Dd position;
    hdGetDoublev(HD_CURRENT_POSITION, position);
	hduVector3Dd velocity;
    hdGetDoublev(HD_CURRENT_VELOCITY, velocity);
	HDdouble tran[16];
    hdGetDoublev(HD_CURRENT_TRANSFORM, tran);

	count_ms++;

	// you don't have to use the following variables, but they may be useful
	hduVector3Dd forceDirection(0,0,0);
	double forceMagnitude = 0;

	//**************************//
	//*** START EDITING HERE ***//
	//**************************//
	
	for(int i = 0; i < 16; i++)
		tranMatrix[i] = tran[i];

	//cout << position[0] << ", " << position[1] << ", " << position[2] << endl;
	//cout << velocity[0] << ", " << velocity[1] << ", " << velocity[2] << endl;

	forceDirection[0] = -position[0];
	forceDirection[1] = -position[1];
	forceDirection[2] = -position[2];
	forceMagnitude = 0.1;



	//**************************//
	//*** STOP EDITING HERE ***//
	//**************************//

	// limiting the force to protect the Omnis
	// DO NOT EDIT THE FOLLOWING
	//hduVector3Dd f = forceMagnitude * forceDirection;
	//if(forceDirection[0] > 2)
	//	forceDirection[0] = 2;
	//if(forceDirection[0] < -2)
	//	forceDirection[0] = -2;
	//if(forceDirection[1] > 2)
	//	forceDirection[1] = 2;
	//if(forceDirection[1] < -2)
	//	forceDirection[1] = -2;
	//forceDirection[2] = 0.0;

	hduVector3Dd f = forceMagnitude * forceDirection;
	if(f[0] > 2)
		f[0] = 2;
	if(f[0] < -2)
		f[0] = -2;
	if(f[1] > 2)
		f[1] = 2;
	if(f[1] < -2)
		f[1] = -2;
	if(f[2] > 2)
		f[2] = 2;
	if(f[2] < -2)
		f[2] = -2;
	
	
	if(position[0] >= 10 || position[0] <= -10)
		p.y = -position[0];
	else
		p.y = 0;

	if(position[1] >= 10 || position[1] <= -10)
		p.z = position[1];
	else
		p.z = 0;

	if(position[2] >= 10 || position[2] <= -10)
		p.x = -position[2];
	else
		p.x = 0;
	

	// calculate and command desired force
	
	hdSetDoublev(HD_CURRENT_FORCE, f);

	hdEndFrame(hdGetCurrentDevice());

    /* Check if an error occurred while attempting to render the force */
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        if (hduIsSchedulerError(&error))
		{
            return HD_CALLBACK_DONE;
        }
    }

    return HD_CALLBACK_CONTINUE;
}

omni::omni()
{
	gCallbackHandle = 0;
	omni::gain = 20.0;

	HDErrorInfo error;
	count_ms = 0;

    // Initialize the default haptic device.
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        _getch();
        //return -1;
    }

	printf("Omni Callback setup complete\n");
	
    /* to enable the motors call:  hdEnable(HD_FORCE_OUTPUT); */
	hdEnable(HD_FORCE_OUTPUT);

    /* Start the haptic rendering loop. */
    hdStartScheduler();
    if(HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        _getch();
    }

	// Schedule the frictionless plane callback, which will then run at 
    // servoloop rates and command forces if the user penetrates the plane.
    gCallbackHandle = hdScheduleAsynchronous(omniCallback, 0, HD_MAX_SCHEDULER_PRIORITY);


	//t = new thread(omniThread,this);
}

omni::~omni()
{
	    /* Cleanup by stopping the haptics loop, unscheduling the asynchronous
       callback, disabling the device. */
    hdStopScheduler();
    hdUnschedule(gCallbackHandle);
    hdDisableDevice(hHD);

}
void omni::omniThread(void *aArg)
{
	   
    // Application loop - schedule our call to the main callback.
    //HDSchedulerHandle gCallbackHandle = hdScheduleAsynchronous(
    //  omniCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    printf("Sphere example.\n");
    printf("Move the device around to feel a frictionless sphere\n\n");
    printf("Press any key to quit.\n\n");

    while (!_kbhit())
    {
        if (!hdWaitForCompletion(gCallbackHandle, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            getch();
            break;
        }
    }

	//while(1)
	//{
	//	cout << "BOOM" << endl;
	//}
}

WMRA::Pose omni::getDeltaPose()
{
//	p.x = p.x*gain;
//	p.y = p.y*gain;
//	p.z = p.z*gain;
	p.pitch = 0;//p.pitch*gain;
	p.roll = 0;//p.roll*gain;
	p.yaw = 0;//p.yaw*gain;

	return p;
}

vector<double> omni::getKinematicPose()
{
	return tranMatrix;
}
