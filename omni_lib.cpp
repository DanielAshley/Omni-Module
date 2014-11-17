
#include "WmraTypes.h"
#include "omni_lib.h"
#include "tinythread.h"

/* Sensable's includes */
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include <windows.h>
#include <string>
#include <stdio.h>
#include <conio.h>
#include <iostream>
#include <math.h>


using namespace std;
using namespace tthread;
using namespace WMRA;

tthread::thread* t;
int count_ms;
HDSchedulerHandle gCallbackHandle;
WMRA::Pose p;
bool b1,b2;
vector<double> baseOrientation;

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

	count_ms++;

	// you don't have to use the following variables, but they may be useful
	hduVector3Dd forceDirection(0,0,0);
	double forceMagnitude = 0;

	//**************************//
	//*** START EDITING HERE ***//
	//**************************//
	

	//cout << position[0] << ", " << position[1] << ", " << position[2] << endl;
	//cout << velocity[0] << ", " << velocity[1] << ", " << velocity[2] << endl;

	forceDirection[0] = -position[0];
	forceDirection[1] = -position[1];
	forceDirection[2] = -position[2];
	forceMagnitude = 0.1;


	// Setting X,Y,Z pose positions
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
		
	static hduVector3Dd gimbal_angles;
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);		
		
	HDint nCurrentButtons, nLastButtons;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nCurrentButtons);
//    if ((nCurrentButtons & HD_DEVICE_BUTTON_1) != 0 && (nLastButtons & HD_DEVICE_BUTTON_1) == 0)
    if ((nCurrentButtons & HD_DEVICE_BUTTON_1) != 0)
    {
        /* Detected button 1 down */
		if(b1==false)
		{
			b1 = true;
			baseOrientation[0] = gimbal_angles[0];
			baseOrientation[1] = gimbal_angles[1];
			baseOrientation[2] = gimbal_angles[2];
		}
		// Setting Roll, Pitch, Yaw values
		p.pitch = baseOrientation[1] - gimbal_angles[1];
		p.roll = baseOrientation[2] - gimbal_angles[2];
		p.yaw = baseOrientation[0] - gimbal_angles[0];
		//cout << "Pitch: " << p.pitch << endl;

    }
	else
	{
        /* Detected button 1 up */
		b1 = false;
		baseOrientation[0] = 0;
		baseOrientation[1] = 0;
		baseOrientation[2] = 0;
		p.pitch = 0;
		p.roll = 0;
		p.yaw = 0;
		
		//cout << "PPPPPPitch: " << p.pitch << endl;

	}

	if ((nCurrentButtons & HD_DEVICE_BUTTON_2) != 0)
    {
        /* Detected button 2 down */
        b2 = true;
    }
	else
	{
        /* Detected button 2 up */
		b2 = false;
	}
	
	

	//**************************//
	//*** STOP EDITING HERE ***//
	//**************************//

	// limiting the force to protect the Omnis
	// DO NOT EDIT THE FOLLOWING
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
	baseOrientation.resize(3);
	this->initialized = false;
	b1 = false;
	b2 = false;
	gCallbackHandle = 0;

	HDErrorInfo error;
	count_ms = 0;

    // Initialize the default haptic device.
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {	
		this->initialized = false;
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        _getch();
        //return -1;
    }
	else
		this->initialized = true;
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

	this->initialized = true;
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

WMRA::Pose omni::getDeltaPose()
{
	return p;
}

bool omni::checkButton1()
{
	bool tgt = b1;
	return tgt;
}

bool omni::checkButton2()
{
	bool tgt = b2;
	return tgt;
}

int omni::isInitialized()
{
	return this->initialized;
}