/**
 ******************************************************************************
 *
 * @file       guidance.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      This module compared @ref PositionActuatl to @ref ActiveWaypoint 
 * and sets @ref AttitudeDesired.  It only does this when the FlightMode field
 * of @ref ManualControlCommand is Auto.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/**
 * Input object: ActiveWaypoint
 * Input object: PositionActual
 * Input object: ManualControlCommand
 * Output object: AttitudeDesired
 *
 * This module will periodically update the value of the AttitudeDesired object.
 *
 * The module executes in its own thread in this example.
 *
 * Modules have no API, all communication to other modules is done through UAVObjects.
 * However modules may use the API exposed by shared libraries.
 * See the OpenPilot wiki for more details.
 * http://www.openpilot.org/OpenPilot_Application_Architecture
 *
 */

#include "openpilot.h"
#include "guidance.h"
#include "guidancesettings.h"
#include "attitudeactual.h"
#include "attitudedesired.h"
#include "ratedesired.h"
#include "positiondesired.h"	// object that will be updated by the module
#include "positionactual.h"
#include "manualcontrolcommand.h"
#include "stabilizationsettings.h"
#include "systemsettings.h"
#include "velocitydesired.h"
#include "velocityactual.h"

// Private constants
#define STACK_SIZE configMINIMAL_STACK_SIZE
#define TASK_PRIORITY (tskIDLE_PRIORITY+1)
// Private types

// Private variables
static xTaskHandle guidanceTaskHandle;

// Private functions
static void guidanceTask(void *parameters);
static float bound(float val, float min, float max);

static void updateVtolDesiredVelocity();
static void updateVtolDesiredAttitude();
static void updatePlaneDesiredAttitude();

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t GuidanceInitialize()
{
	// Start main task
	xTaskCreate(guidanceTask, (signed char *)"Guidance", STACK_SIZE, NULL, TASK_PRIORITY, &guidanceTaskHandle);

	return 0;
}

static float northIntegral = 0;
static float northErrorLast = 0;
static float eastIntegral = 0;
static float eastErrorLast = 0;
static float downIntegral = 0;
static float downErrorLast = 0;
static float speedIntegral = 0;
static float headingIntegral = 0;

/**
 * Module thread, should not return.
 */
static void guidanceTask(void *parameters)
{
	SystemSettingsData systemSettings;
	GuidanceSettingsData guidanceSettings;
	ManualControlCommandData manualControl;

	portTickType lastSysTime;

	// Main task loop
	lastSysTime = xTaskGetTickCount();
	while (1) {
		ManualControlCommandGet(&manualControl);
		SystemSettingsGet(&systemSettings);
		GuidanceSettingsGet(&guidanceSettings);

		if ((manualControl.FlightMode == MANUALCONTROLCOMMAND_FLIGHTMODE_AUTO)) {
			updateVtolDesiredVelocity();
			switch (systemSettings.AirframeType) {
			case (SYSTEMSETTINGS_AIRFRAMETYPE_VTOL):
			case (SYSTEMSETTINGS_AIRFRAMETYPE_QUADP):
			case (SYSTEMSETTINGS_AIRFRAMETYPE_QUADX):
				updateVtolDesiredAttitude();
				break;
			default:
				updatePlaneDesiredAttitude();
			}
		} else {
			// Be cleaner and get rid of global variables
			northIntegral = 0;
			northErrorLast = 0;
			eastIntegral = 0;
			eastErrorLast = 0;
			downIntegral = 0;
			downErrorLast = 0;
			speedIntegral = 0;
			headingIntegral = 0;
		}			
		vTaskDelayUntil(&lastSysTime, guidanceSettings.VelUpdatePeriod / portTICK_RATE_MS);
	}
}

void updateVtolDesiredVelocity()
{
	GuidanceSettingsData guidanceSettings;
	PositionActualData positionActual;
	PositionDesiredData positionDesired;
	VelocityDesiredData velocityDesired;
	
	GuidanceSettingsGet(&guidanceSettings);
	PositionActualGet(&positionActual);
	PositionDesiredGet(&positionDesired);
	VelocityDesiredGet(&velocityDesired);
	
	// Note all distances in cm
	float dNorth = positionDesired.North - positionActual.North;
	float dEast = positionDesired.East - positionActual.East;
	float distance = sqrt(pow(dNorth, 2) + pow(dEast, 2));
	float heading = atan2f(dEast, dNorth);
	float groundspeed = bound(guidanceSettings.GroundVelocityP * distance, 
				  0, guidanceSettings.MaxGroundspeed);
	
	velocityDesired.North = groundspeed * cosf(heading);
	velocityDesired.East = groundspeed * sinf(heading);
	
	float dDown = positionDesired.Down - positionActual.Down;
	velocityDesired.Down = bound(guidanceSettings.VertVelocityP * dDown,
					 -guidanceSettings.MaxVerticalSpeed, 
					 guidanceSettings.MaxVerticalSpeed);
	
	VelocityDesiredSet(&velocityDesired);	
}

static void updatePlaneDesiredAttitude()
{
	static portTickType lastSysTime;
	portTickType thisSysTime = xTaskGetTickCount();;
	float dT;

	VelocityDesiredData velocityDesired;
	VelocityActualData velocityActual;
	AttitudeDesiredData attitudeDesired;
	RateDesiredData rateDesired;
	AttitudeActualData attitudeActual;
	GuidanceSettingsData guidanceSettings;
	StabilizationSettingsData stabSettings;
	SystemSettingsData systemSettings;

	// Check how long since last update
	if(thisSysTime > lastSysTime) // reuse dt in case of wraparound
		dT = (thisSysTime - lastSysTime) / portTICK_RATE_MS / 1000.0f;		
	lastSysTime = thisSysTime;
	
	SystemSettingsGet(&systemSettings);
	GuidanceSettingsGet(&guidanceSettings);
	
	VelocityActualGet(&velocityActual);
	VelocityDesiredGet(&velocityDesired);
	AttitudeDesiredGet(&attitudeDesired);
	RateDesiredGet(&rateDesired);
	VelocityDesiredGet(&velocityDesired);
	AttitudeActualGet(&attitudeActual);
	StabilizationSettingsGet(&stabSettings);
   
	#define RAD2DEG (180./M_PI)
	float headingDesired = atan2f(velocityDesired.East, velocityDesired.North) * RAD2DEG;
	float speedDesired = bound(sqrt(pow(velocityDesired.East, 2) + pow(velocityDesired.North, 2) + pow(velocityDesired.Down, 2)),
							guidanceSettings.MinAirspeed, guidanceSettings.MaxGroundspeed);
	float energyDesired =  pow((speedDesired/100),2) - (velocityDesired.Down/100) / guidanceSettings.VertVelocityP;
	float headingActual = atan2f(velocityActual.East, velocityActual.North) * RAD2DEG;
	float speedActual = sqrt(pow(velocityActual.East, 2) + pow(velocityActual.North, 2) + pow(velocityActual.Down, 2));
	float energyActual =  pow((speedActual/100),2);

	// desired roll angle is speed independent and proportional to heading error
	float headingError =  headingDesired - headingActual;
	if (headingError>180.) headingError-=360.;
	if (headingError<-180.) headingError+=360.;
	headingIntegral =	bound(headingIntegral + headingError * guidanceSettings.VelPIDUpdatePeriod, 
				  -guidanceSettings.MaxHeadingIntegral,
				  guidanceSettings.MaxHeadingIntegral);
	if (isnan(headingIntegral)) headingIntegral = 0;
	attitudeDesired.Roll = bound( 
				bound( guidanceSettings.HeadingP * headingError + headingIntegral * guidanceSettings.HeadingI,
				-stabSettings.RollMax, stabSettings.RollMax ),
			-80, 80 );
	
	// but desired yaw rate is dependent on both roll and speed (formula for "smooth" curve)
	#define RAD2DEG (180./M_PI)
	#define GEE (9.81*100.)
	if (speedActual>1) {
        rateDesired.Yaw = RAD2DEG * tanf(attitudeDesired.Roll / RAD2DEG) * GEE / speedActual;
    } else {
        rateDesired.Yaw = 0;
    }
    printf("heading: %f desired: %f error: %f roll: %f° yawrate: %f °/s\n",headingActual,headingDesired,headingError,attitudeDesired.Roll,rateDesired.Yaw);

	// pitch is dependent on speed alone - PI loop - note the minus sign when assigning pitch! (negative pitches increase speed):
	float speedError = speedDesired - speedActual;
	speedIntegral =	bound(speedIntegral + speedError * guidanceSettings.VelPIDUpdatePeriod, 
				  -guidanceSettings.MaxVelIntegral,
				  guidanceSettings.MaxVelIntegral);
	if (isnan(speedIntegral)) speedIntegral = 0;
	attitudeDesired.Pitch = - bound(speedError * guidanceSettings.VelP + speedIntegral * guidanceSettings.VelI, -stabSettings.PitchMax, stabSettings.PitchMax);
    printf(" speed: %f desired: %f error: %f pitch:%f°\n",speedActual,speedDesired,speedError,attitudeDesired.Pitch);
    printf(" integral is: %f term is %f, prop term is %f\n",speedIntegral,speedIntegral * guidanceSettings.VelI,speedError * guidanceSettings.VelP);

	// throttle is dependent on flight energy:
	float downError = energyDesired - energyActual;
	downIntegral =	bound(downIntegral + downError * guidanceSettings.VelPIDUpdatePeriod, 
				  -guidanceSettings.MaxThrottleIntegral,
				  guidanceSettings.MaxThrottleIntegral);
	if (isnan(downIntegral)) downIntegral = 0;
	downErrorLast = downError;
	attitudeDesired.Throttle = bound(0.5 + (downError * guidanceSettings.DownP + downIntegral * guidanceSettings.DownI),0, 1);
    printf(" energy: %f desired: %f error: %f throttle: %f\n",energyActual,energyDesired,downError,attitudeDesired.Throttle);
    printf(" integral is: %f term is %f, prop term is %f\n",downIntegral,downIntegral * guidanceSettings.DownI,downError * guidanceSettings.DownP);
	printf(" desired vertical velocity: %i\n",velocityDesired.Down);
	AttitudeDesiredSet(&attitudeDesired);
	RateDesiredSet(&rateDesired);
}

/**
 * Module thread, should not return.
 */
static void updateVtolDesiredAttitude()
{
	static portTickType lastSysTime;
	portTickType thisSysTime = xTaskGetTickCount();;
	float dT;

	VelocityDesiredData velocityDesired;
	VelocityActualData velocityActual;
	AttitudeDesiredData attitudeDesired;
	AttitudeActualData attitudeActual;
	GuidanceSettingsData guidanceSettings;
	StabilizationSettingsData stabSettings;
	SystemSettingsData systemSettings;

	float northError;
	float northDerivative;
	float northCommand;
	float eastError;
	float eastDerivative;
	float eastCommand;
	float downError;
	float downDerivative;
	
	// Check how long since last update
	if(thisSysTime > lastSysTime) // reuse dt in case of wraparound
		dT = (thisSysTime - lastSysTime) / portTICK_RATE_MS / 1000.0f;		
	lastSysTime = thisSysTime;
	
	SystemSettingsGet(&systemSettings);
	GuidanceSettingsGet(&guidanceSettings);
	
	VelocityActualGet(&velocityActual);
	VelocityDesiredGet(&velocityDesired);
	AttitudeDesiredGet(&attitudeDesired);
	VelocityDesiredGet(&velocityDesired);
	AttitudeActualGet(&attitudeActual);
	StabilizationSettingsGet(&stabSettings);
	
	attitudeDesired.Yaw = 0;	// try and face north
	
	// Yaw and pitch output from ground speed PID loop
	northError = velocityDesired.North - velocityActual.North;
	northDerivative = (northError - northErrorLast) / dT;
	northIntegral =
	bound(northIntegral + northError * dT, -guidanceSettings.MaxVelIntegral,
		  guidanceSettings.MaxVelIntegral);
	if (isnan(northIntegral)) northIntegral = 0;
	northErrorLast = northError;
	northCommand =
	northError * guidanceSettings.VelP + northDerivative * guidanceSettings.VelD + northIntegral * guidanceSettings.VelI;
	
	eastError = velocityDesired.East - velocityActual.East;
	eastDerivative = (eastError - eastErrorLast) / dT;
	eastIntegral = bound(eastIntegral + eastError * dT, 
				 -guidanceSettings.MaxVelIntegral,
				 guidanceSettings.MaxVelIntegral);
	if (isnan(eastIntegral)) eastIntegral = 0;
	eastErrorLast = eastError;
	eastCommand = eastError * guidanceSettings.VelP + eastDerivative * guidanceSettings.VelD + eastIntegral * guidanceSettings.VelI;
	
	// Project the north and east command signals into the pitch and roll based on yaw.  For this to behave well the
	// craft should move similarly for 5 deg roll versus 5 deg pitch
	attitudeDesired.Pitch = bound(-northCommand * cosf(attitudeActual.Yaw * M_PI / 180) + 
					  eastCommand * sinf(attitudeActual.Yaw * M_PI / 180),
					  -stabSettings.PitchMax, stabSettings.PitchMax);
	attitudeDesired.Roll = bound(-northCommand * sinf(attitudeActual.Yaw * M_PI / 180) + 
					 eastCommand * cosf(attitudeActual.Yaw * M_PI / 180),
					 -stabSettings.RollMax, stabSettings.RollMax);
	
	downError = velocityDesired.Down - velocityActual.Down;
	downDerivative = (downError - downErrorLast) / guidanceSettings.VelPIDUpdatePeriod;
	downIntegral =	bound(downIntegral + downError * guidanceSettings.VelPIDUpdatePeriod, 
				  -guidanceSettings.MaxThrottleIntegral,
				  guidanceSettings.MaxThrottleIntegral);
	if (isnan(downIntegral)) downIntegral = 0;
	downErrorLast = downError;
	attitudeDesired.Throttle = bound(downError * guidanceSettings.DownP + downDerivative * guidanceSettings.DownD +
					 downIntegral * guidanceSettings.DownI, 0, 1);
	
	// For now override throttle with manual control.  Disable at your risk, quad goes to China.
	ManualControlCommandData manualControl;
	ManualControlCommandGet(&manualControl);
	attitudeDesired.Throttle = manualControl.Throttle;
	
	AttitudeDesiredSet(&attitudeDesired);
}

/**
 * Bound input value between limits
 */
static float bound(float val, float min, float max)
{
	if (val < min) {
		val = min;
	} else if (val > max) {
		val = max;
	}
	return val;
}
