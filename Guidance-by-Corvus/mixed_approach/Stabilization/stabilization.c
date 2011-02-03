/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup StabilizationModule Stabilization Module
 * @brief Stabilization PID loops in an airframe type independent manner
 * @note This object updates the @ref ActuatorDesired "Actuator Desired" based on the
 * PID loops on the @ref AttitudeDesired "Attitude Desired" and @ref AttitudeActual "Attitude Actual"
 * @{
 *
 * @file       stabilization.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Attitude stabilization module.
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

#include "openpilot.h"
#include "stabilization.h"
#include "stabilizationsettings.h"
#include "actuatordesired.h"
#include "ratedesired.h"
#include "attitudedesired.h"
#include "attitudeactual.h"
#include "attituderaw.h"
#include "manualcontrolcommand.h"
#include "systemsettings.h"
#include "ahrssettings.h"

// Private constants
#define MAX_QUEUE_SIZE 1

#if defined(PIOS_STABILIZATION_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_STABILIZATION_STACK_SIZE
#else
#define STACK_SIZE_BYTES 724
#endif

#define TASK_PRIORITY (tskIDLE_PRIORITY+4)
#define FAILSAFE_TIMEOUT_MS 30
#define DEG2RAD ( M_PI / 180.0 )

enum {PID_RATE_ROLL, PID_RATE_PITCH, PID_RATE_YAW, PID_ROLL, PID_PITCH, PID_YAW, PID_MAX};

enum {ROLL,PITCH,YAW,MAX_AXES};


// Private types
typedef struct {
	float p;
	float i;
	float d;
	float iLim;
	float iAccumulator;
	float lastErr;
} pid_type;

// Private variables
static xTaskHandle taskHandle;
static StabilizationSettingsData settings;
static xQueueHandle queue;
float dT = 1;
pid_type pids[PID_MAX];

// Private functions
static void stabilizationTask(void* parameters);
static float ApplyPid(pid_type * pid, const float error);
static float bound(float val);
static void ZeroPids(void);
static void SettingsUpdatedCb(UAVObjEvent * ev);
static void calcDifference(float * values, float * reference, const uint8_t angular);
static void translateValues(float * values, float * reference);

/**
 * Module initialization
 */
int32_t StabilizationInitialize()
{
	// Initialize variables

	// Create object queue
	queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));

	// Listen for updates.
//	AttitudeActualConnectQueue(queue);
	AttitudeRawConnectQueue(queue);

	StabilizationSettingsConnectCallback(SettingsUpdatedCb);
	SettingsUpdatedCb(StabilizationSettingsHandle());
	// Start main task
	xTaskCreate(stabilizationTask, (signed char*)"Stabilization", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY, &taskHandle);
	TaskMonitorAdd(TASKINFO_RUNNING_STABILIZATION, taskHandle);
	PIOS_WDG_RegisterFlag(PIOS_WDG_STABILIZATION);
	
	return 0;
}

/**
 * Module task
 */
static void stabilizationTask(void* parameters)
{
	portTickType lastSysTime;
	portTickType thisSysTime;
	UAVObjEvent ev;


	ActuatorDesiredData actuatorDesired;
	AttitudeDesiredData attitudeDesired;
	RateDesiredData rateDesired;
	AttitudeActualData attitudeActual;
	AttitudeRawData attitudeRaw;
	SystemSettingsData systemSettings;
	ManualControlCommandData manualControl;

	SettingsUpdatedCb((UAVObjEvent *) NULL);

	// Main task loop
	lastSysTime = xTaskGetTickCount();
	ZeroPids();
	while(1) {
		PIOS_WDG_UpdateFlag(PIOS_WDG_STABILIZATION);

		// Wait until the AttitudeRaw object is updated, if a timeout then go to failsafe
		if ( xQueueReceive(queue, &ev, FAILSAFE_TIMEOUT_MS / portTICK_RATE_MS) != pdTRUE )
		{
			AlarmsSet(SYSTEMALARMS_ALARM_STABILIZATION,SYSTEMALARMS_ALARM_WARNING);
			continue;
		} 
		
		// Check how long since last update
		thisSysTime = xTaskGetTickCount();
		if(thisSysTime > lastSysTime) // reuse dt in case of wraparound
			dT = (thisSysTime - lastSysTime) / portTICK_RATE_MS / 1000.0f;
		lastSysTime = thisSysTime;

		ManualControlCommandGet(&manualControl);
		AttitudeDesiredGet(&attitudeDesired);
		AttitudeActualGet(&attitudeActual);
		AttitudeRawGet(&attitudeRaw);
		RateDesiredGet(&rateDesired);
		SystemSettingsGet(&systemSettings);


		float *attitudeDesiredAxis = &attitudeDesired.Roll;
		float *attitudeActualAxis = &attitudeActual.Roll;
		float *actuatorDesiredAxis = &actuatorDesired.Roll;
		float *rateDesiredAxis = &rateDesired.Roll;
		float localRate[3];

		// calculate attitude errors
		calcDifference( attitudeDesiredAxis, attitudeActualAxis, 1);

		//Zero axis errors we are not using
		for (int8_t ct=0; ct< MAX_AXES; ct++)
		{
			switch (manualControl.StabilizationSettings[ct]) {
				case MANUALCONTROLCOMMAND_STABILIZATIONSETTINGS_ATTITUDE:   // attitude stabilization.
					rateDesiredAxis[ct] = 0;                                // ignore rate demand
					localRate[ct] = 0;                                      // reset translated rate demand
					break;
				case MANUALCONTROLCOMMAND_STABILIZATIONSETTINGS_RATE:       // local rate stabilization.
					localRate[ct] = rateDesiredAxis[ct];                    // set translated rate demand directly
					rateDesiredAxis[ct] = 0;                                // disable rate translation
					attitudeDesiredAxis[ct] = 0;                            // ignore attitude demand
					break;
				case MANUALCONTROLCOMMAND_STABILIZATIONSETTINGS_MIXED:      // mixed mode (stunt mode)
					localRate[ct] = rateDesiredAxis[ct];                    // set translated rate demand directly
					rateDesiredAxis[ct] = 0;                                // disable rate translation
					break;
				case MANUALCONTROLCOMMAND_STABILIZATIONSETTINGS_GLOBALRATE: // rate stabilization
					attitudeDesiredAxis[ct] = 0;                            // ignore attitude demand
					localRate[ct] = 0;                                      // reset translated rate demand
					break;
			}
		}

		//Translate attitude and global rate to local reference frame.
		translateValues(attitudeDesiredAxis, attitudeActualAxis);
		translateValues(rateDesiredAxis, attitudeActualAxis);

		//Calculate desired rate
		for(int8_t ct=0; ct< MAX_AXES; ct++)
		{
			localRate[ct] += rateDesiredAxis[ct];                                      // add (translated) rate desired
			localRate[ct] += ApplyPid(&pids[PID_ROLL + ct],  attitudeDesiredAxis[ct]); // add result of attitude PID loop
			if(fabs(localRate[ct]) > settings.MaximumRate[ct]) // make sure we stay within bounds
			{
				if(localRate[ct] > 0)
				{
					localRate[ct] = settings.MaximumRate[ct];
				}else
				{
					localRate[ct] = -settings.MaximumRate[ct];
				}
			}
		}

		uint8_t shouldUpdate = 0;
		//RateDesiredSet(&rateDesired); TODO: need new UAVObject to store local rate
		ActuatorDesiredGet(&actuatorDesired);

		// calculate rate errors
		calcDifference( localRate, attitudeRaw.gyros, 0);

		//Calculate desired command
		for(int8_t ct=0; ct< MAX_AXES; ct++)
		{
			switch(manualControl.StabilizationSettings[ct])
			{
			case MANUALCONTROLCOMMAND_STABILIZATIONSETTINGS_RATE:
			case MANUALCONTROLCOMMAND_STABILIZATIONSETTINGS_GLOBALRATE:
			case MANUALCONTROLCOMMAND_STABILIZATIONSETTINGS_ATTITUDE:
			case MANUALCONTROLCOMMAND_STABILIZATIONSETTINGS_MIXED:
				{
					float command = ApplyPid(&pids[PID_RATE_ROLL + ct],  localRate[ct]);
					actuatorDesiredAxis[ct] = bound(command);
					shouldUpdate = 1;
					break;
				}
            case MANUALCONTROLCOMMAND_STABILIZATIONSETTINGS_DIRECT:
                    //actuatorDesiredAxis[ct] = bound(manualAxis[ct]);
                    //shouldUpdate = 1;
                    switch (ct)
                    {
                    case ROLL:
                            actuatorDesiredAxis[ct] = bound(attitudeDesiredAxis[ct]/settings.RollMax);
                            shouldUpdate = 1;
                    break;
                    case PITCH:
                            actuatorDesiredAxis[ct] = bound(attitudeDesiredAxis[ct]/settings.PitchMax);
                            shouldUpdate = 1;
                    break;
                    case YAW:
                            actuatorDesiredAxis[ct] = bound(attitudeDesiredAxis[ct]/180);
                            shouldUpdate = 1;
                    break;
                    }
                break;

			}
		}

		// Save dT
		actuatorDesired.UpdateTime = dT * 1000;

		if(manualControl.FlightMode == MANUALCONTROLCOMMAND_FLIGHTMODE_MANUAL)
		{
			shouldUpdate = 0;
		}


		if(shouldUpdate)
		{
			actuatorDesired.Throttle = attitudeDesired.Throttle;
			if(dT > 15)
				actuatorDesired.NumLongUpdates++;
			ActuatorDesiredSet(&actuatorDesired);
		}

		if(manualControl.Armed == MANUALCONTROLCOMMAND_ARMED_FALSE ||
			!shouldUpdate || (attitudeDesired.Throttle < 0))
		{
			ZeroPids();
		}
		
		// Clear alarms
		AlarmsClear(SYSTEMALARMS_ALARM_STABILIZATION);		
	}
}


float ApplyPid(pid_type * pid, const float err)
{
	float diff = (err - pid->lastErr);
	pid->lastErr = err;
	pid->iAccumulator += err * pid->i * dT;
	if (isnan(pid->iAccumulator)) pid->iAccumulator = 0;
	if(fabs(pid->iAccumulator) > pid->iLim) {
		if(pid->iAccumulator >0) {
			pid->iAccumulator = pid->iLim;
		} else {
			pid->iAccumulator = -pid->iLim;
		}
	}
	return ((err * pid->p) + pid->iAccumulator + (diff * pid->d / dT));
}


static void ZeroPids(void)
{
	for(int8_t ct = 0; ct < PID_MAX; ct++) {
		pids[ct].iAccumulator = 0;
		pids[ct].lastErr = 0;
	}
}


/**
 * Bound input value between limits
 */
static float bound(float val)
{
	if(val < -1) {
		val = -1;
	} else if(val > 1) {
		val = 1;
	}
	if (isnan(val)) return 0;
	return val;
}


/**
 * calculate difference vector
 */
static void calcDifference(float * values, float * reference, const uint8_t angular)
{
	for(int8_t ct=0; ct< MAX_AXES; ct++)
	{
		values[ct] = values[ct] - reference[ct];
		if (angular) {
			if (values[ct] >  180.) values[ct] -= 360.;
			if (values[ct] < -180.) values[ct] += 360.;
		}
	}
}


/**
 * translate rotational vector into local reference frame
 */
static void translateValues(float * values, float * reference)
{

	float tmp[MAX_AXES];

	// traditional translation: rotate YAW and PITCH by roll
	tmp[PITCH] = cos( DEG2RAD * reference[ROLL] ) * values[PITCH]
			+ sin( DEG2RAD * reference[ROLL] ) * values[YAW];
	tmp[YAW]   = cos( DEG2RAD * reference[ROLL] ) * values[YAW]
			- sin( DEG2RAD * reference[ROLL] ) * values[PITCH];
	values[PITCH] = tmp[PITCH];
	values[YAW]   = tmp[YAW];

}

static void SettingsUpdatedCb(UAVObjEvent * ev)
{
	memset(pids,0,sizeof (pid_type) * PID_MAX);
	StabilizationSettingsGet(&settings);

	float * data = settings.RollRatePI;
	for(int8_t pid=0; pid < PID_MAX; pid++)
	{
		pids[pid].p = *data++;
		pids[pid].i = *data++;
		pids[pid].iLim = *data++;
	}
}


/**
  * @}
  * @}
  */
