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
#include "attitudedesired.h"
#include "attitudeactual.h"
#include "attituderaw.h"
#include "manualcontrolcommand.h"
#include "systemsettings.h"
#include "ahrssettings.h"


// Private constants
#define STACK_SIZE configMINIMAL_STACK_SIZE
#define TASK_PRIORITY (tskIDLE_PRIORITY+4)
enum {PID_MANUALROLL,PID_MANUALPITCH,PID_MANUALYAW,PID_STABROLL,PID_STABPITCH,PID_STABYAW,PID_MAX};
enum {ROLL,PITCH,YAW,MAX_AXES};


// Private types
typedef struct {
    float * p;
    float * i;
    float * d;
    float * iLim;
    float iAccumulator;
    float lastErr;
} pid_type;

// Private variables
static xTaskHandle taskHandle;

// Private functions
static void stabilizationTask(void* parameters);
static float ApplyPid(pid_type * pid, const float desired, const float actual);
static float bound(float val);

/**
 * Module initialization
 */
int32_t StabilizationInitialize()
{
	// Initialize variables

	// Start main task
	xTaskCreate(stabilizationTask, (signed char*)"Stabilization", STACK_SIZE, NULL, TASK_PRIORITY, &taskHandle);

	return 0;
}

/**
 * Module task
 */
static void stabilizationTask(void* parameters)
{
	portTickType lastSysTime;
    pid_type pids[PID_MAX];
    LesStabilizationSettingsData stabSettings;
    ActuatorDesiredData actuatorDesired;
    AttitudeDesiredData attitudeDesired;
    AttitudeActualData attitudeActual;
    AttitudeRawData attitudeRaw;
    ManualControlCommandData manualControl;
    SystemSettingsData systemSettings;

	StabilizationSettingsGet(&stabSettings);
	AHRSSettingsData ahrs;
    float * valuePtr = &stabSettings.ManualRoll;
    for(int ct = 0; ct < PID_MAX; ct++)
    {
        pid_type * pid = &pids[ct];
        pid->p = valuePtr++;
        pid->i = valuePtr++;
        pid->d = valuePtr++;
        pid->iLim = valuePtr++;

        //FIXME: this is a really hacky way of applying defaults
        if(*pid->p < 0)
        {
            *pid->p = 1;
            *pid->i = 0;
            *pid->d = 0;
            *pid->iLim = 0;
        }
        pid->iAccumulator = 0;
        pid->lastErr = 0;
    }
	StabilizationSettingsSet(&stabSettings);

	// Main task loop
	lastSysTime = xTaskGetTickCount();
	while (1)
	{
		// Read settings and other objects
		StabilizationSettingsGet(&stabSettings);
		SystemSettingsGet(&systemSettings);
		ManualControlCommandGet(&manualControl);
		AttitudeDesiredGet(&attitudeDesired);
		AttitudeActualGet(&attitudeActual);
		AttitudeRawGet(&attitudeRaw);
		actuatorDesiredGet(&actuatorDesired);

        float rates[MAX_AXES]={0,0,0};
        //First calculate the rate commands
        if(manualControl.FlightMode == MANUALCONTROLCOMMAND_FLIGHTMODE_MANUAL)
        {
            rates[ROLL] = manualControl.Roll;
            rates[PITCH] = manualControl.Pitch;
            rates[YAW] = manualControl.Yaw;
        }else
        {
            rates[ROLL] = ApplyPid(&pids[PID_STABROLL], AttitudeDesired.Roll, AttitudeActual.Roll);
            rates[PITCH] = ApplyPid(&pids[PID_STABPITCH], AttitudeDesired.Pitch, AttitudeActual.Pitch);

            float yawDesired = AttitudeDesired.Yaw;
            float diff = yawDesired - AttitudeActual.Yaw;
            if(diff > 180)
            {
                yawDesired -=
            }


            rates[YAW] = ApplyPid(&pids[PID_STABYAW], AttitudeDesired.Yaw, AttitudeActual.Yaw);
        }

        //Now calculate the actuator commands
        if(stabSettings.ManualStabilization >= LESSTABILIZATIONSETTINGS_MANUALSTABILIZATION_YAW)
        {
             actuatorDesired.Yaw = Bound(ApplyPid(&pids[PID_MANUALYAW], rates[YAW], attitudeRaw.gyros_filtered[YAW]));
        }else
        {
            actuatorDesired.Yaw = rates[YAW];
        }
        if(stabSettings.ManualStabilization >= LESSTABILIZATIONSETTINGS_MANUALSTABILIZATION_ALL)
        {
            actuatorDesired.Roll = Bound(ApplyPid(&pids[PID_MANUALROLL], rates[ROLL], attitudeRaw.gyros_filtered[ROLL]));
            actuatorDesired.Pitch = Bound(ApplyPid(&pids[PID_MANUALPITCH], rates[PITCH], attitudeRaw.gyros_filtered[PITCH]));
        }else
        {
            actuatorDesired.Roll = rates[ROLL];
            actuatorDesired.Pitch = rates[Pitch];
        }

		if(ahrs.UpdateRaw != AHRSSETTINGS_UPDATERAW_TRUE) //we need raw AHRS data
		{
		    AlarmsSet(SYSTEMALARMS_ALARM_STABILIZATION, SYSTEMALARMS_ALARM_CRITICAL);
		}else
		{
       		AlarmsClear(SYSTEMALARMS_ALARM_STABILIZATION);
		}

		// Wait until next update
		vTaskDelayUntil(&lastSysTime, stabSettings.UpdatePeriod / portTICK_RATE_MS );
	}
}

float ApplyPid(pid_type * pid, const float desired, const float actual)
{
    float err = desired - actual;
    float diff = (err - pid->lastErr);
    pid->lastErr = err;
    if(fabs(err) < pid->iLim)
    {
        pid->iAccumulator += err * (*pid->i);
        if (fabs(pid->iAccumulator) > iLim)
        {
            if (pid->iAccumulator >0)
            {
                pid->iAccumulator = iLim;
            }
            else
            {
                pid->iAccumulator = -iLim;
            }
        }
    }
    return ((err * (*pid->p) + pid->iAccumulator + (diff * (*pid->d));
}




/**
 * Bound input value between limits
 */
static float bound(float val)
{
	if ( val < -1 )
	{
		val = -1;
	}
	else if ( val > 1 )
	{
		val = 1;
	}
	return val;
}

/**
  * @}
  * @}
  */
