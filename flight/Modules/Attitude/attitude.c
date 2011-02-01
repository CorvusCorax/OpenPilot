/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup Attitude Copter Control Attitude Estimation
 * @brief Acquires sensor data and computes attitude estimate 
 * Specifically updates the the @ref AttitudeActual "AttitudeActual" and @ref AttitudeRaw "AttitudeRaw" settings objects
 * @{
 *
 * @file       attitude.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Module to handle all comms to the AHRS on a periodic basis.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************/
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
 * Input objects: None, takes sensor data via pios
 * Output objects: @ref AttitudeRaw @ref AttitudeActual
 *
 * This module computes an attitude estimate from the sensor data
 *
 * The module executes in its own thread.
 *
 * UAVObjects are automatically generated by the UAVObjectGenerator from
 * the object definition XML file.
 *
 * Modules have no API, all communication to other modules is done through UAVObjects.
 * However modules may use the API exposed by shared libraries.
 * See the OpenPilot wiki for more details.
 * http://www.openpilot.org/OpenPilot_Application_Architecture
 *
 */

#include "pios.h"
#include "attitude.h"
#include "attituderaw.h"
#include "attitudeactual.h"
#include "attitudedesired.h"
#include "attitudesettings.h"
#include "manualcontrolcommand.h"
#include "CoordinateConversions.h"
#include "pios_flash_w25x.h"

// Private constants
#define STACK_SIZE_BYTES 440
#define TASK_PRIORITY (tskIDLE_PRIORITY+3)

#define UPDATE_RATE  3
#define GYRO_NEUTRAL 1665
#define GYRO_SCALE   (0.008f * 180 / M_PI)

#define PI_MOD(x) (fmod(x + M_PI, M_PI * 2) - M_PI)
// Private types

// Private variables
static xTaskHandle taskHandle;

// Private functions
static void AttitudeTask(void *parameters);

void adc_callback(float * data);
float gyro[3] = {0, 0, 0};

void updateSensors();
void updateAttitude();

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t AttitudeInitialize(void)
{
	// Start main task
	xTaskCreate(AttitudeTask, (signed char *)"Attitude", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY, &taskHandle);
	TaskMonitorAdd(TASKINFO_RUNNING_ATTITUDE, taskHandle);
	PIOS_WDG_RegisterFlag(PIOS_WDG_ATTITUDE);
	return 0;
}
static portTickType lastSysTime;
/**
 * Module thread, should not return.
 */
static void AttitudeTask(void *parameters)
{

	AlarmsClear(SYSTEMALARMS_ALARM_ATTITUDE);

	PIOS_ADC_SetCallback(adc_callback);

	// Keep flash CS pin high while talking accel
	PIOS_FLASH_DISABLE;		
	PIOS_ADXL345_Init();

	// Main task loop
	while (1) {
		//
		PIOS_WDG_UpdateFlag(PIOS_WDG_ATTITUDE);
		
		// TODO: register the adc callback, push the data onto a queue (safe for thread)
		// with the queue ISR version
		updateSensors();		
		updateAttitude();
		
		/* Wait for the next update interval */
		vTaskDelayUntil(&lastSysTime, UPDATE_RATE / portTICK_RATE_MS);
		//vTaskDelay(UPDATE_RATE / portTICK_RATE_MS);

	}
}

void updateSensors() 
{
	AttitudeRawData attitudeRaw;
	AttitudeRawGet(&attitudeRaw);		
	
	AttitudeSettingsData settings;
	AttitudeSettingsGet(&settings);
	
	struct pios_adxl345_data accel_data;
	
	static float gyro_bias[3] = {0,0,0};
	float tau = (1-settings.GyroBiasTau);

	attitudeRaw.gyros_filtered[ATTITUDERAW_GYROS_FILTERED_X] = -(gyro[0] - GYRO_NEUTRAL) * GYRO_SCALE;
	attitudeRaw.gyros_filtered[ATTITUDERAW_GYROS_FILTERED_Y] = (gyro[1] - GYRO_NEUTRAL) * GYRO_SCALE;
	attitudeRaw.gyros_filtered[ATTITUDERAW_GYROS_FILTERED_Z] = -(gyro[2] - GYRO_NEUTRAL) * GYRO_SCALE;
	
	gyro_bias[0] = tau * gyro_bias[0] + (1-tau) * attitudeRaw.gyros_filtered[ATTITUDERAW_GYROS_FILTERED_X];
	gyro_bias[1] = tau * gyro_bias[1] + (1-tau) * attitudeRaw.gyros_filtered[ATTITUDERAW_GYROS_FILTERED_Y];
	gyro_bias[2] = tau * gyro_bias[2] + (1-tau) * attitudeRaw.gyros_filtered[ATTITUDERAW_GYROS_FILTERED_Z];
	
	attitudeRaw.gyros_filtered[ATTITUDERAW_GYROS_FILTERED_X] -= gyro_bias[0];
	attitudeRaw.gyros_filtered[ATTITUDERAW_GYROS_FILTERED_Y] -= gyro_bias[1];
	attitudeRaw.gyros_filtered[ATTITUDERAW_GYROS_FILTERED_Z] -= gyro_bias[2];
	
	// Get the accel data
	uint8_t i = 0;
	attitudeRaw.accels_filtered[ATTITUDERAW_ACCELS_FILTERED_X] = 0;
	attitudeRaw.accels_filtered[ATTITUDERAW_ACCELS_FILTERED_Y] = 0;
	attitudeRaw.accels_filtered[ATTITUDERAW_ACCELS_FILTERED_Z] = 0; 
	
	do {
		i++;
		attitudeRaw.gyrotemp[0] = PIOS_ADXL345_Read(&accel_data);
		
		attitudeRaw.accels_filtered[ATTITUDERAW_ACCELS_FILTERED_X] += (float) accel_data.x * 0.004f * 9.81;
		attitudeRaw.accels_filtered[ATTITUDERAW_ACCELS_FILTERED_Y] += -(float) accel_data.y * 0.004f * 9.81;
		attitudeRaw.accels_filtered[ATTITUDERAW_ACCELS_FILTERED_Z] += -(float) accel_data.z * 0.004f * 9.81;
	} while ( (i < 32) && (attitudeRaw.gyrotemp[0] > 0) );
	attitudeRaw.gyrotemp[1] = i;
	
	attitudeRaw.accels_filtered[ATTITUDERAW_ACCELS_FILTERED_X] /= i;
	attitudeRaw.accels_filtered[ATTITUDERAW_ACCELS_FILTERED_Y] /= i;
	attitudeRaw.accels_filtered[ATTITUDERAW_ACCELS_FILTERED_Z] /= i; 
	
	attitudeRaw.accels[ATTITUDERAW_ACCELS_X] = accel_data.x;
	attitudeRaw.accels[ATTITUDERAW_ACCELS_Y] = accel_data.y;
	attitudeRaw.accels[ATTITUDERAW_ACCELS_Z] = accel_data.z;
	
	AttitudeRawSet(&attitudeRaw); 	
}

#define UPDATE_FRAC 0.99999f
void updateAttitude()
{
	AttitudeSettingsData settings;
	AttitudeSettingsGet(&settings);

	AttitudeActualData attitudeActual;
	AttitudeActualGet(&attitudeActual);
	
	AttitudeRawData attitudeRaw;
	AttitudeRawGet(&attitudeRaw);		

	static portTickType lastSysTime = 0;
	static portTickType thisSysTime;
	
	float accel_pitch, accel_roll;
	static float dT = 0;
	float tau = 1-settings.AccelTau;
	
	thisSysTime = xTaskGetTickCount();
	if(thisSysTime > lastSysTime) // reuse dt in case of wraparound
		dT = (thisSysTime - lastSysTime) / portTICK_RATE_MS / 1000.0f;
	lastSysTime = thisSysTime;
	
	// Convert into radians
	attitudeActual.Roll = attitudeActual.Roll * M_PI / 180;
	attitudeActual.Pitch = attitudeActual.Pitch * M_PI / 180;
	attitudeActual.Yaw = attitudeActual.Yaw * M_PI / 180;
	
	// Integrate gyros
	attitudeActual.Roll  = PI_MOD(attitudeActual.Roll + attitudeRaw.gyros_filtered[ATTITUDERAW_GYROS_FILTERED_X] * dT * M_PI / 180);
	attitudeActual.Pitch = PI_MOD(attitudeActual.Pitch + attitudeRaw.gyros_filtered[ATTITUDERAW_GYROS_FILTERED_Y] * dT * M_PI / 180);
	attitudeActual.Yaw += attitudeRaw.gyros_filtered[ATTITUDERAW_GYROS_FILTERED_Z] * dT * M_PI / 180;
	       
	// Compute gravity sense of ground
	accel_roll = atan2(-attitudeRaw.accels_filtered[ATTITUDERAW_ACCELS_FILTERED_Y],
			   -attitudeRaw.accels_filtered[ATTITUDERAW_ACCELS_FILTERED_Z]);
	accel_pitch = atan2(attitudeRaw.accels_filtered[ATTITUDERAW_ACCELS_FILTERED_X],
			    -attitudeRaw.accels_filtered[ATTITUDERAW_ACCELS_FILTERED_Z]);
	
	// Compute quaternion
	RPY2Quaternion(&attitudeActual.Roll, &attitudeActual.q1);
	
	// Weighted average and back into degrees
	attitudeActual.Roll = (tau * attitudeActual.Roll + (1-tau) * accel_roll) * 180 / M_PI;
	attitudeActual.Pitch = (tau * attitudeActual.Pitch + (1-tau) * accel_pitch) * 180 / M_PI;
	attitudeActual.Yaw = fmod(attitudeActual.Yaw * 180 / M_PI, 360);	
	AttitudeActualSet(&attitudeActual);
}

void adc_callback(float * data) 
{
	gyro[0] = data[1];
	gyro[1] = data[2];
	gyro[2] = data[3];
}

/**
  * @}
  * @}
  */
