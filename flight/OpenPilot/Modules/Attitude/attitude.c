/**
 ******************************************************************************
 *
 * @file       attitude.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Module to read the attitude solution from the AHRS on a periodic basis.
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
 * Input object: AttitudeSettings
 * Output object: AttitudeActual
 *
 * This module will periodically update the value of latest attitude solution
 * that is available from the AHRS.
 * The module settings can configure how often AHRS is polled for a new solution.
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

#include "attitude.h"
#include "attitudeactual.h" // object that will be updated by the module
#include "attitudesettings.h" // object holding module settings

#include "pios_opahrs.h" // library for OpenPilot AHRS access functions

// Private constants
#define STACK_SIZE 200
#define TASK_PRIORITY (tskIDLE_PRIORITY+1)

// Private types

// Private variables
static xTaskHandle taskHandle;

// Private functions
static void attitudeTask(void* parameters);

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t AttitudeInitialize(void)
{
	// Start main task
	xTaskCreate(attitudeTask, (signed char*)"Attitude", STACK_SIZE, NULL, TASK_PRIORITY, &taskHandle);

	return 0;
}

/**
 * Module thread, should not return.
 */
static void attitudeTask(void* parameters)
{
	AttitudeSettingsData settings;
	AttitudeActualData data;

	// Main task loop
	while (1)
	{
		// Update settings with latest value
		AttitudeSettingsGet(&settings);

		// Get the current object data
		AttitudeActualGet(&data);

		// Query the latest attitude solution from the AHRS
		PIOS_OPAHRS_ReadAttitude();

		// Update the data
		data.seq++;
		data.q1 += 0.111;
		data.q2 += 1.1;
		data.q3 += 7.0;
		data.q4 -= 2.321;

		data.ex += 0.01;
		data.ey -= 0.03;
		data.ez += 0.05;

		// Update the ExampleObject, after this function is called
		// notifications to any other modules listening to that object
		// will be sent and the GCS object will be updated through the
		// telemetry link. All operations will take place asynchronously
		// and the following call will return immediately.
		AttitudeActualSet(&data);

		// Since this module executes at fixed time intervals, we need to
		// block the task until it is time for the next update.
		// The settings field is in ms, to convert to RTOS ticks we need
		// to divide by portTICK_RATE_MS.
		vTaskDelay( settings.UpdatePeriod / portTICK_RATE_MS );
	}
}
