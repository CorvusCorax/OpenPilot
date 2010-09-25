/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup AHRSCommsModule AHRSComms Module
 * @brief Handles communication with AHRS and updating position
 * Specifically updates the the @ref AttitudeActual "AttitudeActual" and @ref AttitudeRaw "AttitudeRaw" settings objects
 * @{
 *
 * @file       ahrs_comms.c
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
 * Input objects: As defined in PiOS/inc/pios_ahrs_comms.h
 * Output objects: As defined in PiOS/inc/pios_ahrs_comms.h
 *
 * This module will periodically update the values of latest attitude solution
 * and other objects that are transferred to and from the AHRS
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

#include "ahrs_comms.h"
#include "CoordinateConversions.h"
#include "stdbool.h"

#include "pios_ahrs_comms.h" // library for OpenPilot AHRS access functions

// Private constants
#define STACK_SIZE 1000
#define TASK_PRIORITY (tskIDLE_PRIORITY+4)

// Private types

typedef struct
{
    UAVObjHandle object;
    PIOS_AHRS_Handle memory;
} ObjectMap;

// Private variables
static xTaskHandle taskHandle;
static ObjectMap[MAX_AHRS_OBJECTS] objectMap;


// Private functions
static void ahrscommsTask(void* parameters);
static void ObjectUpdatedCb(UAVObjEvent * ev);
static void AhrsUpdatedCb(PIOS_AHRS_Handle handle);


/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t AHRSCommsInitialize(void)
{

    PIOS_AhrsCommsInit();

#define ADDMAP(hnd) ObjectMap[idx].object = hnd##Handle(); ObjectMap[idx++].memory = &(PIOS_AHRSGetMemory()->hnd)
    int idx = 0;
    initError = 0;
    ADDMAP(AttitudeActual);
    ADDMAP(AttitudeRaw);
    ADDMAP(AHRSSettings);
    ADDMAP(AHRSCalibration);
    ADDMAP(AttitudeSettings);
    ADDMAP(AhrsStatus);
    ADDMAP(BaroAltitude);
    ADDMAP(GPSPosition);
    ADDMAP(PositionActual);
    ADDMAP(HomeLocation);
    if(idx != MAX_AHRS_OBJECTS) //Each ADDMAP above incremented idx
    {
        PIOS_DEBUG_Assert(0);
    }


    AHRSSettingsConnectCallback(ObjectUpdatedCb);
    BaroAltitudeConnectCallback(ObjectUpdatedCb);
    GPSPositionConnectCallback(ObjectUpdatedCb);
    HomeLocationConnectCallback(ObjectUpdatedCb);
    AHRSCalibrationConnectCallback(ObjectUpdatedCb);


    // Start main task
    xTaskCreate(ahrscommsTask, (signed char*)"AHRSComms", STACK_SIZE, NULL, TASK_PRIORITY, &taskHandle);

    return 0;
}


/**
 * Module thread, should not return.
 */
static void ahrscommsTask(void* parameters)
{
    enum opahrs_result result;
    portTickType lastSysTime;

    AhrsStatusData data;

    AlarmsSet(SYSTEMALARMS_ALARM_AHRSCOMMS, SYSTEMALARMS_ALARM_CRITICAL);

    /*Until AHRS connects, assume it doesn't know home */
    AhrsStatusGet(&data);
    data.HomeSet = AHRSSTATUS_HOMESET_FALSE;
    //data.CalibrationSet = AHRSSTATUS_CALIBRATIONSET_FALSE;
    data.AlgorithmSet = AHRSSTATUS_CALIBRATIONSET_FALSE;
    AhrsStatusSet(&data);

    // Main task loop
    while (1)
    {

        PIOS_AHRS_SendObjects();
        if(PIOS_AHRS_GetError() != )
        AlarmsClear(SYSTEMALARMS_ALARM_AHRSCOMMS);

        /* Wait for the next update interval */
        vTaskDelayUntil(&lastSysTime, settings.UpdatePeriod / portTICK_RATE_MS );

    }
}


static void AhrsUpdatedCb(PIOS_AHRS_Handle handle)
{
    for(int ct=0; ct< MAX_AHRS_OBJECTS; ct++)
    {
        if(handle == objectMap[ct].data)
        {
            PIOS_AHRS_SharedObject data; //this is guaranteed to be big enough
            PIOS_AHRS_GetData(objectMap[ct].memory, &data);
            UAVObjectSetData(objectMap[ct].object, &data)
            return;
        }
    }
}

static void ObjectUpdatedCb(UAVObjEvent * ev)
{
    if(!(ev->event & EV_MASK_ALL_UPDATES))
    {
        return;
    }
    for(int ct=0; ct< MAX_AHRS_OBJECTS; ct++)
    {
        if(ev->obj == objectMap[ct].object)
        {
            PIOS_AHRS_SharedObject data; //this is guaranteed to be big enough
            UAVObjGetData(ev->obj,&data);
            PIOS_AHRS_SetData(objectMap[ct].memory, &data);
            return;
        }
    }
}






/**
  * @}
  * @}
  */
