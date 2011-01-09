/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{ 
 * @addtogroup FlightPlanSettings FlightPlanSettings 
 * @brief Settings for the flight plan module, control the execution of the script
 *
 * Autogenerated files and functions for FlightPlanSettings Object
 
 * @{ 
 *
 * @file       flightplansettings.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Implementation of the FlightPlanSettings object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: flightplansettings.xml. 
 *             This is an automatically generated file.
 *             DO NOT modify manually.
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

#ifndef FLIGHTPLANSETTINGS_H
#define FLIGHTPLANSETTINGS_H

// Object constants
#define FLIGHTPLANSETTINGS_OBJID 2234942498U
#define FLIGHTPLANSETTINGS_NAME "FlightPlanSettings"
#define FLIGHTPLANSETTINGS_METANAME "FlightPlanSettingsMeta"
#define FLIGHTPLANSETTINGS_ISSINGLEINST 1
#define FLIGHTPLANSETTINGS_ISSETTINGS 1
#define FLIGHTPLANSETTINGS_NUMBYTES sizeof(FlightPlanSettingsData)

// Object access macros
/**
 * @function FlightPlanSettingsGet(dataOut)
 * @brief Populate a FlightPlanSettingsData object
 * @param[out] dataOut 
 */
#define FlightPlanSettingsGet(dataOut) UAVObjGetData(FlightPlanSettingsHandle(), dataOut)
#define FlightPlanSettingsSet(dataIn) UAVObjSetData(FlightPlanSettingsHandle(), dataIn)
#define FlightPlanSettingsInstGet(instId, dataOut) UAVObjGetInstanceData(FlightPlanSettingsHandle(), instId, dataOut)
#define FlightPlanSettingsInstSet(instId, dataIn) UAVObjSetInstanceData(FlightPlanSettingsHandle(), instId, dataIn)
#define FlightPlanSettingsConnectQueue(queue) UAVObjConnectQueue(FlightPlanSettingsHandle(), queue, EV_MASK_ALL_UPDATES)
#define FlightPlanSettingsConnectCallback(cb) UAVObjConnectCallback(FlightPlanSettingsHandle(), cb, EV_MASK_ALL_UPDATES)
#define FlightPlanSettingsCreateInstance() UAVObjCreateInstance(FlightPlanSettingsHandle())
#define FlightPlanSettingsRequestUpdate() UAVObjRequestUpdate(FlightPlanSettingsHandle())
#define FlightPlanSettingsRequestInstUpdate(instId) UAVObjRequestInstanceUpdate(FlightPlanSettingsHandle(), instId)
#define FlightPlanSettingsUpdated() UAVObjUpdated(FlightPlanSettingsHandle())
#define FlightPlanSettingsInstUpdated(instId) UAVObjUpdated(FlightPlanSettingsHandle(), instId)
#define FlightPlanSettingsGetMetadata(dataOut) UAVObjGetMetadata(FlightPlanSettingsHandle(), dataOut)
#define FlightPlanSettingsSetMetadata(dataIn) UAVObjSetMetadata(FlightPlanSettingsHandle(), dataIn)
#define FlightPlanSettingsReadOnly(dataIn) UAVObjReadOnly(FlightPlanSettingsHandle())

// Object data
typedef struct {
    float Test;

} __attribute__((packed)) FlightPlanSettingsData;

// Field information
// Field Test information


// Generic interface functions
int32_t FlightPlanSettingsInitialize();
UAVObjHandle FlightPlanSettingsHandle();

#endif // FLIGHTPLANSETTINGS_H

/**
 * @}
 * @}
 */
