/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{ 
 * @addtogroup TelemetrySettings TelemetrySettings 
 * @brief Select baud rate of telemetry.  Warning - this must match your modem.
 *
 * Autogenerated files and functions for TelemetrySettings Object
 
 * @{ 
 *
 * @file       telemetrysettings.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Implementation of the TelemetrySettings object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: telemetrysettings.xml. 
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

#ifndef TELEMETRYSETTINGS_H
#define TELEMETRYSETTINGS_H

// Object constants
#define TELEMETRYSETTINGS_OBJID 2785592614U
#define TELEMETRYSETTINGS_NAME "TelemetrySettings"
#define TELEMETRYSETTINGS_METANAME "TelemetrySettingsMeta"
#define TELEMETRYSETTINGS_ISSINGLEINST 1
#define TELEMETRYSETTINGS_ISSETTINGS 1
#define TELEMETRYSETTINGS_NUMBYTES sizeof(TelemetrySettingsData)

// Object access macros
/**
 * @function TelemetrySettingsGet(dataOut)
 * @brief Populate a TelemetrySettingsData object
 * @param[out] dataOut 
 */
#define TelemetrySettingsGet(dataOut) UAVObjGetData(TelemetrySettingsHandle(), dataOut)
#define TelemetrySettingsSet(dataIn) UAVObjSetData(TelemetrySettingsHandle(), dataIn)
#define TelemetrySettingsInstGet(instId, dataOut) UAVObjGetInstanceData(TelemetrySettingsHandle(), instId, dataOut)
#define TelemetrySettingsInstSet(instId, dataIn) UAVObjSetInstanceData(TelemetrySettingsHandle(), instId, dataIn)
#define TelemetrySettingsConnectQueue(queue) UAVObjConnectQueue(TelemetrySettingsHandle(), queue, EV_MASK_ALL_UPDATES)
#define TelemetrySettingsConnectCallback(cb) UAVObjConnectCallback(TelemetrySettingsHandle(), cb, EV_MASK_ALL_UPDATES)
#define TelemetrySettingsCreateInstance() UAVObjCreateInstance(TelemetrySettingsHandle())
#define TelemetrySettingsRequestUpdate() UAVObjRequestUpdate(TelemetrySettingsHandle())
#define TelemetrySettingsRequestInstUpdate(instId) UAVObjRequestInstanceUpdate(TelemetrySettingsHandle(), instId)
#define TelemetrySettingsUpdated() UAVObjUpdated(TelemetrySettingsHandle())
#define TelemetrySettingsInstUpdated(instId) UAVObjUpdated(TelemetrySettingsHandle(), instId)
#define TelemetrySettingsGetMetadata(dataOut) UAVObjGetMetadata(TelemetrySettingsHandle(), dataOut)
#define TelemetrySettingsSetMetadata(dataIn) UAVObjSetMetadata(TelemetrySettingsHandle(), dataIn)
#define TelemetrySettingsReadOnly(dataIn) UAVObjReadOnly(TelemetrySettingsHandle())

// Object data
typedef struct {
    uint8_t Speed;

} __attribute__((packed)) TelemetrySettingsData;

// Field information
// Field Speed information
/* Enumeration options for field Speed */
typedef enum { TELEMETRYSETTINGS_SPEED_9600=0, TELEMETRYSETTINGS_SPEED_38400=1, TELEMETRYSETTINGS_SPEED_57600=2, TELEMETRYSETTINGS_SPEED_115200=3 } TelemetrySettingsSpeedOptions;


// Generic interface functions
int32_t TelemetrySettingsInitialize();
UAVObjHandle TelemetrySettingsHandle();

#endif // TELEMETRYSETTINGS_H

/**
 * @}
 * @}
 */
