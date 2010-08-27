/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{ 
 * @addtogroup AttitudeActual AttitudeActual 
 * @brief The updated Attitude estimation from @ref AHRSCommsModule.
 *
 * Autogenerated files and functions for AttitudeActual Object
 
 * @{ 
 *
 * @file       attitudeactual.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Implementation of the AttitudeActual object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: attitudeactual.xml. 
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

#ifndef ATTITUDEACTUAL_H
#define ATTITUDEACTUAL_H

// Object constants
#define ATTITUDEACTUAL_OBJID 4233858292U
#define ATTITUDEACTUAL_NAME "AttitudeActual"
#define ATTITUDEACTUAL_METANAME "AttitudeActualMeta"
#define ATTITUDEACTUAL_ISSINGLEINST 1
#define ATTITUDEACTUAL_ISSETTINGS 0
#define ATTITUDEACTUAL_NUMBYTES sizeof(AttitudeActualData)

// Object access macros
/**
 * @function AttitudeActualGet(dataOut)
 * @brief Populate a AttitudeActualData object
 * @param[out] dataOut 
 */
#define AttitudeActualGet(dataOut) UAVObjGetData(AttitudeActualHandle(), dataOut)
#define AttitudeActualSet(dataIn) UAVObjSetData(AttitudeActualHandle(), dataIn)
#define AttitudeActualInstGet(instId, dataOut) UAVObjGetInstanceData(AttitudeActualHandle(), instId, dataOut)
#define AttitudeActualInstSet(instId, dataIn) UAVObjSetInstanceData(AttitudeActualHandle(), instId, dataIn)
#define AttitudeActualConnectQueue(queue) UAVObjConnectQueue(AttitudeActualHandle(), queue, EV_MASK_ALL_UPDATES)
#define AttitudeActualConnectCallback(cb) UAVObjConnectCallback(AttitudeActualHandle(), cb, EV_MASK_ALL_UPDATES)
#define AttitudeActualCreateInstance() UAVObjCreateInstance(AttitudeActualHandle())
#define AttitudeActualRequestUpdate() UAVObjRequestUpdate(AttitudeActualHandle())
#define AttitudeActualRequestInstUpdate(instId) UAVObjRequestInstanceUpdate(AttitudeActualHandle(), instId)
#define AttitudeActualUpdated() UAVObjUpdated(AttitudeActualHandle())
#define AttitudeActualInstUpdated(instId) UAVObjUpdated(AttitudeActualHandle(), instId)
#define AttitudeActualGetMetadata(dataOut) UAVObjGetMetadata(AttitudeActualHandle(), dataOut)
#define AttitudeActualSetMetadata(dataIn) UAVObjSetMetadata(AttitudeActualHandle(), dataIn)
#define AttitudeActualReadOnly(dataIn) UAVObjReadOnly(AttitudeActualHandle())

// Object data
typedef struct {
    float q1;
    float q2;
    float q3;
    float q4;
    float Roll;
    float Pitch;
    float Yaw;

} __attribute__((packed)) AttitudeActualData;

// Field information
// Field q1 information
// Field q2 information
// Field q3 information
// Field q4 information
// Field Roll information
// Field Pitch information
// Field Yaw information


// Generic interface functions
int32_t AttitudeActualInitialize();
UAVObjHandle AttitudeActualHandle();

#endif // ATTITUDEACTUAL_H

/**
 * @}
 * @}
 */