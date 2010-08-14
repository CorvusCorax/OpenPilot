/**
 ******************************************************************************
 *
 * @file       attituderaw.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Implementation of the AttitudeRaw object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: attituderaw.xml. 
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

#ifndef ATTITUDERAW_H
#define ATTITUDERAW_H

// Object constants
#define ATTITUDERAW_OBJID 4179445416U
#define ATTITUDERAW_NAME "AttitudeRaw"
#define ATTITUDERAW_METANAME "AttitudeRawMeta"
#define ATTITUDERAW_ISSINGLEINST 1
#define ATTITUDERAW_ISSETTINGS 0
#define ATTITUDERAW_NUMBYTES sizeof(AttitudeRawData)

// Object access macros
#define AttitudeRawGet(dataOut) UAVObjGetData(AttitudeRawHandle(), dataOut)
#define AttitudeRawSet(dataIn) UAVObjSetData(AttitudeRawHandle(), dataIn)
#define AttitudeRawInstGet(instId, dataOut) UAVObjGetInstanceData(AttitudeRawHandle(), instId, dataOut)
#define AttitudeRawInstSet(instId, dataIn) UAVObjSetInstanceData(AttitudeRawHandle(), instId, dataIn)
#define AttitudeRawConnectQueue(queue) UAVObjConnectQueue(AttitudeRawHandle(), queue, EV_MASK_ALL_UPDATES)
#define AttitudeRawConnectCallback(cb) UAVObjConnectCallback(AttitudeRawHandle(), cb, EV_MASK_ALL_UPDATES)
#define AttitudeRawCreateInstance() UAVObjCreateInstance(AttitudeRawHandle())
#define AttitudeRawRequestUpdate() UAVObjRequestUpdate(AttitudeRawHandle())
#define AttitudeRawRequestInstUpdate(instId) UAVObjRequestInstanceUpdate(AttitudeRawHandle(), instId)
#define AttitudeRawUpdated() UAVObjUpdated(AttitudeRawHandle())
#define AttitudeRawInstUpdated(instId) UAVObjUpdated(AttitudeRawHandle(), instId)
#define AttitudeRawGetMetadata(dataOut) UAVObjGetMetadata(AttitudeRawHandle(), dataOut)
#define AttitudeRawSetMetadata(dataIn) UAVObjSetMetadata(AttitudeRawHandle(), dataIn)

// Object data
typedef struct {
    int16_t magnetometers[3];
    uint16_t gyros[3];
    uint16_t gyrotemp[2];
    uint16_t accelerometers[3];

} __attribute__((packed)) AttitudeRawData;

// Field information
// Field magnetometers information
/* Array element names for field magnetometers */
typedef enum { ATTITUDERAW_MAGNETOMETERS_X=0, ATTITUDERAW_MAGNETOMETERS_Y=1, ATTITUDERAW_MAGNETOMETERS_Z=2 } AttitudeRawmagnetometersElem;
/* Number of elements for field magnetometers */
#define ATTITUDERAW_MAGNETOMETERS_NUMELEM 3
// Field gyros information
/* Array element names for field gyros */
typedef enum { ATTITUDERAW_GYROS_X=0, ATTITUDERAW_GYROS_Y=1, ATTITUDERAW_GYROS_Z=2 } AttitudeRawgyrosElem;
/* Number of elements for field gyros */
#define ATTITUDERAW_GYROS_NUMELEM 3
// Field gyrotemp information
/* Array element names for field gyrotemp */
typedef enum { ATTITUDERAW_GYROTEMP_XY=0, ATTITUDERAW_GYROTEMP_Z=1 } AttitudeRawgyrotempElem;
/* Number of elements for field gyrotemp */
#define ATTITUDERAW_GYROTEMP_NUMELEM 2
// Field accelerometers information
/* Array element names for field accelerometers */
typedef enum { ATTITUDERAW_ACCELEROMETERS_X=0, ATTITUDERAW_ACCELEROMETERS_Y=1, ATTITUDERAW_ACCELEROMETERS_Z=2 } AttitudeRawaccelerometersElem;
/* Number of elements for field accelerometers */
#define ATTITUDERAW_ACCELEROMETERS_NUMELEM 3


// Generic interface functions
int32_t AttitudeRawInitialize();
UAVObjHandle AttitudeRawHandle();

#endif // ATTITUDERAW_H
