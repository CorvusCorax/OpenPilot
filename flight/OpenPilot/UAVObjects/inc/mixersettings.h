/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{ 
 * @addtogroup MixerSettings MixerSettings 
 * @brief Settings for the @ref ActuatorModule that controls the channel assignments for the mixer based on AircraftType
 *
 * Autogenerated files and functions for MixerSettings Object
 
 * @{ 
 *
 * @file       mixersettings.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Implementation of the MixerSettings object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: mixersettings.xml. 
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

#ifndef MIXERSETTINGS_H
#define MIXERSETTINGS_H

// Object constants
#define MIXERSETTINGS_OBJID 1336817486U
#define MIXERSETTINGS_NAME "MixerSettings"
#define MIXERSETTINGS_METANAME "MixerSettingsMeta"
#define MIXERSETTINGS_ISSINGLEINST 1
#define MIXERSETTINGS_ISSETTINGS 1
#define MIXERSETTINGS_NUMBYTES sizeof(MixerSettingsData)

// Object access macros
/**
 * @function MixerSettingsGet(dataOut)
 * @brief Populate a MixerSettingsData object
 * @param[out] dataOut 
 */
#define MixerSettingsGet(dataOut) UAVObjGetData(MixerSettingsHandle(), dataOut)
#define MixerSettingsSet(dataIn) UAVObjSetData(MixerSettingsHandle(), dataIn)
#define MixerSettingsInstGet(instId, dataOut) UAVObjGetInstanceData(MixerSettingsHandle(), instId, dataOut)
#define MixerSettingsInstSet(instId, dataIn) UAVObjSetInstanceData(MixerSettingsHandle(), instId, dataIn)
#define MixerSettingsConnectQueue(queue) UAVObjConnectQueue(MixerSettingsHandle(), queue, EV_MASK_ALL_UPDATES)
#define MixerSettingsConnectCallback(cb) UAVObjConnectCallback(MixerSettingsHandle(), cb, EV_MASK_ALL_UPDATES)
#define MixerSettingsCreateInstance() UAVObjCreateInstance(MixerSettingsHandle())
#define MixerSettingsRequestUpdate() UAVObjRequestUpdate(MixerSettingsHandle())
#define MixerSettingsRequestInstUpdate(instId) UAVObjRequestInstanceUpdate(MixerSettingsHandle(), instId)
#define MixerSettingsUpdated() UAVObjUpdated(MixerSettingsHandle())
#define MixerSettingsInstUpdated(instId) UAVObjUpdated(MixerSettingsHandle(), instId)
#define MixerSettingsGetMetadata(dataOut) UAVObjGetMetadata(MixerSettingsHandle(), dataOut)
#define MixerSettingsSetMetadata(dataIn) UAVObjSetMetadata(MixerSettingsHandle(), dataIn)
#define MixerSettingsReadOnly(dataIn) UAVObjReadOnly(MixerSettingsHandle())

// Object data
typedef struct {
    float MaxAccel;
    float FeedForward;
    float AccelTime;
    float DecelTime;
    float ThrottleCurve1[5];
    float ThrottleCurve2[5];
    uint8_t Mixer1Type;
    int8_t Mixer1Vector[5];
    uint8_t Mixer2Type;
    int8_t Mixer2Vector[5];
    uint8_t Mixer3Type;
    int8_t Mixer3Vector[5];
    uint8_t Mixer4Type;
    int8_t Mixer4Vector[5];
    uint8_t Mixer5Type;
    int8_t Mixer5Vector[5];
    uint8_t Mixer6Type;
    int8_t Mixer6Vector[5];
    uint8_t Mixer7Type;
    int8_t Mixer7Vector[5];
    uint8_t Mixer8Type;
    int8_t Mixer8Vector[5];

} __attribute__((packed)) MixerSettingsData;

// Field information
// Field MaxAccel information
// Field FeedForward information
// Field AccelTime information
// Field DecelTime information
// Field ThrottleCurve1 information
/* Array element names for field ThrottleCurve1 */
typedef enum { MIXERSETTINGS_THROTTLECURVE1_0=0, MIXERSETTINGS_THROTTLECURVE1_25=1, MIXERSETTINGS_THROTTLECURVE1_50=2, MIXERSETTINGS_THROTTLECURVE1_75=3, MIXERSETTINGS_THROTTLECURVE1_100=4 } MixerSettingsThrottleCurve1Elem;
/* Number of elements for field ThrottleCurve1 */
#define MIXERSETTINGS_THROTTLECURVE1_NUMELEM 5
// Field ThrottleCurve2 information
/* Array element names for field ThrottleCurve2 */
typedef enum { MIXERSETTINGS_THROTTLECURVE2_0=0, MIXERSETTINGS_THROTTLECURVE2_25=1, MIXERSETTINGS_THROTTLECURVE2_50=2, MIXERSETTINGS_THROTTLECURVE2_75=3, MIXERSETTINGS_THROTTLECURVE2_100=4 } MixerSettingsThrottleCurve2Elem;
/* Number of elements for field ThrottleCurve2 */
#define MIXERSETTINGS_THROTTLECURVE2_NUMELEM 5
// Field Mixer1Type information
/* Enumeration options for field Mixer1Type */
typedef enum { MIXERSETTINGS_MIXER1TYPE_DISABLED=0, MIXERSETTINGS_MIXER1TYPE_MOTOR=1, MIXERSETTINGS_MIXER1TYPE_SERVO=2 } MixerSettingsMixer1TypeOptions;
// Field Mixer1Vector information
/* Array element names for field Mixer1Vector */
typedef enum { MIXERSETTINGS_MIXER1VECTOR_THROTTLECURVE1=0, MIXERSETTINGS_MIXER1VECTOR_THROTTLECURVE2=1, MIXERSETTINGS_MIXER1VECTOR_ROLL=2, MIXERSETTINGS_MIXER1VECTOR_PITCH=3, MIXERSETTINGS_MIXER1VECTOR_YAW=4 } MixerSettingsMixer1VectorElem;
/* Number of elements for field Mixer1Vector */
#define MIXERSETTINGS_MIXER1VECTOR_NUMELEM 5
// Field Mixer2Type information
/* Enumeration options for field Mixer2Type */
typedef enum { MIXERSETTINGS_MIXER2TYPE_DISABLED=0, MIXERSETTINGS_MIXER2TYPE_MOTOR=1, MIXERSETTINGS_MIXER2TYPE_SERVO=2 } MixerSettingsMixer2TypeOptions;
// Field Mixer2Vector information
/* Array element names for field Mixer2Vector */
typedef enum { MIXERSETTINGS_MIXER2VECTOR_THROTTLECURVE1=0, MIXERSETTINGS_MIXER2VECTOR_THROTTLECURVE2=1, MIXERSETTINGS_MIXER2VECTOR_ROLL=2, MIXERSETTINGS_MIXER2VECTOR_PITCH=3, MIXERSETTINGS_MIXER2VECTOR_YAW=4 } MixerSettingsMixer2VectorElem;
/* Number of elements for field Mixer2Vector */
#define MIXERSETTINGS_MIXER2VECTOR_NUMELEM 5
// Field Mixer3Type information
/* Enumeration options for field Mixer3Type */
typedef enum { MIXERSETTINGS_MIXER3TYPE_DISABLED=0, MIXERSETTINGS_MIXER3TYPE_MOTOR=1, MIXERSETTINGS_MIXER3TYPE_SERVO=2 } MixerSettingsMixer3TypeOptions;
// Field Mixer3Vector information
/* Array element names for field Mixer3Vector */
typedef enum { MIXERSETTINGS_MIXER3VECTOR_THROTTLECURVE1=0, MIXERSETTINGS_MIXER3VECTOR_THROTTLECURVE2=1, MIXERSETTINGS_MIXER3VECTOR_ROLL=2, MIXERSETTINGS_MIXER3VECTOR_PITCH=3, MIXERSETTINGS_MIXER3VECTOR_YAW=4 } MixerSettingsMixer3VectorElem;
/* Number of elements for field Mixer3Vector */
#define MIXERSETTINGS_MIXER3VECTOR_NUMELEM 5
// Field Mixer4Type information
/* Enumeration options for field Mixer4Type */
typedef enum { MIXERSETTINGS_MIXER4TYPE_DISABLED=0, MIXERSETTINGS_MIXER4TYPE_MOTOR=1, MIXERSETTINGS_MIXER4TYPE_SERVO=2 } MixerSettingsMixer4TypeOptions;
// Field Mixer4Vector information
/* Array element names for field Mixer4Vector */
typedef enum { MIXERSETTINGS_MIXER4VECTOR_THROTTLECURVE1=0, MIXERSETTINGS_MIXER4VECTOR_THROTTLECURVE2=1, MIXERSETTINGS_MIXER4VECTOR_ROLL=2, MIXERSETTINGS_MIXER4VECTOR_PITCH=3, MIXERSETTINGS_MIXER4VECTOR_YAW=4 } MixerSettingsMixer4VectorElem;
/* Number of elements for field Mixer4Vector */
#define MIXERSETTINGS_MIXER4VECTOR_NUMELEM 5
// Field Mixer5Type information
/* Enumeration options for field Mixer5Type */
typedef enum { MIXERSETTINGS_MIXER5TYPE_DISABLED=0, MIXERSETTINGS_MIXER5TYPE_MOTOR=1, MIXERSETTINGS_MIXER5TYPE_SERVO=2 } MixerSettingsMixer5TypeOptions;
// Field Mixer5Vector information
/* Array element names for field Mixer5Vector */
typedef enum { MIXERSETTINGS_MIXER5VECTOR_THROTTLECURVE1=0, MIXERSETTINGS_MIXER5VECTOR_THROTTLECURVE2=1, MIXERSETTINGS_MIXER5VECTOR_ROLL=2, MIXERSETTINGS_MIXER5VECTOR_PITCH=3, MIXERSETTINGS_MIXER5VECTOR_YAW=4 } MixerSettingsMixer5VectorElem;
/* Number of elements for field Mixer5Vector */
#define MIXERSETTINGS_MIXER5VECTOR_NUMELEM 5
// Field Mixer6Type information
/* Enumeration options for field Mixer6Type */
typedef enum { MIXERSETTINGS_MIXER6TYPE_DISABLED=0, MIXERSETTINGS_MIXER6TYPE_MOTOR=1, MIXERSETTINGS_MIXER6TYPE_SERVO=2 } MixerSettingsMixer6TypeOptions;
// Field Mixer6Vector information
/* Array element names for field Mixer6Vector */
typedef enum { MIXERSETTINGS_MIXER6VECTOR_THROTTLECURVE1=0, MIXERSETTINGS_MIXER6VECTOR_THROTTLECURVE2=1, MIXERSETTINGS_MIXER6VECTOR_ROLL=2, MIXERSETTINGS_MIXER6VECTOR_PITCH=3, MIXERSETTINGS_MIXER6VECTOR_YAW=4 } MixerSettingsMixer6VectorElem;
/* Number of elements for field Mixer6Vector */
#define MIXERSETTINGS_MIXER6VECTOR_NUMELEM 5
// Field Mixer7Type information
/* Enumeration options for field Mixer7Type */
typedef enum { MIXERSETTINGS_MIXER7TYPE_DISABLED=0, MIXERSETTINGS_MIXER7TYPE_MOTOR=1, MIXERSETTINGS_MIXER7TYPE_SERVO=2 } MixerSettingsMixer7TypeOptions;
// Field Mixer7Vector information
/* Array element names for field Mixer7Vector */
typedef enum { MIXERSETTINGS_MIXER7VECTOR_THROTTLECURVE1=0, MIXERSETTINGS_MIXER7VECTOR_THROTTLECURVE2=1, MIXERSETTINGS_MIXER7VECTOR_ROLL=2, MIXERSETTINGS_MIXER7VECTOR_PITCH=3, MIXERSETTINGS_MIXER7VECTOR_YAW=4 } MixerSettingsMixer7VectorElem;
/* Number of elements for field Mixer7Vector */
#define MIXERSETTINGS_MIXER7VECTOR_NUMELEM 5
// Field Mixer8Type information
/* Enumeration options for field Mixer8Type */
typedef enum { MIXERSETTINGS_MIXER8TYPE_DISABLED=0, MIXERSETTINGS_MIXER8TYPE_MOTOR=1, MIXERSETTINGS_MIXER8TYPE_SERVO=2 } MixerSettingsMixer8TypeOptions;
// Field Mixer8Vector information
/* Array element names for field Mixer8Vector */
typedef enum { MIXERSETTINGS_MIXER8VECTOR_THROTTLECURVE1=0, MIXERSETTINGS_MIXER8VECTOR_THROTTLECURVE2=1, MIXERSETTINGS_MIXER8VECTOR_ROLL=2, MIXERSETTINGS_MIXER8VECTOR_PITCH=3, MIXERSETTINGS_MIXER8VECTOR_YAW=4 } MixerSettingsMixer8VectorElem;
/* Number of elements for field Mixer8Vector */
#define MIXERSETTINGS_MIXER8VECTOR_NUMELEM 5


// Generic interface functions
int32_t MixerSettingsInitialize();
UAVObjHandle MixerSettingsHandle();

#endif // MIXERSETTINGS_H

/**
 * @}
 * @}
 */
