/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{ 
 * @addtogroup ExampleObject1 ExampleObject1 
 * @brief Example object
 *
 * Autogenerated files and functions for ExampleObject1 Object
 
 * @{ 
 *
 * @file       exampleobject1.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Implementation of the ExampleObject1 object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: exampleobject1.xml. 
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

#ifndef EXAMPLEOBJECT1_H
#define EXAMPLEOBJECT1_H

// Object constants
#define EXAMPLEOBJECT1_OBJID 3852936276U
#define EXAMPLEOBJECT1_NAME "ExampleObject1"
#define EXAMPLEOBJECT1_METANAME "ExampleObject1Meta"
#define EXAMPLEOBJECT1_ISSINGLEINST 0
#define EXAMPLEOBJECT1_ISSETTINGS 0
#define EXAMPLEOBJECT1_NUMBYTES sizeof(ExampleObject1Data)

// Object access macros
/**
 * @function ExampleObject1Get(dataOut)
 * @brief Populate a ExampleObject1Data object
 * @param[out] dataOut 
 */
#define ExampleObject1Get(dataOut) UAVObjGetData(ExampleObject1Handle(), dataOut)
#define ExampleObject1Set(dataIn) UAVObjSetData(ExampleObject1Handle(), dataIn)
#define ExampleObject1InstGet(instId, dataOut) UAVObjGetInstanceData(ExampleObject1Handle(), instId, dataOut)
#define ExampleObject1InstSet(instId, dataIn) UAVObjSetInstanceData(ExampleObject1Handle(), instId, dataIn)
#define ExampleObject1ConnectQueue(queue) UAVObjConnectQueue(ExampleObject1Handle(), queue, EV_MASK_ALL_UPDATES)
#define ExampleObject1ConnectCallback(cb) UAVObjConnectCallback(ExampleObject1Handle(), cb, EV_MASK_ALL_UPDATES)
#define ExampleObject1CreateInstance() UAVObjCreateInstance(ExampleObject1Handle())
#define ExampleObject1RequestUpdate() UAVObjRequestUpdate(ExampleObject1Handle())
#define ExampleObject1RequestInstUpdate(instId) UAVObjRequestInstanceUpdate(ExampleObject1Handle(), instId)
#define ExampleObject1Updated() UAVObjUpdated(ExampleObject1Handle())
#define ExampleObject1InstUpdated(instId) UAVObjUpdated(ExampleObject1Handle(), instId)
#define ExampleObject1GetMetadata(dataOut) UAVObjGetMetadata(ExampleObject1Handle(), dataOut)
#define ExampleObject1SetMetadata(dataIn) UAVObjSetMetadata(ExampleObject1Handle(), dataIn)
#define ExampleObject1ReadOnly(dataIn) UAVObjReadOnly(ExampleObject1Handle())

// Object data
typedef struct {
    int8_t Field1;
    int16_t Field2;
    int32_t Field3;
    float Field4[4];
    uint8_t Field5;
    uint16_t Field6;
    uint32_t Field7;
    uint8_t Field8;

} __attribute__((packed)) ExampleObject1Data;

// Field information
// Field Field1 information
// Field Field2 information
// Field Field3 information
// Field Field4 information
/* Number of elements for field Field4 */
#define EXAMPLEOBJECT1_FIELD4_NUMELEM 4
// Field Field5 information
// Field Field6 information
// Field Field7 information
// Field Field8 information
/* Enumeration options for field Field8 */
typedef enum { EXAMPLEOBJECT1_FIELD8_OPTION1=0, EXAMPLEOBJECT1_FIELD8_OPTION2=1 } ExampleObject1Field8Options;


// Generic interface functions
int32_t ExampleObject1Initialize();
UAVObjHandle ExampleObject1Handle();

#endif // EXAMPLEOBJECT1_H

/**
 * @}
 * @}
 */
