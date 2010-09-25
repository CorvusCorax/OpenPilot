/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{
 * @addtogroup UAV Object Manager
 * @brief The core UAV Objects functions, most of which are wrappered by
 * autogenerated defines
 * @{
 *
 * @file       uavobjectmanager.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Include files of the uavobjectlist library
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

#ifndef UAVOBJECTMANAGER_H
#define UAVOBJECTMANAGER_H

#define UAVOBJ_ALL_INSTANCES 0xFFFF
#define UAVOBJ_MAX_INSTANCES 1000

typedef void* UAVObjHandle;

/**
 * Object update mode, used by multiple modules (e.g. telemetry and logger)
 */
typedef enum {
	UPDATEMODE_PERIODIC = 0, /** Automatically update object at periodic intervals */
	UPDATEMODE_ONCHANGE = 1, /** Only update object when its data changes */
	UPDATEMODE_MANUAL = 2,  /** Manually update object, by calling the updated() function */
	UPDATEMODE_NEVER = 3 /** Object is never updated */
} UAVObjUpdateMode;

/**
 * Object metadata, each object has a meta object that holds its metadata. The metadata define
 * properties for each object and can be used by multiple modules (e.g. telemetry and logger)
 */
typedef struct {
	uint8_t access; /** Defines the access level for the local transactions (readonly and readwrite) */
	uint8_t gcsAccess; /** Defines the access level for the local GCS transactions (readonly and readwrite), not used in the flight s/w */
	uint8_t telemetryAcked; /** Defines if an ack is required for the transactions of this object (1:acked, 0:not acked) */
	uint8_t telemetryUpdateMode; /** Update mode used by the telemetry module (UAVObjUpdateMode) */
	int32_t telemetryUpdatePeriod; /** Update period used by the telemetry module (only if telemetry mode is PERIODIC) */
	uint8_t gcsTelemetryAcked; /** Defines if an ack is required for the transactions of this object (1:acked, 0:not acked) */
	uint8_t gcsTelemetryUpdateMode; /** Update mode used by the GCS (UAVObjUpdateMode) */
	int32_t gcsTelemetryUpdatePeriod; /** Update period used by the GCS (only if telemetry mode is PERIODIC) */
	uint8_t loggingUpdateMode; /** Update mode used by the logging module (UAVObjUpdateMode) */
	uint32_t loggingUpdatePeriod; /** Update period used by the logging module (only if logging mode is PERIODIC) */
} __attribute__((packed)) UAVObjMetadata;

/**
 * Event types generated by the objects.
 */
typedef enum {
	EV_UNPACKED = 0x01, /** Object data updated by unpacking */
    EV_UPDATED = 0x02, /** Object data updated by changing the data structure */
    EV_UPDATED_MANUAL = 0x04, /** Object update event manually generated */
    EV_UPDATE_REQ = 0x08 /** Request to update object data */
} UAVObjEventType;

/**
 * Helper macros for event masks
 */
#define EV_MASK_ALL 0
#define EV_MASK_ALL_UPDATES (EV_UNPACKED | EV_UPDATED | EV_UPDATED_MANUAL)

/**
 * Access types
 */
typedef enum {
	ACCESS_READWRITE = 0,
	ACCESS_READONLY = 1
} UAVObjAccessType;

/**
 * Event message, this structure is sent in the event queue each time an event is generated
 */
typedef struct {
	UAVObjHandle obj;
	uint16_t instId;
	UAVObjEventType event;
} UAVObjEvent;

/**
 * Event callback, this function is called when an event is invoked. The function
 * will be executed in the event task. The ev parameter should be copied if needed
 * after the function returns.
 */
typedef void (*UAVObjEventCallback)(UAVObjEvent* ev);

/**
 * Callback used to initialize the object fields to their default values.
 */
typedef void (*UAVObjInitializeCallback)(UAVObjHandle obj, uint16_t instId);

/**
 * Event manager statistics
 */
typedef struct {
	uint32_t eventErrors;
} UAVObjStats;

int32_t UAVObjInitialize();
void UAVObjGetStats(UAVObjStats* statsOut);
void UAVObjClearStats();
UAVObjHandle UAVObjRegister(uint32_t id, const char* name, const char* metaName, int32_t isMetaobject,
		int32_t isSingleInstance, int32_t isSettings, uint32_t numBytes, UAVObjInitializeCallback initCb);
UAVObjHandle UAVObjGetByID(uint32_t id);
UAVObjHandle UAVObjGetByName(char* name);
uint32_t UAVObjGetID(UAVObjHandle obj);
const char* UAVObjGetName(UAVObjHandle obj);
uint32_t UAVObjGetNumBytes(UAVObjHandle obj);
uint16_t UAVObjGetNumInstances(UAVObjHandle obj);
UAVObjHandle UAVObjGetLinkedObj(UAVObjHandle obj);
uint16_t UAVObjCreateInstance(UAVObjHandle obj);
int32_t UAVObjIsSingleInstance(UAVObjHandle obj);
int32_t UAVObjIsMetaobject(UAVObjHandle obj);
int32_t UAVObjIsSettings(UAVObjHandle obj);
int32_t UAVObjUnpack(UAVObjHandle obj, uint16_t instId, const uint8_t* dataIn);
int32_t UAVObjPack(UAVObjHandle obj, uint16_t instId, uint8_t* dataOut);
int32_t UAVObjSave(UAVObjHandle obj, uint16_t instId);
int32_t UAVObjLoad(UAVObjHandle obj, uint16_t instId);
int32_t UAVObjDelete(UAVObjHandle obj, uint16_t instId);
int32_t UAVObjSaveToFile(UAVObjHandle obj, uint16_t instId, FILEINFO* file);
UAVObjHandle UAVObjLoadFromFile(FILEINFO* file);
int32_t UAVObjSaveSettings();
int32_t UAVObjLoadSettings();
int32_t UAVObjDeleteSettings();
int32_t UAVObjSaveMetaobjects();
int32_t UAVObjLoadMetaobjects();
int32_t UAVObjDeleteMetaobjects();
int32_t UAVObjSetData(UAVObjHandle obj, const void* dataIn);
int32_t UAVObjGetData(UAVObjHandle obj, void* dataOut);
int32_t UAVObjSetInstanceData(UAVObjHandle obj, uint16_t instId, const void* dataIn);
int32_t UAVObjGetInstanceData(UAVObjHandle obj, uint16_t instId, void* dataOut);
int32_t UAVObjSetMetadata(UAVObjHandle obj, const UAVObjMetadata* dataIn);
int32_t UAVObjGetMetadata(UAVObjHandle obj, UAVObjMetadata* dataOut);
int8_t UAVObjReadOnly(UAVObjHandle obj);
int32_t UAVObjConnectQueue(UAVObjHandle obj, xQueueHandle queue, int32_t eventMask);
int32_t UAVObjDisconnectQueue(UAVObjHandle obj, xQueueHandle queue);
int32_t UAVObjConnectCallback(UAVObjHandle obj, UAVObjEventCallback cb, int32_t eventMask);
int32_t UAVObjDisconnectCallback(UAVObjHandle obj, UAVObjEventCallback cb);
void UAVObjRequestUpdate(UAVObjHandle obj);
void UAVObjRequestInstanceUpdate(UAVObjHandle obj, uint16_t instId);
void UAVObjUpdated(UAVObjHandle obj);
void UAVObjInstanceUpdated(UAVObjHandle obj, uint16_t instId);
void UAVObjIterate(void (*iterator)(UAVObjHandle obj));

#endif // UAVOBJECTMANAGER_H

/**
 * @}
 * @}
 */
