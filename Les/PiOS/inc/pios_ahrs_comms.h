#ifndef PIOS_AHRS_COMMS_H_INCLUDED
#define PIOS_AHRS_COMMS_H_INCLUDED

#ifdef IN_AHRS  //AHRS only
#include <stdint.h>

typedef void* UAVObjHandle;
typedef UAVObjHandle PIOS_AHRS_Handle;

#define UAVObjGetData(obj, data) PIOS_AHRS_GetData(obj, data)
#define UAVObjSetData(obj, data) PIOS_AHRS_SetData(obj, data)
#define UAVObjConnectCallback(obj, callback, mask) PIOS_AHRS_ConnectCallBack(obj,callback)

#else

#include "openpilot.h"
typedef void * PIOS_AHRS_Handle;

#endif

#include "attitudeactual.h"
#include "ahrssettings.h"
#include "attituderaw.h"
#include "attitudesettings.h"
#include "ahrsstatus.h"
#include "baroaltitude.h"
#include "gpsposition.h"
#include "positionactual.h"
#include "homelocation.h"
#include "ahrscalibration.h"


/*NOTE: As well as updating these two structures you need to update
pios_ahrs_comms.c and the OpenPilot comms module
The first object in the list has the highest priority.
The last has the lowest priority.

TODO: This is a bit ugly.
It would be nice to automate this with uavobjgenerator

*/

typedef struct
{
    AttitudeRawData AttitudeRaw;
    AttitudeActualData AttitudeActual;
    AHRSSettingsData AHRSSettings;
    AHRSCalibrationData AHRSCalibration;
    AttitudeSettingsData AttitudeSettings;
    AhrsStatusData AhrsStatus;
    BaroAltitudeData BaroAltitude;
    GPSPositionData GPSPosition;
    PositionActualData PositionActual;
    HomeLocationData HomeLocation;
}PIOS_AHRS_SharedMemory;

typedef union
{
    AttitudeRawData AttitudeRaw;
    AttitudeActualData AttitudeActual;
    AHRSSettingsData AHRSSettings;
    AHRSCalibrationData AHRSCalibration;
    AttitudeSettingsData AttitudeSettings;
    AhrsStatusData AhrsStatus;
    BaroAltitudeData BaroAltitude;
    GPSPositionData GPSPosition;
    PositionActualData PositionActual;
    HomeLocationData HomeLocation;
}__attribute__((packed)) PIOS_AHRS_SharedObject;

#define MAX_AHRS_OBJECTS 10 //this must correspond to the number of objects above


#define PROGRAM_DATA_SIZE 64

typedef struct{
    uint32_t address;
    uint8_t data[PROGRAM_DATA_SIZE];
}PIOS_AHRS_ProgramPacketData;

typedef enum {
    PIOS_AHRS_ERR_NONE,
    PIOS_AHRS_ERR_INIT = 1,
    PIOS_AHRS_ERR_CONNECT = 2,
    PIOS_AHRS_ERR_TRANSMIT = 4,
}PIOS_AHRS_ERROR;

/**
 * Event callback, this function is called when an event is invoked. The function
 * will be executed in the event task. The ev parameter should be copied if needed
 * after the function returns.
 */
typedef void (*PIOS_AHRS_EventCallback)(PIOS_AHRS_Handle obj);

void PIOS_AHRS_InitComms(void);
int32_t PIOS_AHRS_SetData(PIOS_AHRS_Handle obj, const void* dataIn);
int32_t PIOS_AHRS_GetData(PIOS_AHRS_Handle obj, void* dataOut);
PIOS_AHRS_SharedMemory * PIOS_AHRSGetMemory();
int32_t PIOS_AHRS_ConnectCallBack(PIOS_AHRS_Handle obj, PIOS_AHRS_EventCallback cb);

PIOS_AHRS_ERROR PIOS_AHRS_GetError();

#ifdef IN_AHRS  //slave only
uint32_t PIOS_AHRS_Poll();

#else //master only
void PIOS_AHRS_SendObjects(void);
int32_t PIOS_AHRS_SendProgram(const PIOS_AHRS_ProgramPacketData * data);
#endif

#endif //#ifndef PIOS_AHRS_COMMS_H_INCLUDED
