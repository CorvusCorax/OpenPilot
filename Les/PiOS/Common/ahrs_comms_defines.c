#include "pios_ahrs_comms.h"

#ifdef IN_AHRS
#include <string.h>
#include <stdbool.h>
#include "pios_debug.h"
#include "pios_spi.h"
#include "pios_irq.h"

#include "STM32103CB_AHRS.h"
#endif


typedef enum {COMMS_NULL, COMMS_OBJECT, COMMS_PROGRAM} COMMSCOMMAND;
#define MAX_UPDATE_OBJECTS 2 //The maximum number of objects that can be updated in one cycle
#define NO_OBJECT 0xff

#define ACK_LATENCY 4  //Number of transmissions + 1 before we expect to see the data acknowledge

typedef struct {
	PIOS_AHRS_Handle handle;
    int size;
} ObjectHandleSize;

typedef struct {
    uint8_t done;
    uint8_t index;
    PIOS_AHRS_SharedObject object;
} ObjectPacketData;

typedef struct {
    COMMSCOMMAND command;
    union { //allow for expansion to other packet types.
        ObjectPacketData objects[MAX_UPDATE_OBJECTS];
        PIOS_AHRS_ProgramPacketData program;
    };
} CommsDataPacket;


static int GetHandleIndex(PIOS_AHRS_Handle obj);
static void FillObjectPacket();
static void CommsCallback(uint8_t crc_ok, uint8_t crc_val);
static void SetObjectDirty(const int idx);
static void SetError(AhrsStatusCommErrorsElem element);
static void HandleObjectPacket();
static void HandleRxPacket();
static void PollEvents();
#ifndef IN_AHRS
static void SendPacket(void);
#endif


static ObjectHandleSize objectHandles[MAX_AHRS_OBJECTS] = {{0,0}};
static PIOS_AHRS_SharedMemory sharedMemory;
static CommsDataPacket txPacket;
static CommsDataPacket rxPacket;
static unsigned int dirtyObjects[MAX_AHRS_OBJECTS];
static int errors;
static PIOS_AHRS_EventCallback objCallbacks[MAX_AHRS_OBJECTS];
static bool callbackPending[MAX_AHRS_OBJECTS];


#ifdef IN_AHRS
//slightly hacky - implement our own subset of UAVObject functionality
#define CREATEHANDLE(hnd) \
    PIOS_AHRS_Handle hnd##Handle() {return (&sharedMemory.hnd);}


CREATEHANDLE (AttitudeRaw);
CREATEHANDLE (AttitudeActual);
CREATEHANDLE (AHRSSettings);
CREATEHANDLE (AHRSCalibration);
CREATEHANDLE (AttitudeSettings);
CREATEHANDLE (AhrsStatus);
CREATEHANDLE (BaroAltitude);
CREATEHANDLE (GPSPosition);
CREATEHANDLE (PositionActual);
CREATEHANDLE (HomeLocation);
#endif

#define ADDHANDLE(idx,hnd) {int n = idx;\
	objectHandles[n].handle = &sharedMemory.hnd;\
	objectHandles[n].size = sizeof(sharedMemory.hnd);}
void PIOS_AHRS_InitComms(void)
{
    int idx = 0;
    errors = 0;
    ADDHANDLE(idx++, AttitudeRaw);
    ADDHANDLE(idx++, AttitudeActual);
    ADDHANDLE(idx++, AHRSSettings);
    ADDHANDLE(idx++, AHRSCalibration);
    ADDHANDLE(idx++, AttitudeSettings);
    ADDHANDLE(idx++, AhrsStatus);
    ADDHANDLE(idx++, BaroAltitude);
    ADDHANDLE(idx++, GPSPosition);
    ADDHANDLE(idx++, PositionActual);
    ADDHANDLE(idx++, HomeLocation);
    if(idx != MAX_AHRS_OBJECTS) {
        errors |= PIOS_AHRS_ERR_INIT;
        PIOS_DEBUG_Assert(0);
    }
    memset(&sharedMemory,0,sizeof(PIOS_AHRS_SharedMemory));
    memset(objCallbacks,0,sizeof(objCallbacks));
    memset(callbackPending,0,sizeof(callbackPending));
    for(int ct=0; ct< MAX_AHRS_OBJECTS; ct++) {
        dirtyObjects[ct] = 0;
    }
    txPacket.command = COMMS_NULL;
    rxPacket.command = COMMS_NULL;

#ifdef IN_AHRS
    PIOS_SPI_Init();
#else
    PIOS_SPI_SetClockSpeed(PIOS_OPAHRS_SPI, PIOS_SPI_PRESCALER_16); //72MHz/16 = 4.5MHz
#endif

}


PIOS_AHRS_SharedMemory * PIOS_AHRSGetMemory()
{
    return(&sharedMemory);
}

int GetHandleIndex(PIOS_AHRS_Handle obj)
{
    if(objectHandles[0].handle == NULL) { //Oops - we haven't been initialised!
        PIOS_DEBUG_Assert(0);
        return(-1);
    }
    for(int ct=0; ct<MAX_AHRS_OBJECTS; ct++) {
        if(objectHandles[ct].handle == obj) {
            return(ct);
        }
    }
    PIOS_DEBUG_Assert(0); //invalid handle
    return(-1);
}

int32_t PIOS_AHRS_SetData(PIOS_AHRS_Handle obj, const void* dataIn)
{
    int idx = GetHandleIndex(obj);
    if(idx < 0) {
        PIOS_DEBUG_Assert(0); //invalid handle
        return(-1);
    }
    int size = objectHandles[idx].size;
    if(memcmp(obj,dataIn,size) == 0) { //nothing to do
        return(0);
    }
    memcpy(obj,dataIn,size);
    SetObjectDirty(idx);
    return(0);
}

void SetObjectDirty(const int idx)
{
    if(idx <0 || idx >= MAX_AHRS_OBJECTS) {
        return;
    }
    dirtyObjects[idx] = ACK_LATENCY;
}

void SetError(AhrsStatusCommErrorsElem element)
{
    sharedMemory.AhrsStatus.CommErrors[element]++;
    SetObjectDirty(GetHandleIndex(&sharedMemory.AhrsStatus));
}


int32_t PIOS_AHRS_GetData(PIOS_AHRS_Handle obj, void* dataOut)
{
    int idx = GetHandleIndex(obj);
    if(idx < 0) {
        PIOS_DEBUG_Assert(0); //invalid handle
        return(-1);
    }
    int size = objectHandles[idx].size;
    if(size <= 0 || dataOut == NULL) {
        PIOS_DEBUG_Assert(0); //invalid handle
        return(-1);
    }
    memcpy(dataOut,obj,size);
    return(0);
}

void FillObjectPacket()
{
    txPacket.command = COMMS_OBJECT;
    int obj = 0;
    for(int ct=0; ct< MAX_UPDATE_OBJECTS; ct++) {
        txPacket.objects[ct].index = NO_OBJECT;
        for(; obj < MAX_AHRS_OBJECTS; obj++) {
            if(dirtyObjects[obj] > 0) {
                if(dirtyObjects[obj] == ACK_LATENCY) {
                    dirtyObjects[obj]--;
                    txPacket.objects[ct].index = obj;
                    memcpy(&txPacket.objects[ct].object, objectHandles[obj].handle, objectHandles[obj].size);
                    break;
                } else {
                    dirtyObjects[obj] --;
                    if(dirtyObjects[obj] == 0) { //timed out
                        dirtyObjects[obj] = ACK_LATENCY;
                    }
                }
            }
        }
    }
}


void HandleRxPacket()
{
    switch(rxPacket.command) {
    case COMMS_NULL:
        break;

    case COMMS_OBJECT:
        HandleObjectPacket();
        break;

    case COMMS_PROGRAM: //TODO: programming protocol
        break;

    default:
        SetError(AHRSSTATUS_COMMERRORS_ALGORITHM);
    }
}

void HandleObjectPacket()
{
    for(int ct=0; ct< MAX_UPDATE_OBJECTS; ct++) {

        //Flag objects that have been successfully received at the other end
        uint8_t obj = rxPacket.objects[ct].done;
        if(obj < MAX_AHRS_OBJECTS) {

            if(dirtyObjects[obj] == 1) { //this ack is the correct one for the last send
                dirtyObjects[obj] = 0;
            }
        }

        txPacket.objects[ct].done = NO_OBJECT;
        obj = rxPacket.objects[ct].index;
        if(obj < MAX_AHRS_OBJECTS) {
            memcpy(objectHandles[obj].handle, &rxPacket.objects[ct].object, objectHandles[obj].size);
            txPacket.objects[ct].done = obj;
            callbackPending[obj] = true;
        } else {
            if(obj != NO_OBJECT) {
                SetError(AHRSSTATUS_COMMERRORS_ATTITUDERAW);
            }
        }
    }
#ifdef IN_AHRS
    FillObjectPacket(); //ready for the next frame
#endif
}


int32_t PIOS_AHRS_ConnectCallBack(PIOS_AHRS_Handle obj, PIOS_AHRS_EventCallback cb)
{
    int idx = GetHandleIndex(obj);
    if(idx <0) {
        return (-1);
    }
    objCallbacks[idx] = cb;
    return(0);
}

PIOS_AHRS_ERROR PIOS_AHRS_GetError()
{
    static int count = 0;
    PIOS_AHRS_ERROR ret = errors;
    if(count++ >= 100) {
        errors = 0;
    }
    return(ret);
}


void CommsCallback(uint8_t crc_ok, uint8_t crc_val)
{
#ifndef IN_AHRS
    PIOS_SPI_RC_PinSet(PIOS_OPAHRS_SPI, 1); //signal the end of the transfer
#endif
    txPacket.command = COMMS_NULL; //we must send something so default to null
    if(crc_ok) {
        HandleRxPacket();
    } else {
        SetError(AHRSSTATUS_COMMERRORS_UPDATE);
    }
#ifdef IN_AHRS
    /*queue next frame
        If PIOS_SPI_TransferBlock() fails for any reason, comms will stop working.
        In that case, PIOS_AHRS_Poll() should kick start things again.
    */
    PIOS_SPI_TransferBlock(PIOS_SPI_OP, (uint8_t *)&txPacket,
    (uint8_t *)&rxPacket, sizeof(CommsDataPacket), &CommsCallback);
#endif
}


void PollEvents(void)
{
    for(int obj = 0; obj < MAX_AHRS_OBJECTS; obj++) {
        if(objCallbacks[obj]) {
            PIOS_IRQ_Disable();
            if( callbackPending[obj]) {
                callbackPending[obj] = false;
                PIOS_IRQ_Enable();
                objCallbacks[obj](objectHandles[obj].handle);
            } else {
                PIOS_IRQ_Enable();
            }
        }
    }
}


#ifdef IN_AHRS
uint32_t PIOS_AHRS_Poll()
{
    static uint32_t kickCount = 0;
    PollEvents();
    if(PIOS_SPI_Busy(PIOS_SPI_OP) != 0) { //Everything is working correctly
        return(kickCount);
    }
    kickCount++;
//comms have broken down - try kick starting it.
    txPacket.command = COMMS_NULL; //we must send something so default to null
    PIOS_SPI_TransferBlock(PIOS_SPI_OP, (uint8_t *)&txPacket,
    (uint8_t *)&rxPacket, sizeof(CommsDataPacket), &CommsCallback);
    return(kickCount);
}

#else



void PIOS_AHRS_SendObjects(void)
{
    PollEvents();
    FillObjectPacket();
    SendPacket();
}


int32_t PIOS_AHRS_SendProgram(const PIOS_AHRS_ProgramPacketData * data)
{
//TODO: Create a program packet, send it and wait for ack.
    return(-1);
}


void SendPacket(void)
{
    PIOS_SPI_RC_PinSet(PIOS_OPAHRS_SPI, 0);
    int32_t rc = PIOS_SPI_TransferBlock(PIOS_OPAHRS_SPI, (uint8_t *)&txPacket,
    (uint8_t *)&rxPacket, sizeof(CommsDataPacket), &CommsCallback);
    if(rc !=0) {
        errors |= PIOS_AHRS_ERR_TRANSMIT;
    }
}
#endif
