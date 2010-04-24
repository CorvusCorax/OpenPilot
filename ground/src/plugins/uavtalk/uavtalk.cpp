/**
 ******************************************************************************
 *
 * @file       uavtalk.cpp
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 *             Parts by Nokia Corporation (qt-info@nokia.com) Copyright (C) 2009.
 * @brief      
 * @see        The GNU Public License (GPL) Version 3
 * @defgroup   
 * @{
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
#include "uavtalk.h"
#include <QtEndian>

/**
 * Constructor
 */
UAVTalk::UAVTalk(QIODevice* iodev, UAVObjectManager* objMngr)
{
    io = iodev;
    this->objMngr = objMngr;
    rxState = STATE_SYNC;
    mutex = new QMutex(QMutex::Recursive);
    respObj = NULL;
    memset(&stats, 0, sizeof(ComStats));
    connect(io, SIGNAL(readyRead()), this, SLOT(processInputStream()));
}

/**
 * Reset the statistics counters
 */
void UAVTalk::resetStats()
{
    QMutexLocker locker(mutex);
    memset(&stats, 0, sizeof(ComStats));
}

/**
 * Get the statistics counters
 */
UAVTalk::ComStats UAVTalk::getStats()
{
    QMutexLocker locker(mutex);
    return stats;
}

/**
 * Called each time there are data in the input buffer
 */
void UAVTalk::processInputStream()
{
    quint8 tmp;

    while (io->bytesAvailable() > 0)
    {
        io->read((char*)&tmp, 1);
        processInputByte(tmp);
    }
}

/**
 * Request an update for the specified object, on success the object data would have been
 * updated by the GCS.
 * \param[in] obj Object to update
 * \param[in] allInstances If set true then all instances will be updated
 * \return Success (true), Failure (false)
 */
bool UAVTalk::sendObjectRequest(UAVObject* obj, bool allInstances)
{
    QMutexLocker locker(mutex);
    return objectTransaction(obj, TYPE_OBJ_REQ, allInstances);
}

/**
 * Send the specified object through the telemetry link.
 * \param[in] obj Object to send
 * \param[in] acked Selects if an ack is required
 * \param[in] allInstances If set true then all instances will be updated
 * \return Success (true), Failure (false)
 */
bool UAVTalk::sendObject(UAVObject* obj, bool acked, bool allInstances)
{
    QMutexLocker locker(mutex);
    if (acked)
    {
        return objectTransaction(obj, TYPE_OBJ_ACK, allInstances);
    }
    else
    {
        return objectTransaction(obj, TYPE_OBJ, allInstances);
    }
}

/**
 * Cancel a pending transaction
 */
void UAVTalk::cancelTransaction()
{
    QMutexLocker locker(mutex);
    respObj = NULL;
}

/**
 * Execute the requested transaction on an object.
 * \param[in] obj Object
 * \param[in] type Transaction type
 * 			  TYPE_OBJ: send object,
 * 			  TYPE_OBJ_REQ: request object update
 * 			  TYPE_OBJ_ACK: send object with an ack
 * \param[in] allInstances If set true then all instances will be updated
 * \return Success (true), Failure (false)
 */
bool UAVTalk::objectTransaction(UAVObject* obj, quint8 type, bool allInstances)
{
    // Send object depending on if a response is needed
    if (type == TYPE_OBJ_ACK || type == TYPE_OBJ_REQ)
    {
        if ( transmitObject(obj, type, allInstances) )
        {
            respObj = obj;
            respAllInstances = allInstances;    
            return true;
        }
        else
        {
            return false;
        }
    }
    else if (type == TYPE_OBJ)
    {
        return transmitObject(obj, TYPE_OBJ, allInstances);
    }
    else
    {
        return false;
    }
}

/**
 * Process an byte from the telemetry stream.
 * \param[in] rxbyte Received byte
 * \return Success (true), Failure (false)
 */
bool UAVTalk::processInputByte(quint8 rxbyte)
{
    // Update stats
    ++stats.rxBytes;
    // Receive state machine
    switch (rxState) {
    case STATE_SYNC:
        if ((rxbyte & TYPE_MASK) == TYPE_VER )
        {
            rxCS = rxbyte;
            rxType = rxbyte;
            rxState = STATE_OBJID;
            rxCount = 0;
        }
        break;
    case STATE_OBJID:
        rxTmpBuffer[rxCount++] = rxbyte;
        if (rxCount == 4)
        {
            // Search for object, if not found reset state machine
            rxObjId = (qint32)qFromLittleEndian<quint32>(rxTmpBuffer);
            UAVObject* rxObj = objMngr->getObject(rxObjId);
            if (rxObj == NULL)
            {
                rxState = STATE_SYNC;
                ++stats.rxErrors;
            }
            else
            {
                // Update checksum
                rxCS = updateChecksum(rxCS, rxTmpBuffer, 4);
                // Determine data length
                if (rxType == TYPE_OBJ_REQ || rxType == TYPE_ACK)
                {
                    rxLength = 0;
                }
                else
                {
                    rxLength = rxObj->getNumBytes();
                }
                // Check length and determine next state
                if (rxLength >= MAX_PAYLOAD_LENGTH)
                {
                    rxState = STATE_SYNC;
                    ++stats.rxErrors;
                }
                else
                {
                    // Check if this is a single instance object (i.e. if the instance ID field is coming next)
                    if ( rxObj->isSingleInstance() )
                    {
                        // If there is a payload get it, otherwise receive checksum
                        if (rxLength > 0)
                        {
                            rxState = STATE_DATA;
                        }
                        else
                        {
                            rxState = STATE_CS;
                        }
                        rxInstId = 0;
                        rxCount = 0;
                    }
                    else
                    {
                        rxState = STATE_INSTID;
                        rxCount = 0;
                    }
                }
            }
        }
        break;
    case STATE_INSTID:
        rxTmpBuffer[rxCount++] = rxbyte;
        if (rxCount == 2)
        {
            rxInstId = (qint16)qFromLittleEndian<quint16>(rxTmpBuffer);
            rxCS = updateChecksum(rxCS, rxTmpBuffer, 2);
            rxCount = 0;
            // If there is a payload get it, otherwise receive checksum
            if (rxLength > 0)
            {
                rxState = STATE_DATA;
            }
            else
            {
                rxState = STATE_CS;
            }
        }
        break;
    case STATE_DATA:
        rxBuffer[rxCount++] = rxbyte;
        if (rxCount == rxLength)
        {
            rxCS = updateChecksum(rxCS, rxBuffer, rxLength);
            rxState = STATE_CS;
            rxCount = 0;
        }
        break;
    case STATE_CS:
        rxTmpBuffer[rxCount++] = rxbyte;
        if (rxCount == 2)
        {
            rxCSPacket = (qint16)qFromLittleEndian<quint16>(rxTmpBuffer);
            if (rxCS == rxCSPacket)
            {
                mutex->lock();
                receiveObject(rxType, rxObjId, rxInstId, rxBuffer, rxLength);
                stats.rxObjectBytes += rxLength;
                ++stats.rxObjects;
                mutex->unlock();
            }
            else
            {
                ++stats.rxErrors;
            }
            rxState = STATE_SYNC;
        }
        break;
    default:
        rxState = STATE_SYNC;
        ++stats.rxErrors;
    }

    // Done
    return true;
}

/**
 * Receive an object. This function process objects received through the telemetry stream.
 * \param[in] type Type of received message (TYPE_OBJ, TYPE_OBJ_REQ, TYPE_OBJ_ACK, TYPE_ACK)
 * \param[in] obj Handle of the received object
 * \param[in] instId The instance ID of UAVOBJ_ALL_INSTANCES for all instances.
 * \param[in] data Data buffer
 * \param[in] length Buffer length
 * \return Success (true), Failure (false)
 */
bool UAVTalk::receiveObject(quint8 type, quint32 objId, quint16 instId, quint8* data, qint32 length)
{
    UAVObject* obj = NULL;
    bool error = false;
    bool allInstances = (instId == ALL_INSTANCES? true : false);

    // Process message type
    switch (type) {
    case TYPE_OBJ:
        // All instances, not allowed for OBJ messages
        if (!allInstances)
        {
            // Get object and update its data
            obj = updateObject(objId, instId, data);
            // Check if an ack is pending
            if ( obj != NULL )
            {
                updateAck(obj);
            }
            else
            {
                error = true;
            }
        }
        else
        {
            error = true;
        }
        break;
    case TYPE_OBJ_ACK:
        // All instances, not allowed for OBJ_ACK messages
        if (!allInstances)
        {
            // Get object and update its data
            obj = updateObject(objId, instId, data);
            // Transmit ACK
            if ( obj != NULL )
            {
               transmitObject(obj, TYPE_ACK, false);
            }
            else
            {
                error = true;
            }
        }
        else
        {
            error = true;
        }
        break;
    case TYPE_OBJ_REQ:
        // Get object, if all instances are requested get instance 0 of the object
        if (allInstances)
        {
            obj = objMngr->getObject(objId);
        }
        else
        {
            obj = objMngr->getObject(objId, instId);
        }
        // If object was found transmit it
        if (obj != NULL)
        {
            transmitObject(obj, TYPE_OBJ, allInstances);
        }
        else
        {
            error = true;
        }
        break;
    case TYPE_ACK:
        // All instances, not allowed for ACK messages
        if (!allInstances)
        {
            // Get object
            obj = objMngr->getObject(objId, instId);
            // Check if an ack is pending
            if (obj != NULL)
            {
                updateAck(obj);
            }
            else
            {
                error = true;
            }
        }
        break;
    default:
        error = true;
    }
    // Done
    return !error;
}

/**
 * Update the data of an object from a byte array (unpack).
 * If the object instance could not be found in the list, then a
 * new one is created.
 */
UAVObject* UAVTalk::updateObject(quint32 objId, quint16 instId, quint8* data)
{
    // Get object
    UAVObject* obj = objMngr->getObject(objId, instId);
    // If the instance does not exist create it
    if (obj == NULL)
    {
        // Get the object type
        UAVObject* tobj = objMngr->getObject(objId);
        if (tobj == NULL)
        {
            return NULL;
        }
        // Make sure this is a data object
        UAVDataObject* dobj = dynamic_cast<UAVDataObject*>(tobj);
        if (dobj == NULL)
        {
            return NULL;
        }
        // Create a new instance, unpack and register
        UAVDataObject* instobj = dobj->clone(instId);
        if ( !objMngr->registerObject(instobj) )
        {
            return NULL;
        }
        instobj->unpack(data);
        return instobj;
    }
    else
    {
        // Unpack data into object instance
        obj->unpack(data);
        return obj;
    }
}

/**
 * Check if a transaction is pending and if yes complete it.
 */
void UAVTalk::updateAck(UAVObject* obj)
{
    if (respObj != NULL && respObj->getObjID() == obj->getObjID() && (respObj->getInstID() == obj->getInstID() || respAllInstances))
    {
        respObj = NULL;
        emit transactionCompleted(obj);
    }
}

/**
 * Send an object through the telemetry link.
 * \param[in] obj Object to send
 * \param[in] type Transaction type
 * \param[in] allInstances True is all instances of the object are to be sent
 * \return Success (true), Failure (false)
 */
bool UAVTalk::transmitObject(UAVObject* obj, quint8 type, bool allInstances)
{   
    // If all instances are requested on a single instance object it is an error
    if (allInstances && obj->isSingleInstance())
    {
        allInstances = false;
    }

    // Process message type
    if ( type == TYPE_OBJ || type == TYPE_OBJ_ACK )
    {
        if (allInstances)
        {
            // Get number of instances
            quint32 numInst = objMngr->getNumInstances(obj->getObjID());
            // Send all instances
            for (quint32 instId = 0; instId < numInst; ++instId)
            {
                UAVObject* inst = objMngr->getObject(obj->getObjID(), instId);
                transmitSingleObject(inst, type, false);
            }
            return true;
        }
        else
        {
            return transmitSingleObject(obj, type, false);
        }
    }
    else if (type == TYPE_OBJ_REQ)
    {
        return transmitSingleObject(obj, TYPE_OBJ_REQ, allInstances);
    }
    else if (type == TYPE_ACK)
    {
        if (!allInstances)
        {
            return transmitSingleObject(obj, TYPE_ACK, false);
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}


/**
 * Send an object through the telemetry link.
 * \param[in] obj Object handle to send
 * \param[in] type Transaction type
 * \return Success (true), Failure (false)
 */
bool UAVTalk::transmitSingleObject(UAVObject* obj, quint8 type, bool allInstances)
{
    qint32 length;
    qint32 dataOffset;
    quint16 cs = 0;
    quint32 objId;
    quint16 instId;
    quint16 allInstId = ALL_INSTANCES;

    // Setup type and object id fields
    objId = obj->getObjID();
    txBuffer[0] = type;
    qToLittleEndian<quint32>(objId, &txBuffer[1]);

    // Setup instance ID if one is required
    if ( obj->isSingleInstance() )
    {
        dataOffset = 5;
    }
    else
    {
        // Check if all instances are requested
        if (allInstances)
        {
            qToLittleEndian<quint16>(allInstId, &txBuffer[5]);
        }
        else
        {
            instId = obj->getInstID();
            qToLittleEndian<quint16>(instId, &txBuffer[5]);
        }
        dataOffset = 7;
    }

    // Determine data length
    if (type == TYPE_OBJ_REQ || type == TYPE_ACK)
    {
        length = 0;
    }
    else
    {
        length = obj->getNumBytes();
    }

    // Check length
    if (length >= MAX_PAYLOAD_LENGTH)
    {
        return false;
    }

    // Copy data (if any)
    if (length > 0)
    {
        if ( !obj->pack(&txBuffer[dataOffset]) )
        {
            return false;
        }
    }

    // Calculate checksum
    cs = 0;
    cs = updateChecksum(cs, txBuffer, dataOffset+length);
    qToLittleEndian<quint16>(cs, &txBuffer[dataOffset+length]);

    // Send buffer
    io->write((const char*)txBuffer, dataOffset+length+CHECKSUM_LENGTH);

    // Update stats
    ++stats.txObjects;
    stats.txBytes += dataOffset+length+CHECKSUM_LENGTH;
    stats.txObjectBytes += length;

    // Done
    return true;
}

/**
 * Update checksum.
 * TODO: Replace with CRC-16
 * \param[in] data Data buffer to update checksum on
 * \param[in] length Length of buffer
 * \return Updated checksum
 */
quint16 UAVTalk::updateChecksum(quint16 cs, quint8* data, qint32 length)
{
    qint32 n;
    for (n = 0; n < length; ++n)
    {
        cs += (quint16)data[n];
    }
    return cs;
}

















