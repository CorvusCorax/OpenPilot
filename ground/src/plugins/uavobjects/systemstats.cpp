/**
 ******************************************************************************
 *
 * @file       systemstats.cpp
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Implementation of the SystemStats object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: systemstats.xml. 
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
#include "systemstats.h"
#include "uavobjectfields.h"

const QString SystemStats::NAME = QString("SystemStats");

SystemStats::SystemStats(): UAVDataObject(OBJID, ISSINGLEINST, ISSETTINGS, NAME)
{
    // Create fields
    QList<UAVObjectField*> fields;
    QStringList FlightTimeElemNames;
    FlightTimeElemNames.append("[0]");
    fields.append(new UAVObjectFieldUInt32(QString("FlightTime"), QString("ms"), FlightTimeElemNames));
    QStringList HeapRemainingElemNames;
    HeapRemainingElemNames.append("[0]");
    fields.append(new UAVObjectFieldUInt16(QString("HeapRemaining"), QString("bytes"), HeapRemainingElemNames));
    QStringList CPULoadElemNames;
    CPULoadElemNames.append("[0]");
    fields.append(new UAVObjectFieldUInt8(QString("CPULoad"), QString("%"), CPULoadElemNames));

    // Initialize object
    initializeFields(fields, (quint8*)&data, NUMBYTES);
}

UAVObject::Metadata SystemStats::getDefaultMetadata()
{
    UAVObject::Metadata metadata;
    metadata.gcsTelemetryAcked = 1;
    metadata.gcsTelemetryUpdateMode = UAVObject::UPDATEMODE_NEVER;
    metadata.gcsTelemetryUpdatePeriod = 0;
    metadata.flightTelemetryAcked = 1;
    metadata.flightTelemetryUpdateMode = UAVObject::UPDATEMODE_PERIODIC;
    metadata.flightTelemetryUpdatePeriod = 1000;
    metadata.loggingUpdateMode = UAVObject::UPDATEMODE_PERIODIC;
    metadata.loggingUpdatePeriod = 1000;
    return metadata;
}

SystemStats::DataFields SystemStats::getData()
{
    QMutexLocker locker(mutex);
    return data;
}

void SystemStats::setData(DataFields& data)
{
    QMutexLocker locker(mutex);
    this->data = data;
    emit objectUpdatedAuto(this); // trigger object updated event
    emit objectUpdated(this);
}

UAVDataObject* SystemStats::clone(quint32 instID)
{
    SystemStats* obj = new SystemStats();
    obj->initialize(instID, this->getMetaObject());
    return obj;
}
