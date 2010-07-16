/**
 ******************************************************************************
 *
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @see        The GNU Public License (GPL) Version 3
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup UAVObjectsPlugin UAVObjects Plugin
 * @{
 * @brief      The UAVUObjects GCS plugin 
 *   
 * @note       Object definition file: exampleobject1.xml. 
 *             This is an automatically generated file.
 *             DO NOT modify manually.
 *
 * @file       exampleobject1.h
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

#include "uavdataobject.h"
#include "uavobjectmanager.h"

class UAVOBJECTS_EXPORT ExampleObject1: public UAVDataObject
{
    Q_OBJECT

public:
    // Field structure
    typedef struct {
        qint8 Field1;
        qint16 Field2;
        qint32 Field3;
        float Field4[4];
        quint8 Field5;
        quint16 Field6;
        quint32 Field7;
        quint8 Field8;

    } __attribute__((packed)) DataFields;

    // Field information
    // Field Field1 information
    // Field Field2 information
    // Field Field3 information
    // Field Field4 information
    /* Number of elements for field Field4 */
    static const quint32 FIELD4_NUMELEM = 4;
    // Field Field5 information
    // Field Field6 information
    // Field Field7 information
    // Field Field8 information
    /* Enumeration options for field Field8 */
    typedef enum { FIELD8_OPTION1=0, FIELD8_OPTION2=1,  } Field8Options;

  
    // Constants
    static const quint32 OBJID = 3852936276U;
    static const QString NAME;
    static const bool ISSINGLEINST = 0;
    static const bool ISSETTINGS = 0;
    static const quint32 NUMBYTES = sizeof(DataFields);

    // Functions
    ExampleObject1();

    DataFields getData();
    void setData(const DataFields& data);
    Metadata getDefaultMetadata();
    UAVDataObject* clone(quint32 instID);

    static ExampleObject1* GetInstance(UAVObjectManager* objMngr, quint32 instID = 0);
	
private:
    DataFields data;

    void setDefaultFieldValues();

};

#endif // EXAMPLEOBJECT1_H
