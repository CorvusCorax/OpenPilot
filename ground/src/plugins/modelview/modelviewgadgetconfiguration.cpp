/**
 ******************************************************************************
 *
 * @file       modelviewgadgetconfiguration.cpp
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup ModelViewPlugin ModelView Plugin
 * @{
 * @brief A gadget that displays a 3D representation of the UAV 
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

#include "modelviewgadgetconfiguration.h"
#include <QtCore/QDataStream>

ModelViewGadgetConfiguration::ModelViewGadgetConfiguration(QString classId, const QByteArray &state, QObject *parent) :
    IUAVGadgetConfiguration(classId, parent),
    m_acFilename("../share/models/Easystar/EasyStar.3ds"),
    m_bgFilename(""),
    m_enableVbo(false)
{
    if (state.count() > 0) {
        QDataStream stream(state);
        stream >> m_acFilename;
        stream >> m_bgFilename;
	stream >> m_enableVbo;
    }
}

IUAVGadgetConfiguration *ModelViewGadgetConfiguration::clone()
{
    ModelViewGadgetConfiguration *mv = new ModelViewGadgetConfiguration(this->classId());
    mv->m_acFilename = m_acFilename;
    mv->m_bgFilename = m_bgFilename;
    mv->m_enableVbo = m_enableVbo;
    return mv;
}

QByteArray ModelViewGadgetConfiguration::saveState() const
{

    QByteArray bytes;
    QDataStream stream(&bytes, QIODevice::WriteOnly);
    stream << m_acFilename;
    stream << m_bgFilename;
    stream << m_enableVbo;
    return bytes;
}

