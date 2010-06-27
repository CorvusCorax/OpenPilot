/**
 ******************************************************************************
 *
 * @file       hitlil2.cpp
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief
 * @see        The GNU Public License (GPL) Version 3
 * @defgroup   hitlil2plugin
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
#include "hitlil2.h"
#include "hitlil2widget.h"
#include "hitlil2configuration.h"

HITLIL2::HITLIL2(QString classId, HITLIL2Widget *widget, QWidget *parent) :
        IUAVGadget(classId, parent),
        m_widget(widget)
{
}

HITLIL2::~HITLIL2()
{

}

void HITLIL2::loadConfiguration(IUAVGadgetConfiguration* config)
{
    HITLIL2Configuration *m = qobject_cast<HITLIL2Configuration*>(config);
    m_widget->setIlHostName( m->il2HostName() );
    m_widget->setIl2Port( m->il2Port() );
    m_widget->setIl2ManualControl( m->il2ManualControl() );
}

