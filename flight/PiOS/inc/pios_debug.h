/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @defgroup   PIOS_DEBUG Debugging Functions
 * @brief Debugging functionality
 * @{
 *
 * @file       pios_i2c.h  
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Debug helper functions header.
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

#ifndef PIOS_DEBUG_H
#define PIOS_DEBUG_H

extern const char *PIOS_DEBUG_AssertMsg;

void PIOS_DEBUG_Init(void);
void PIOS_DEBUG_PinHigh(uint8_t pin);
void PIOS_DEBUG_PinLow(uint8_t pin);
void PIOS_DEBUG_Panic(const char *msg);

#ifdef DEBUG
#define PIOS_DEBUG_Assert(test) if (!(test)) PIOS_DEBUG_Panic(PIOS_DEBUG_AssertMsg);
#else
#define PIOS_DEBUG_Assert(test)
#endif

#endif /* PIOS_DEBUG_H */

/**
  * @}
  * @}
  */
