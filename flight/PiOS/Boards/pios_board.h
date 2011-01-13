#ifndef PIOS_BOARD_H_
#define PIOS_BOARD_H_

#ifdef USE_STM32103CB_AHRS
#include "STM32103CB_AHRS.h"
#elif USE_STM3210E_OP
#include "STM3210E_OP.h"
#elif USE_STM32103CB_PIPXTREME
#include "STM32103CB_PIPXTREME.h"
#endif

#endif /* PIOS_BOARD_H_ */
