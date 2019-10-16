/*
 *  Copyright (c) 2019, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * @brief
 *   This file includes the platform-specific initializers.
 */

#include <string.h>

#include "openthread-system.h"
#include <openthread/platform/uart.h>

#include "common/logging.hpp"

#include "bsp.h"
#include "em_chip.h"
#include "em_core.h"
#include "em_emu.h"
#include "em_system.h"
#include "hal-config.h"
#include "hal_common.h"
#include "rail.h"
#include "rtcdriver.h"

#include "openthread-core-efr32-config.h"
#include "platform-efr32.h"







//#include "stack/include/ember.h"
//#include "include/error.h"
//#include "hal/hal.h"
//#include "serial/serial.h"
//#include "cstartup-common.h"
//#include "coexistence/protocol/ieee802154/coexistence-802154.h"


//  #include "rtos/rtos.h"


//#include <interrupts-efm32.h>









#if (HAL_FEM_ENABLE)
#include "fem-control.h"
#endif

#define USE_EFR32_LOG                                                                   \
    ((OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED) || \
     (OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_NCP_SPINEL))

void halInitChipSpecific(void);

otInstance *sInstance;

void otSysInit(int argc, char *argv[])
{
    OT_UNUSED_VARIABLE(argc);
    OT_UNUSED_VARIABLE(argv);

    __disable_irq();
    
      // Configure BASEPRI to be at the interrupts disabled level so that when we
  // turn interrupts back on nothing fires immediately.
//  INTERRUPTS_OFF();

  CORE_ATOMIC_IRQ_DISABLE();

  // Bootloader might be at the base of flash, or even in the NULL_BTL case,
  // the BAT/AAT will be at the beginning of the image.
  // Setting the vectorTable is required.
//  SCB->VTOR =  (uint32_t)halAppAddressTable.baseTable.vectorTable;


#undef FIXED_EXCEPTION
#define FIXED_EXCEPTION(vectorNumber, functionName, deviceIrqn, deviceIrqHandler)
#define EXCEPTION(vectorNumber, functionName, deviceIrqn, deviceIrqHandler, priorityLevel, subpriority) \
    NVIC_SetPriority(deviceIrqn, NVIC_EncodePriority(PRIGROUP_POSITION, priorityLevel, subpriority));
#include NVIC_CONFIG
#undef EXCEPTION

    NVIC_SetPriorityGrouping(PRIGROUP_POSITION);
    CHIP_Init();
    halInitChipSpecific();
    BSP_Init(BSP_INIT_BCC);
    RTCDRV_Init();

#if (HAL_FEM_ENABLE)
    initFem();
    wakeupFem();
#endif

    __enable_irq();

    CORE_ATOMIC_IRQ_ENABLE();

#if USE_EFR32_LOG
    efr32LogInit();
#endif
    efr32RadioInit();
    efr32AlarmInit();
    efr32MiscInit();
}

bool otSysPseudoResetWasRequested(void)
{
    return false;
}

void otSysDeinit(void)
{
    efr32RadioDeinit();

#if USE_EFR32_LOG
    efr32LogDeinit();
#endif
}

void otSysProcessDrivers(otInstance *aInstance)
{
    sInstance = aInstance;

    // should sleep and wait for interrupts here

    efr32UartProcess();
    efr32RadioProcess(aInstance);
    efr32AlarmProcess(aInstance);
}

__WEAK void otSysEventSignalPending(void)
{
    // Intentionally empty
}
