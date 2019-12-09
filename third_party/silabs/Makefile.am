#
#  Copyright (c) 2019, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

include $(abs_top_nlbuild_autotools_dir)/automake/pre.am

if OPENTHREAD_EXAMPLES_EFR32MG12
lib_LIBRARIES                                                                                = \
    libsilabs-efr32mg12-sdk.a                                                                  \
    $(NULL)
endif

if OPENTHREAD_EXAMPLES_EFR32MG21
lib_LIBRARIES                                                                                = \
    libsilabs-efr32mg21-sdk.a                                                                  \
    $(NULL)
endif

# Do not enable -Wconversion for Silicon Labs SDK sources
override CFLAGS                              := $(filter-out -Wconversion,$(CFLAGS))
override CXXFLAGS                            := $(filter-out -Wconversion,$(CXXFLAGS))

# Do not enable -pedantic-errors for Silicon Labs SDK sources
override CFLAGS                              := $(filter-out -pedantic-errors,$(CFLAGS))
override CXXFLAGS                            := $(filter-out -pedantic-errors,$(CXXFLAGS))

# Do not enable -Wshadow for Silicon Labs SDK sources
override CFLAGS                              := $(filter-out -Wshadow,$(CFLAGS))
override CXXFLAGS                            := $(filter-out -Wshadow,$(CXXFLAGS))

EFR32_BOARD_DIR                             = $(shell echo $(BOARD) | tr A-Z a-z)

EFR32MG_SDK_SRCDIR                            = $(top_srcdir)/third_party/silabs/sdk

COMMONCPPFLAGS                                                                                = \
    -D__STARTUP_CLEAR_BSS                                                                       \
    -I$(srcdir)                                                                                 \
    -I$(top_srcdir)/include                                                                     \
    -I$(top_srcdir)/src/core                                                                    \
    -I$(top_srcdir)/third_party/silabs/rail_config                                              \
    -I$(top_srcdir)/examples/platforms/efr32mg21/$(EFR32_BOARD_DIR)                             \
    -I$(EFR32MG_SDK_SRCDIR)                                                                     \
    -I$(EFR32MG_SDK_SRCDIR)/platform/radio/rail_lib/common                                      \
    -I$(EFR32MG_SDK_SRCDIR)/platform/radio/rail_lib/chip/efr32                                  \
    -I$(EFR32MG_SDK_SRCDIR)/platform/radio/rail_lib/protocol/ieee802154                         \
    -I$(EFR32MG_SDK_SRCDIR)/platform/radio/rail_lib/chip/efr32/rf/common/cortex                 \
    -I$(EFR32MG_SDK_SRCDIR)/platform/radio/rail_lib/hal                                         \
    -I$(EFR32MG_SDK_SRCDIR)/platform/radio/rail_lib/hal/efr32                                   \
    -I$(EFR32MG_SDK_SRCDIR)/platform/radio/rail_lib/plugin/pa-conversions                       \
    -I$(EFR32MG_SDK_SRCDIR)/hardware/kit/common/bsp                                             \
    -I$(EFR32MG_SDK_SRCDIR)/platform/CMSIS/Include                                              \
    -I$(EFR32MG_SDK_SRCDIR)/platform/emdrv/common/inc                                           \
    -I$(EFR32MG_SDK_SRCDIR)/platform/emdrv/gpiointerrupt/inc                                    \
    -I$(EFR32MG_SDK_SRCDIR)/platform/emdrv/uartdrv/inc                                          \
    -I$(EFR32MG_SDK_SRCDIR)/platform/emdrv/uartdrv/config                                       \
    -I$(EFR32MG_SDK_SRCDIR)/platform/emdrv/ustimer/inc                                          \
    -I$(EFR32MG_SDK_SRCDIR)/platform/emdrv/dmadrv/inc                                           \
    -I$(EFR32MG_SDK_SRCDIR)/platform/emdrv/dmadrv/config                                        \
    -I$(EFR32MG_SDK_SRCDIR)/platform/emdrv/rtcdrv/inc                                           \
    -I$(EFR32MG_SDK_SRCDIR)/platform/emlib/inc                                                  \
    -I$(EFR32MG_SDK_SRCDIR)/platform/halconfig/inc/hal-config                                   \
    -I$(EFR32MG_SDK_SRCDIR)/util/plugin/plugin-common/fem-control                               \
    -I$(EFR32MG_SDK_SRCDIR)/util/third_party/mbedtls/sl_crypto/include                          \
    -Wno-unused-parameter                                                                       \
    -Wno-missing-field-initializers                                                             \
    $(NULL)

SILABS_EFR32MG12_CPPFLAGS                                                                     = \
    -I$(EFR32MG_SDK_SRCDIR)/hardware/kit/EFR32MG12_$(BOARD)/config                              \
    -I$(EFR32MG_SDK_SRCDIR)/platform/Device/SiliconLabs/EFR32MG12P/Include                      \
    $(NULL)

SILABS_EFR32MG21_CPPFLAGS                                                                     = \
    -I$(EFR32MG_SDK_SRCDIR)/hardware/kit/EFR32MG21_$(BOARD)/config                              \
    -I$(EFR32MG_SDK_SRCDIR)/platform/Device/SiliconLabs/EFR32MG21P/Include                      \
    $(NULL)

SILABS_COMMON_SOURCES                                                                         = \
    rail_config/rail_config.c                                                                   \
    sdk/hardware/kit/common/bsp/bsp_bcc.c                                      \
    sdk/hardware/kit/common/bsp/bsp_init.c                                     \
    sdk/hardware/kit/common/bsp/bsp_stk.c                                      \
    sdk/hardware/kit/common/bsp/bsp_stk_leds.c                                 \
    sdk/platform/emdrv/dmadrv/src/dmadrv.c                                     \
    sdk/platform/emdrv/gpiointerrupt/src/gpiointerrupt.c                       \
    sdk/platform/emdrv/rtcdrv/src/rtcdriver.c                                  \
    sdk/platform/emdrv/uartdrv/src/uartdrv.c                                   \
    sdk/platform/emdrv/ustimer/src/ustimer.c                                   \
    sdk/platform/emlib/src/em_adc.c                                            \
    sdk/platform/emlib/src/em_cmu.c                                            \
    sdk/platform/emlib/src/em_core.c                                           \
    sdk/platform/emlib/src/em_crypto.c                                         \
    sdk/platform/emlib/src/em_emu.c                                            \
    sdk/platform/emlib/src/em_gpio.c                                           \
    sdk/platform/emlib/src/em_ldma.c                                           \
    sdk/platform/emlib/src/em_leuart.c                                         \
    sdk/platform/emlib/src/em_msc.c                                            \
    sdk/platform/emlib/src/em_rmu.c                                            \
    sdk/platform/emlib/src/em_rtcc.c                                           \
    sdk/platform/emlib/src/em_system.c                                         \
    sdk/platform/emlib/src/em_timer.c                                          \
    sdk/platform/emlib/src/em_usart.c                                          \
    sdk/platform/radio/rail_lib/hal/efr32/hal_efr.c                            \
    sdk/platform/radio/rail_lib/hal/hal_common.c                               \
    sdk/util/third_party/mbedtls/library/ecp.c                                 \
    sdk/util/third_party/mbedtls/sl_crypto/src/crypto_aes.c                    \
    sdk/util/third_party/mbedtls/sl_crypto/src/crypto_ecp.c                    \
    sdk/util/third_party/mbedtls/sl_crypto/src/crypto_management.c             \
    $(NULL)

SILABS_EFR32MG12_SOURCES                                                                      = \
    sdk/platform/Device/SiliconLabs/EFR32MG12P/Source/system_efr32mg12p.c      \
    sdk/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC/startup_efr32mg12p.c \
    $(NULL)

SILABS_EFR32MG21_SOURCES                                                                      = \
    sdk/platform/Device/SiliconLabs/EFR32MG21/Source/system_efr32mg21.c        \
    sdk/platform/Device/SiliconLabs/EFR32MG21/Source/GCC/startup_efr32mg21.c   \
    sdk/platform/emlib/src/em_se.c                                             \
    sdk/util/third_party/mbedtls/sl_crypto/src/se_trng.c                       \
    sdk/util/third_party/mbedtls/sl_crypto/src/se_aes.c                        \
    sdk/util/third_party/mbedtls/sl_crypto/src/se_management.c                 \
    $(NULL)

libsilabs_efr32mg12_sdk_a_CPPFLAGS                                                            = \
    $(COMMONCPPFLAGS)                                                                           \
    $(SILABS_EFR32MG12_CPPFLAGS)                                                                \
    $(NULL)

libsilabs_efr32mg12_sdk_a_SOURCES                                                             = \
    $(SILABS_COMMON_SOURCES)                                                                    \
    $(SILABS_EFR32MG12_SOURCES)                                                                 \
    $(NULL)

libsilabs_efr32mg21_sdk_a_CPPFLAGS                                                            = \
    $(COMMONCPPFLAGS)                                                                           \
    $(SILABS_EFR32MG21_CPPFLAGS)                                                                \
    $(NULL)

libsilabs_efr32mg21_sdk_a_SOURCES                                                             = \
    $(SILABS_COMMON_SOURCES)                                                                    \
    $(SILABS_EFR32MG21_SOURCES)                                                                 \
    $(NULL)

noinst_HEADERS                                                                                = \
        $(NULL) 


include $(abs_top_nlbuild_autotools_dir)/automake/post.am
