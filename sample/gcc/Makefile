TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_PCA10001

# Paths
PROJ_PATH = /home/luis/Desktop/SETEC/sample/
SDK_PATH = /home/luis/Desktop/SETEC/nrf51822/
SOFTDEVICE = /home/luis/Desktop/SETEC/s110_nrf51822_7.3.0_softdevice.hex

# Project Source files
C_SOURCE_FILES := $(shell find $(PROJ_PATH) -name '*.c')

# SDK Source files
C_SOURCE_FILES += twi_hw_master.c
C_SOURCE_FILES += ble_dis.c
C_SOURCE_FILES += ble_srv_common.c
C_SOURCE_FILES += softdevice_handler.c
C_SOURCE_FILES += ble_advdata.c
C_SOURCE_FILES += ble_conn_params.c
C_SOURCE_FILES += app_timer.c
C_SOURCE_FILES += pstorage.c
C_SOURCE_FILES += device_manager_peripheral.c
C_SOURCE_FILES += app_gpiote.c
C_SOURCE_FILES += nrf_delay.c
C_SOURCE_FILES += app_scheduler.c

OUTPUT_FILENAME := setec

DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

USE_SOFTDEVICE := S110
#USE_SOFTDEVICE := S210

# -ffunction-sections: keep every function in separate section. This will allow linker to dump unused functions
CFLAGS := -DDEBUG_NRF_USER -DBLE_STACK_SUPPORT_REQD -DARM_MATH_CM0 -DFIXED_POINT -DFAST_AND_DIRTY -fno-strict-aliasing -ffunction-sections

# we do not use heap in this app
ASMFLAGS := -D__HEAP_SIZE=0

# let linker to dump unused sections
LDFLAGS := -Wl,--gc-sections -DARM_MATH_CM0 -DFIXED_POINT -DFAST_AND_DIRTY -fno-strict-aliasing

LIBRARIES = -lm

# SDK paths
INCLUDEPATHS += -I"/home/luis/Desktop/SETEC/s110_nrf51822_7.3.0_API/include"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/device_manager"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/ble_services"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sdk"

# Add project paths here
INCLUDEPATHS += -I"$(PROJ_PATH)ble_services"
INCLUDEPATHS += -I"$(PROJ_PATH)drivers"
INCLUDEPATHS += -I"$(PROJ_PATH)drivers/sensors/ambient"
INCLUDEPATHS += -I"$(PROJ_PATH)drivers/hardware/bme280"
INCLUDEPATHS += -I"$(PROJ_PATH)drivers/hardware/led"
INCLUDEPATHS += -I"$(PROJ_PATH)drivers/hardware/max1161x"
INCLUDEPATHS += -I"$(PROJ_PATH)drivers/hardware/max17048"
INCLUDEPATHS += -I"$(PROJ_PATH)drivers/hardware/mpu9250"
INCLUDEPATHS += -I"$(PROJ_PATH)drivers/hardware/pca9538"
INCLUDEPATHS += -I"$(PROJ_PATH)drivers/hardware/sd_card"
INCLUDEPATHS += -I"$(PROJ_PATH)drivers/hardware/segger_rtt"
INCLUDEPATHS += -I"$(PROJ_PATH)drivers/hardware/spi"
INCLUDEPATHS += -I"$(PROJ_PATH)util"
INCLUDEPATHS += -I"$(PROJ_PATH)config"
INCLUDEPATHS += -I"$(PROJ_PATH)config/ble_services_config"
INCLUDEPATHS += -I"$(PROJ_PATH)config/board_config"
INCLUDEPATHS += -I"$(PROJ_PATH)config/board_config/sensingplus"

# SDK source paths
C_SOURCE_PATHS += $(SDK_PATH)Source/ble
C_SOURCE_PATHS += $(SDK_PATH)Source/ble/device_manager
C_SOURCE_PATHS += $(SDK_PATH)Source/app_common
C_SOURCE_PATHS += $(SDK_PATH)Source/sd_common

#Add all project source paths here
C_SOURCE_PATHS += $(PROJ_PATH)util
C_SOURCE_PATHS += $(PROJ_PATH)drivers
C_SOURCE_PATHS += $(PROJ_PATH)drivers/sensors/ambient
C_SOURCE_PATHS += $(PROJ_PATH)drivers/hardware/bme280
C_SOURCE_PATHS += $(PROJ_PATH)drivers/hardware/led
C_SOURCE_PATHS += $(PROJ_PATH)drivers/hardware/max1161x
C_SOURCE_PATHS += $(PROJ_PATH)drivers/hardware/max17048
C_SOURCE_PATHS += $(PROJ_PATH)drivers/hardware/mpu9250
C_SOURCE_PATHS += $(PROJ_PATH)drivers/hardware/pca9538
C_SOURCE_PATHS += $(PROJ_PATH)drivers/hardware/sd_card
C_SOURCE_PATHS += $(PROJ_PATH)drivers/hardware/segger_rtt
C_SOURCE_PATHS += $(PROJ_PATH)drivers/hardware/spi
C_SOURCE_PATHS += $(PROJ_PATH)ble_services
C_SOURCE_PATHS += $(PROJ_PATH)config

include $(SDK_PATH)Source/templates/gcc/Makefile.common
