PROJECT_NAME     := Watch
TARGETS          := Watch
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../nrfSDK
NRF_TOOL_PATH := ../nrfTools
PROJ_DIR := .
# UNDONE: I don't think I really want to commit this change.
MRI_PATH := ../../mri-swd/mri
MRIBLUE_PATH := ../mriblue_boot/shared

# UNDONE: Put this back.
#$(OUTPUT_DIRECTORY)/Watch.out: \
#  LINKER_SCRIPT  := $(MRIBLUE_PATH)/nrf52.ld
$(OUTPUT_DIRECTORY)/Watch.out: \
  LINKER_SCRIPT  := nrf52.ld

# Source files common to all targets
SRC_FILES += \
  $(MRIBLUE_PATH)/newlib_retarget.c \
  $(PROJ_DIR)/main.cpp \
  $(PROJ_DIR)/Adafruit-GFX-Library/Adafruit_GFX.cpp \
  $(PROJ_DIR)/ColorMemLCD/ColorMemLCD.cpp \
  $(PROJ_DIR)/SAADCScanner/SAADCScanner.cpp \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/crc16/crc16.c \
  $(SDK_ROOT)/components/libraries/fds/fds.c \
  $(SDK_ROOT)/components/libraries/fstorage/fstorage.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/util/sdk_errors.c \
  $(SDK_ROOT)/components/libraries/util/sdk_mapped_flags.c \
  $(SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c \
  $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
  $(SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c \
  $(SDK_ROOT)/components/drivers_nrf/hal/nrf_adc.c \
  $(SDK_ROOT)/components/drivers_nrf/spi_master/nrf_drv_spi.c \
  $(SDK_ROOT)/components/toolchain/system_nrf52.c \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
  $(SDK_ROOT)/components/drivers_nrf/timer/nrf_drv_timer.c \
  $(SDK_ROOT)/components/libraries/simple_timer/app_simple_timer.c \
  $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf52840.S \
  $(SDK_ROOT)/components/drivers_nrf/hal/nrf_saadc.c \
  $(SDK_ROOT)/components/ble/ble_db_discovery/ble_db_discovery.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_manager.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas_c/ble_bas_c.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs_c/ble_hrs_c.c \
  $(SDK_ROOT)/components/libraries/fstorage/fstorage.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_state.c \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c \
  $(SDK_ROOT)/components/libraries/fds/fds.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/ble/peer_manager/id_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_dispatcher.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatt_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatts_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_data_storage.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_database.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_id.c \
  $(SDK_ROOT)/components/ble/peer_manager/pm_buffer.c \
  $(SDK_ROOT)/components/ble/peer_manager/pm_mutex.c \
  $(SDK_ROOT)/components/drivers_nrf/pwm/nrf_drv_pwm.c \


# UNDONE: Add the below back in to the above list before main.c
#  $(MRIBLUE_PATH)/gcc_startup_nrf52840.S \

# Add in MRI debug monitor to get the newlib stubs contained within.
LIB_FILES += $(MRI_PATH)/lib/armv7-m/libmri_nrf52dk_fpu_hard.a

# Include folders common to all targets
INC_FOLDERS += \
  $(PROJ_DIR) \
  $(MRI_PATH) \
  $(MRIBLUE_PATH) \
  $(PROJ_DIR)/Adafruit-GFX-Library \
  $(SDK_ROOT)/components/drivers_nrf/comp \
  $(SDK_ROOT)/components/drivers_nrf/twi_master \
  $(SDK_ROOT)/components/softdevice/s132/headers \
  $(SDK_ROOT)/components/libraries/pwm \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc/acm \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/generic \
  $(SDK_ROOT)/components/libraries/usbd/class/msc \
  $(SDK_ROOT)/components/libraries/usbd/class/hid \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/drivers_nrf/i2s \
  $(SDK_ROOT)/components/libraries/gpiote \
  $(SDK_ROOT)/components/drivers_nrf/gpiote \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/drivers_nrf/common \
  $(SDK_ROOT)/components/ble/ble_advertising \
  $(SDK_ROOT)/components/drivers_nrf/adc \
  $(SDK_ROOT)/components/softdevice/s132/headers/nrf52 \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/drivers_nrf/uart \
  $(SDK_ROOT)/components/drivers_nrf/wdt \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/libraries/slip \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/libraries/csense_drv \
  $(SDK_ROOT)/components/drivers_nrf/hal \
  $(SDK_ROOT)/components/drivers_nrf/rtc \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/mouse \
  $(SDK_ROOT)/components/drivers_nrf/ppi \
  $(SDK_ROOT)/components/drivers_nrf/twis_slave \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/drivers_nrf/delay \
  $(SDK_ROOT)/components/libraries/crc16 \
  $(SDK_ROOT)/components/drivers_nrf/timer \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/drivers_nrf/pwm \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc \
  $(SDK_ROOT)/components/libraries/csense \
  $(SDK_ROOT)/components/drivers_nrf/rng \
  $(SDK_ROOT)/components/libraries/low_power_pwm \
  $(SDK_ROOT)/components/libraries/hardfault \
  $(SDK_ROOT)/components/libraries/uart \
  $(SDK_ROOT)/components/libraries/hci \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/kbd \
  $(SDK_ROOT)/components/drivers_nrf/spi_slave \
  $(SDK_ROOT)/components/drivers_nrf/lpcomp \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/drivers_nrf/power \
  $(SDK_ROOT)/components/libraries/usbd/config \
  $(SDK_ROOT)/components/toolchain \
  $(SDK_ROOT)/components/libraries/led_softblink \
  $(SDK_ROOT)/components/drivers_nrf/qdec \
  $(SDK_ROOT)/components/ble/ble_services/ble_cts_c \
  $(SDK_ROOT)/components/drivers_nrf/spi_master \
  $(SDK_ROOT)/components/drivers_nrf/pdm \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/libraries/usbd/class/audio \
  $(SDK_ROOT)/components/libraries/sensorsim \
  $(SDK_ROOT)/components/drivers_nrf/swi \
  $(SDK_ROOT)/components/device \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/components/libraries/usbd \
  $(SDK_ROOT)/components/drivers_nrf/saadc \
  $(SDK_ROOT)/components/toolchain/gcc \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/components/libraries/twi \
  $(SDK_ROOT)/components/drivers_nrf/clock \
  $(SDK_ROOT)/components/drivers_nrf/usbd \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/libraries/simple_timer \
  $(SDK_ROOT)/components/ble/ble_db_discovery \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/peer_manager \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas_c \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt \

# C flags common to all targets
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -D__STACK_SIZE=8
CFLAGS += -DDEBUG_NRF_USER
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DBOARD_PCA10056
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DS132
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall -Werror -O3 -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DNRF_SD_BLE_API_VERSION=3

# Disable function cloning as it grows the code and explodes hardware breakpoint requirements.
CFLAGS += -fno-ipa-cp-clone

# Disabling inlining tends to make debugging easier.
CFLAGS += -fno-inline

# C++ flags common to all targets
CXXFLAGS += -fno-rtti
# Don't want exception handling enabled for embedded builds, to shrink size.
CXXFLAGS += -fno-exceptions
# Don't wrap function static object instantiation in guards since we are single threaded.
CXXFLAGS += -fno-threadsafe-statics

# Assembler flags common to all targets
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -D__HEAP_SIZE=0
ASMFLAGS += -D__STACK_SIZE=8
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DBOARD_PCA10056
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DS132
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=3

# Linker flags
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs
# UNDONE: I might be able to remove this.
# Enabled floating point support in printf() and scanf() format specifiers.
LDFLAGS += -u _printf_float -u _scanf_float
SYSLIBS += -lc -lnosys -lm

.PHONY: $(TARGETS) default all clean help flash flash_softdevice sdk mri

# Default target - first one defined
default: Watch

# Print all targets that can be built
help:
	@echo The following rules are available:
	@echo   sdk - Needs to be run first to install Nordic SDK, required for build.
	@echo   mri - Make sure that the mri submodule prereq is initialized and built.
	@echo   clean - Deletes all output binaries so that next build is performed from scratch.
	@echo   all - Builds all supported targets - nrf52832_xxaa.
	@echo   erase - Erases all of the FLASH on the nRF5 device.
	@echo   flash_softdevice - Flashes the Nordic BLE Soft Device. Will erase device first.
	@echo   flash - Just flashes your application firmware. flash_softdevice must be run at least once before.
	@echo 	$(TARGETS)

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

-include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

# Flash the program
flash: $(OUTPUT_DIRECTORY)/Watch.hex
	@echo Flashing: $<
	nrfjprog --program $< -f nrf52 --sectorerase
	nrfjprog --reset -f nrf52

# Flash softdevice
flash_softdevice: erase
	@echo Flashing: s132_nrf52_3.0.0_softdevice.hex
	nrfjprog --program $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_3.0.0_softdevice.hex -f nrf52 --sectorerase
	nrfjprog --reset -f nrf52

erase:
	nrfjprog --eraseall -f nrf52


SDK_VER := nRF5_SDK_12.3.0_d7731ad
SDK_ZIP := $(SDK_VER).zip
SDK_URL := https://developer.nordicsemi.com/nRF5_SDK/pieces/nRF5_SDK_v12.x.x/$(SDK_ZIP)
SDK_ZIP_PATH = $(dir $(SDK_ROOT))$(SDK_ZIP)

# sdk target is used to download SDK from the Nordic website.
sdk: $(SDK_ROOT)

# The SDK source code only exists after it has been unzipped from the SDK archive.
$(SDK_ROOT): $(SDK_ZIP_PATH)
	@echo Unzipping $<
	$(NO_ECHO)unzip -d $@ $<
	$(NO_ECHO)mv $(SDK_ROOT)/$(SDK_VER)/* $(SDK_ROOT)
	$(NO_ECHO)rm -d $(SDK_ROOT)/$(SDK_VER)
	$(NO_ECHO)patch -d $(SDK_ROOT) -p1 <$(dir $(SDK_ROOT))sdk.patch

# Download SDK archive from the Nordic website.
$(SDK_ZIP_PATH):
	@echo Downloading $@
	$(NO_ECHO)curl -L0 $(SDK_URL) >$@

mri :
	@echo Initializing mri submodules...
	@cd ..; git submodule update --init mri
	@echo Building mri...
	@$(MAKE) --no-print-directory -C ../mri
