#
# Copyright (C) 2014 The CyanogenMod Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# This variable is set first, so it can be overridden
# by BoardConfigVendor.mk

# Architecture
TARGET_CPU_ABI := armeabi-v7a
TARGET_CPU_ABI2 := armeabi
TARGET_ARCH := arm
TARGET_ARCH_VARIANT := armv7-a-neon
TARGET_CPU_VARIANT := cortex-a15

# Bootloader
TARGET_BOOTLOADER_BOARD_NAME := dalmore
TARGET_NO_BOOTLOADER := true

# Platform
TARGET_BOARD_PLATFORM := tegra

# Kernel
BOARD_KERNEL_CMDLINE := androidboot.selinux=permissive
TARGET_KERNEL_SOURCE := kernel/hp/maya
TARGET_KERNEL_CONFIG := cyanogenmod_maya_defconfig

# Audio
BOARD_HAVE_PRE_KITKAT_AUDIO_BLOB := true
BOARD_HAVE_PRE_KITKAT_AUDIO_POLICY_BLOB := true
USE_LEGACY_AUDIO_POLICY := 1

# Bluetooth
BOARD_HAVE_BLUETOOTH := true
BOARD_HAVE_BLUETOOTH_BCM := true
BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR ?= device/hp/maya/bluetooth

# Graphics
USE_OPENGL_RENDERER := true
SF_VSYNC_EVENT_PHASE_OFFSET_NS := 5000000
VSYNC_EVENT_PHASE_OFFSET_NS := 7500000

# Healthd
BOARD_HAL_STATIC_LIBRARIES := libhealthd.maya

# Include an expanded selection of fonts
EXTENDED_FONT_FOOTPRINT := true

# Malloc
MALLOC_IMPL := dlmalloc

# Partition
TARGET_USERIMAGES_USE_EXT4 := true
BOARD_BOOTIMAGE_PARTITION_SIZE := 8388608
BOARD_RECOVERYIMAGE_PARTITION_SIZE := 8388608
BOARD_SYSTEMIMAGE_PARTITION_SIZE := 805306368
BOARD_USERDATAIMAGE_PARTITION_SIZE := 28225568768
BOARD_FLASH_BLOCK_SIZE := 4096

# Pre-L blob support
TARGET_RELEASE_CPPFLAGS += \
    -DADD_LEGACY_ACQUIRE_BUFFER_SYMBOL \
    -DADD_LEGACY_MEMORY_DEALER_CONSTRUCTOR_SYMBOL \
    -DNEEDS_VECTORIMPL_SYMBOLS

# Recovery
BOARD_USE_CUSTOM_RECOVERY_FONT := \"roboto_23x41.h\"
COMMON_GLOBAL_CFLAGS += -DNO_SECURE_DISCARD
TARGET_RECOVERY_DEVICE_DIRS += device/hp/maya
TARGET_RECOVERY_FSTAB := device/hp/maya/rootdir/etc/fstab.maya

# SELinux
BOARD_SEPOLICY_DIRS += \
    device/hp/maya/sepolicy

BOARD_SEPOLICY_UNION += \
    app.te \
    bluetooth.te \
    bootanim.te \
    cpuvoltcap.te \
    debuggerd.te \
    device.te \
    domain.te \
    drmserver.te \
    dumpstate.te \
    file_contexts \
    file.te \
    genfs_contexts \
    gpload.te \
    gpsd.te \
    healthd.te \
    hostapd.te \
    init.te \
    installd.te \
    mediaserver.te \
    netd.te \
    platform_app.te \
    property_contexts \
    property.te \
    service_contexts \
    set_hwui.te \
    setup_fs.te \
    shell.te \
    surfaceflinger.te \
    system_app.te \
    system_server.te \
    syntouchpad.te \
    tee.te \
    te_macros \
    ueventd.te \
    untrusted_app.te \
    usb.te \
    usdwatchdog.te \
    ussrd.te \
    ussr_setup.te \
    vold.te \
    wifi_loader.te \
    wl18xx.te \
    wpa.te \
    zygote.te

# Wifi related defines
BOARD_WPA_SUPPLICANT_DRIVER      := NL80211
WPA_SUPPLICANT_VERSION           := VER_0_8_X
BOARD_WPA_SUPPLICANT_PRIVATE_LIB := lib_driver_cmd_bcmdhd
BOARD_WLAN_DEVICE                := bcmdhd
BOARD_HOSTAPD_DRIVER             := NL80211
BOARD_HOSTAPD_PRIVATE_LIB        := lib_driver_cmd_bcmdhd
WIFI_DRIVER_FW_PATH_PARAM        := "/sys/module/bcmdhd/parameters/firmware_path"
WIFI_DRIVER_FW_PATH_AP           := "/system/vendor/firmware/bcm4334/fw_bcmdhd_apsta.bin"
WIFI_DRIVER_FW_PATH_STA          := "/system/vendor/firmware/bcm4334/fw_bcmdhd.bin"

TW_THEME := landscape_hdpi
BOARD_HAS_NO_REAL_SDCARD := true
RECOVERY_GRAPHICS_USE_LINELENGTH := true
RECOVERY_SDCARD_ON_DATA := true
TW_BRIGHTNESS_PATH := "/sys/class/backlight/pwm-backlight/brightness"
TW_INCLUDE_SYNTOUCHPAD := true
TW_NO_USB_STORAGE := true
