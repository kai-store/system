LOCAL_PATH := $(call my-dir)

# Setup bdroid local make variables for handling configuration
ifneq ($(BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR),)
  bdroid_C_INCLUDES := $(BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR)
  bdroid_CFLAGS += -DHAS_BDROID_BUILDCFG
else
  bdroid_C_INCLUDES :=
  bdroid_CFLAGS += -DHAS_NO_BDROID_BUILDCFG
endif

#BLUEDROID_HCI_VENDOR_STATIC_LINKING := true

#BLUEDROID_EXTRA_BPLUS_STATIC_LINKING := true

ifneq ($(BOARD_BLUETOOTH_BDROID_HCILP_INCLUDED),)
  bdroid_CFLAGS += -DHCILP_INCLUDED=$(BOARD_BLUETOOTH_BDROID_HCILP_INCLUDED)
endif

ifneq ($(TARGET_BUILD_VARIANT),user)
bdroid_CFLAGS += -DBLUEDROID_DEBUG
endif

ifeq ($(BOARD_SPRD_WCNBT_MARLIN), true)
  bdroid_CFLAGS += -DSPRD_WCNBT_MARLIN
  ifneq ($(strip $(WCN_EXTENSION)),true)
    bdroid_CFLAGS += -DSPRD_WCNBT_MARLIN_15A
  endif
endif
ifeq ($(BOARD_SPRD_WCNBT_SR2351), true)
  bdroid_CFLAGS += -DSPRD_WCNBT_SR2351
endif

ifeq ($(BOARD_BLUETOOTH_CHIP), rda)
  bdroid_CFLAGS += -DRDA_BT
endif

#ifneq ($(filter sp9820a_4c10_marlin sp9820a_refh10,$(TARGET_BOOTLOADER_BOARD_NAME)),)
#KaiOS do not support WBS, due to they do not realize the WBS/NBS switch callback for the audio(16K/8K).
  bdroid_CFLAGS += -DKAIOS_8K_SAMPLERATE_SUPPORT
#endif

bdroid_CFLAGS += \
  -Wall \
  -Wno-unused-parameter \
  -Wunused-but-set-variable \
  -UNDEBUG \
  -DLOG_NDEBUG=1

include $(call all-subdir-makefiles)

# Cleanup our locals
bdroid_C_INCLUDES :=
bdroid_CFLAGS :=
