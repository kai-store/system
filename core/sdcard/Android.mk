LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)


ifeq ($(TARGET_USE_SDCARDFS),true)
LOCAL_SRC_FILES := sdcard_sdcardfs.c
else
LOCAL_SRC_FILES := sdcard.c
endif

LOCAL_MODULE := sdcard
LOCAL_CFLAGS := -Wall -Wno-unused-parameter -Werror

LOCAL_SHARED_LIBRARIES := libcutils

include $(BUILD_EXECUTABLE)
