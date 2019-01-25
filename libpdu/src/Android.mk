LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := pdu.c \
                   pdubuf.c
LOCAL_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_CFLAGS := -DANDROID_VERSION=$(PLATFORM_SDK_VERSION) \
                -Wall -Werror
LOCAL_SHARED_LIBRARIES := liblog
LOCAL_MODULE := libpdu
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_TAGS := optional
include $(BUILD_SHARED_LIBRARY)
