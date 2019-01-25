# © 2017 KAI OS TECHNOLOGIES (HONG KONG) LIMITED, all rights reserved.
LOCAL_PATH := $(call my-dir)

gps_api_inc := $(LOCAL_PATH)/../../.././hardware/libhardware/include/hardware/

include $(CLEAR_VARS)
LOCAL_MODULE := gps_test
LOCAL_C_INCLUDES := $(gps_api_inc)
LOCAL_SRC_FILES := gps_test.c
LOCAL_MODULE_TAGS := optional

LOCAL_SHARED_LIBRARIES := liblog libcutils libhardware libutils


include $(BUILD_EXECUTABLE)

