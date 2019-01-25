LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)
#
# FM rds lib
#
LOCAL_MODULE := libbt-fmrds
LOCAL_MODULE_CLASS :=  STATIC_LIBRARIES
LOCAL_SRC_FILES := ./libbt-fmrds.a
LOCAL_MODULE_SUFFIX := .a
LOCAL_MULTILIB := 32
include $(BUILD_PREBUILT)
