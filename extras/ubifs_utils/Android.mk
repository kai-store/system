# Copyright 2014 The Android Open Source Project

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_CFLAGS += -O2 -Wall
LOCAL_LDLIBS += -lz -lm
LOCAL_SHARED_LIBRARIES := libcutils 
LOCAL_MODULE := mkfs_ubifs
LOCAL_SRC_FILES := \
	ubi-utils/libubi.c \
    lib/libcrc32.c \
    libselinux/src/label_android_property.c \
    libselinux/src/label_file.c \
    libselinux/src/init.c \
    libselinux/src/check_context.c \
    libselinux/src/callbacks.c \
    libselinux/src/label.c \
	mkfs-ubifs/mkfs.ubifs.c \
	mkfs-ubifs/crc16.c \
	mkfs-ubifs/lpt.c \
	mkfs-ubifs/compr.c \
	mkfs-ubifs/devtable.c \
	mkfs-ubifs/hashtable/hashtable.c \
	mkfs-ubifs/hashtable/hashtable_itr.c \
	lzo-2.06/src/lzo1x_9x.c \
	uuid/gen_uuid.c \
	uuid/unpack.c \
	uuid/pack.c \
	uuid/unparse.c
LOCAL_C_INCLUDES += \
	$(LOCAL_PATH)/include \
	$(LOCAL_PATH)/ubi-utils/include \
	$(LOCAL_PATH)/ubi-utils/src \
	$(LOCAL_PATH)/libselinux/include \
	$(LOCAL_PATH)/lzo-2.06/include \
	$(LOCAL_PATH)/uuid/ 
include $(BUILD_HOST_EXECUTABLE)


#==================
include $(CLEAR_VARS)
LOCAL_SRC_FILES:= \
    lib/libcrc32.c \
    ubi-utils/ubiutils-common.c \
	ubi-utils/ubinize.c \
	ubi-utils/libubigen.c \
    ubi-utils/libiniparser.c \
    ubi-utils/dictionary.c    
LOCAL_C_INCLUDES+= \
	$(LOCAL_PATH)/include \
	$(LOCAL_PATH)/ubi-utils/include \
	$(LOCAL_PATH)/ubi-utils/src
LOCAL_MODULE:=ubinize
LOCAL_CFLAGS+=-O3
include $(BUILD_HOST_EXECUTABLE)

