LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE    := fastgyro
LOCAL_SRC_FILES := fastgyro.c
LOCAL_CFLAGS    := -O2 -Wall
LOCAL_LDLIBS    := -landroid
include $(BUILD_SHARED_LIBRARY)
