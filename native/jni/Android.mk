LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE    := ochiminative
LOCAL_SRC_FILES := ochiminative.cpp
LOCAL_LDLIBS    := -landroid -llog
include $(BUILD_SHARED_LIBRARY)
