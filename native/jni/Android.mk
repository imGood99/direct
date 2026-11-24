LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := fastgyro
LOCAL_SRC_FILES := fastgyro.cpp
LOCAL_CPPFLAGS := -std=c++17 -O2 -Wall
LOCAL_LDLIBS := -landroid -llog
include $(BUILD_SHARED_LIBRARY)
