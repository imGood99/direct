LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE    := fastgyro
LOCAL_SRC_FILES := fastgyro_native.cpp
LOCAL_LDLIBS    := -landroid -ldl
LOCAL_CPPFLAGS  := -O2 -fno-exceptions -fno-rtti
include $(BUILD_SHARED_LIBRARY)
