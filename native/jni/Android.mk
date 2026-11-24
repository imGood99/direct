LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE     := fastgyro
LOCAL_SRC_FILES  := fastgyro.cpp
LOCAL_CPPFLAGS   := -std=c++17 -O2 -Wall -fno-exceptions -fno-rtti
LOCAL_LDLIBS     := -landroid
include $(BUILD_SHARED_LIBRARY)
