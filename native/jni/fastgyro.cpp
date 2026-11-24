#include <jni.h>
#include <android/sensor.h>
#include <android/looper.h>
#include <android/log.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <time.h>

#define LOG_TAG "FastGyroNDK"
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace {
JavaVM* gVm = nullptr;
jobject gActivity = nullptr;                 // Global reference
jmethodID gOnGyro = nullptr;
std::thread gWorker;
std::atomic<bool> gRunning{false};
std::atomic<bool> gCalRequest{false};
std::mutex gMutex;

ASensorManager* gSensorManager = nullptr;
ASensorEventQueue* gQueue = nullptr;
const ASensor* gGyro = nullptr;

float gBiasX = 0.0f, gBiasY = 0.0f, gBiasZ = 0.0f;
float gSumX = 0.0f, gSumY = 0.0f, gSumZ = 0.0f;
int gCalCount = 0;
bool gCalibrating = false;
int64_t gCalEndNs = 0;

int64_t bootTimeNs() {
    timespec ts{};
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

void sendToJava(JNIEnv* env, float x, float y, float z, float latencyMs,
                float bx, float by, float bz, float rateHz,
                int count, bool calibrating) {
    if (gActivity && gOnGyro) {
        env->CallVoidMethod(gActivity, gOnGyro,
                            x, y, z, latencyMs,
                            bx, by, bz, rateHz,
                            static_cast<jint>(count),
                            static_cast<jboolean>(calibrating));
        if (env->ExceptionCheck()) {
            env->ExceptionDescribe();
            env->ExceptionClear();
        }
    }
}

void sensorLoop() {
    JNIEnv* env = nullptr;
    bool attachedHere = false;
    if (gVm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
        if (gVm->AttachCurrentThread(&env, nullptr) != JNI_OK) {
            LOGE("AttachCurrentThread failed");
            gRunning = false;
            return;
        }
        attachedHere = true;
    }

    ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    if (!looper) {
        LOGE("ALooper_prepare failed");
        gRunning = false;
        if (attachedHere) gVm->DetachCurrentThread();
        return;
    }

    gSensorManager = ASensorManager_getInstance();
    if (!gSensorManager) {
        LOGE("ASensorManager_getInstance failed");
        gRunning = false;
        if (attachedHere) gVm->DetachCurrentThread();
        return;
    }

    gGyro = ASensorManager_getDefaultSensor(gSensorManager, ASENSOR_TYPE_GYROSCOPE);
    if (!gGyro) {
        LOGE("Gyroscope not found");
        gRunning = false;
        if (attachedHere) gVm->DetachCurrentThread();
        return;
    }

    constexpr int kLooperIdent = 1;
    gQueue = ASensorManager_createEventQueue(
        gSensorManager, looper, kLooperIdent, nullptr, nullptr);

    if (!gQueue) {
        LOGE("ASensorManager_createEventQueue failed");
        gRunning = false;
        if (attachedHere) gVm->DetachCurrentThread();
        return;
    }

    if (ASensorEventQueue_enableSensor(gQueue, gGyro) < 0) {
        LOGE("enableSensor failed");
        ASensorManager_destroyEventQueue(gSensorManager, gQueue);
        gQueue = nullptr;
        gRunning = false;
        if (attachedHere) gVm->DetachCurrentThread();
        return;
    }

    // 0 us requests the fastest rate, matching SENSOR_DELAY_FASTEST intent.
    ASensorEventQueue_setEventRate(gQueue, gGyro, 0);

    int totalCount = 0;
    int lastRateCount = 0;
    int64_t lastRateNs = bootTimeNs();
    float rateHz = 0.0f;
    float latencyMs = 0.0f;
    bool hasLatency = false;

    while (gRunning.load()) {
        int ident = ALooper_pollOnce(100, nullptr, nullptr, nullptr);
        if (ident == ALOOPER_POLL_ERROR) continue;

        ASensorEvent events[32];
        ssize_t n;
        while ((n = ASensorEventQueue_getEvents(gQueue, events, 32)) > 0) {
            for (ssize_t i = 0; i < n; ++i) {
                const ASensorEvent& ev = events[i];
                if (ev.type != ASENSOR_TYPE_GYROSCOPE) continue;

                const float rawX = ev.vector.x;
                const float rawY = ev.vector.y;
                const float rawZ = ev.vector.z;
                const int64_t nowNs = bootTimeNs();

                float currentLatency =
                    static_cast<float>(nowNs - ev.timestamp) / 1000000.0f;
                if (!hasLatency) {
                    latencyMs = currentLatency;
                    hasLatency = true;
                } else {
                    latencyMs += 0.02f * (currentLatency - latencyMs);
                }

                if (gCalRequest.exchange(false)) {
                    gCalibrating = true;
                    gCalCount = 0;
                    gSumX = gSumY = gSumZ = 0.0f;
                    gCalEndNs = ev.timestamp + 3000000000LL;
                }

                if (gCalibrating) {
                    gSumX += rawX;
                    gSumY += rawY;
                    gSumZ += rawZ;
                    ++gCalCount;

                    if (ev.timestamp >= gCalEndNs && gCalCount > 0) {
                        const float denom = static_cast<float>(gCalCount);
                        gBiasX = gSumX / denom;
                        gBiasY = gSumY / denom;
                        gBiasZ = gSumZ / denom;
                        gCalibrating = false;
                    }
                }

                const float x = rawX - gBiasX;
                const float y = rawY - gBiasY;
                const float z = rawZ - gBiasZ;
                ++totalCount;

                const int64_t rateNowNs = bootTimeNs();
                const int64_t elapsedNs = rateNowNs - lastRateNs;
                if (elapsedNs >= 1000000000LL) {
                    rateHz = static_cast<float>(totalCount - lastRateCount) *
                             1000000000.0f / static_cast<float>(elapsedNs);
                    lastRateCount = totalCount;
                    lastRateNs = rateNowNs;
                }

                sendToJava(env, x, y, z, latencyMs,
                           gBiasX, gBiasY, gBiasZ, rateHz,
                           totalCount, gCalibrating);
            }
        }
    }

    if (gQueue) {
        ASensorEventQueue_disableSensor(gQueue, gGyro);
        ASensorManager_destroyEventQueue(gSensorManager, gQueue);
        gQueue = nullptr;
    }
    gGyro = nullptr;
    gSensorManager = nullptr;

    if (attachedHere) gVm->DetachCurrentThread();
}
} // namespace

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void*) {
    gVm = vm;
    return JNI_VERSION_1_6;
}

// Java declaration:
// private native void nativeStart();
// Call from Activity.onResume().
extern "C" JNIEXPORT void JNICALL
Java_com_fastgyro_app_MainActivity_nativeStart(JNIEnv* env, jobject thiz) {
    std::lock_guard<std::mutex> lock(gMutex);

    if (gRunning.load()) return;

    if (gActivity) {
        env->DeleteGlobalRef(gActivity);
        gActivity = nullptr;
    }
    gActivity = env->NewGlobalRef(thiz);

    jclass cls = env->GetObjectClass(thiz);
    // Java callback required:
    // private void onNativeGyro(float x, float y, float z, float latencyMs,
    //   float bx, float by, float bz, float rateHz, int count, boolean calibrating)
    gOnGyro = env->GetMethodID(cls, "onNativeGyro", "(FFFFFFFFIZ)V");
    env->DeleteLocalRef(cls);

    if (!gOnGyro) {
        LOGE("Missing Java callback onNativeGyro(FFFFFFFFIZ)V");
        env->ExceptionClear();
        env->DeleteGlobalRef(gActivity);
        gActivity = nullptr;
        return;
    }

    gBiasX = gBiasY = gBiasZ = 0.0f;
    gCalRequest = false;
    gRunning = true;
    gWorker = std::thread(sensorLoop);
}

// Java declaration:
// private native void nativeStop();
// Call from Activity.onPause().
extern "C" JNIEXPORT void JNICALL
Java_com_fastgyro_app_MainActivity_nativeStop(JNIEnv* env, jobject) {
    {
        std::lock_guard<std::mutex> lock(gMutex);
        gRunning = false;
    }
    if (gWorker.joinable()) gWorker.join();

    std::lock_guard<std::mutex> lock(gMutex);
    if (gActivity) {
        env->DeleteGlobalRef(gActivity);
        gActivity = nullptr;
    }
    gOnGyro = nullptr;
}

// Java declaration:
// private native void nativeRequestCalibration();
// Call from the calibration button's onClick().
extern "C" JNIEXPORT void JNICALL
Java_com_fastgyro_app_MainActivity_nativeRequestCalibration(JNIEnv*, jobject) {
    gCalRequest = true;
}
