// Fast Gyro - versi NDK C++ (ASensorManager / ASensorEventQueue)
#include <jni.h>
#include <android/sensor.h>
#include <android/looper.h>
#include <android/api-level.h>
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <thread>
#include <ctime>
#include <sys/resource.h>

namespace {

constexpr const char *kPkg      = "com.fastgyro.app";
constexpr int         kLooperId = 1;
constexpr int64_t     kCalNs    = 3000000000LL;   // 3 detik

// ---- state sensor ----
ASensorManager     *g_mgr    = nullptr;
const ASensor      *g_gyro   = nullptr;
ALooper            *g_looper = nullptr;
std::mutex          g_lock;
std::condition_variable g_ready;
std::thread         g_thr;
std::atomic<bool>   g_running{false};
bool                g_started = false;

// ---- data (ditulis thread sensor, dibaca thread UI) ----
std::atomic<float>  gx{0}, gy{0}, gz{0}, bx{0}, by{0}, bz{0}, latMs{0}, inHz{0};
std::atomic<unsigned long long> inCount{0};
std::atomic<bool>   calReq{false}, calRun{false};

// hanya thread sensor
int64_t calEnd = 0;
int     calN   = 0;
float   sx = 0, sy = 0, sz = 0;

// hanya thread UI (nativeSnapshot)
long long          lastMs = 0;
unsigned long long lastIn = 0;

int64_t nowBootNs() {
    timespec ts{};
    clock_gettime(CLOCK_BOOTTIME, &ts);   // domain sama dengan ASensorEvent.timestamp
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

void initSensor() {
    if (g_gyro) return;
#if __ANDROID_API__ >= 26
    if (__builtin_available(android 26, *))
        g_mgr = ASensorManager_getInstanceForPackage(kPkg);
#endif
    if (!g_mgr) g_mgr = ASensorManager_getInstance();
    if (g_mgr) g_gyro = ASensorManager_getDefaultSensor(g_mgr, ASENSOR_TYPE_GYROSCOPE);
}

void onEvent(const ASensorEvent &e) {
    const float x = e.vector.x, y = e.vector.y, z = e.vector.z;
    const int64_t ts = e.timestamp;

    const float lat = static_cast<float>(nowBootNs() - ts) / 1000000.0f;
    const float l0  = latMs.load(std::memory_order_relaxed);
    latMs.store(l0 + (lat - l0) * 0.02f, std::memory_order_relaxed);

    if (calReq.exchange(false)) {
        calRun.store(true);
        calEnd = ts + kCalNs;
        calN = 0; sx = sy = sz = 0.f;
    }
    if (calRun.load()) {
        sx += x; sy += y; sz += z; ++calN;
        if (ts >= calEnd) {
            bx.store(sx / calN); by.store(sy / calN); bz.store(sz / calN);
            calRun.store(false);
        }
    }
    gx.store(x - bx.load()); gy.store(y - by.load()); gz.store(z - bz.load());
    inCount.fetch_add(1, std::memory_order_relaxed);
}

void sensorThread() {
    setpriority(PRIO_PROCESS, 0, -8);

    ALooper *looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    ASensorEventQueue *q =
        ASensorManager_createEventQueue(g_mgr, looper, kLooperId, nullptr, nullptr);

    if (q) {
        int period = ASensor_getMinDelay(g_gyro);   // us; rate maksimum
        if (period <= 0) period = 1000;
#if __ANDROID_API__ >= 26
        if (__builtin_available(android 26, *)) {
            ASensorEventQueue_registerSensor(q, g_gyro, period, 0);  // report latency 0
        } else
#endif
        {
            ASensorEventQueue_enableSensor(q, g_gyro);
            ASensorEventQueue_setEventRate(q, g_gyro, period);
        }
    }

    {
        std::lock_guard<std::mutex> lk(g_lock);
        g_looper = looper;
    }
    g_ready.notify_all();

    ASensorEvent buf[64];
    while (q && g_running.load()) {
        const int ident = ALooper_pollOnce(-1, nullptr, nullptr, nullptr);
        if (ident == kLooperId) {
            ssize_t n;
            while ((n = ASensorEventQueue_getEvents(q, buf, 64)) > 0)
                for (ssize_t i = 0; i < n; ++i)
                    if (buf[i].type == ASENSOR_TYPE_GYROSCOPE) onEvent(buf[i]);
        }
    }

    if (q) {
        ASensorEventQueue_disableSensor(q, g_gyro);
        ASensorManager_destroyEventQueue(g_mgr, q);
    }
}

} // namespace

extern "C" {

JNIEXPORT jboolean JNICALL
Java_com_fastgyro_app_MainActivity_nativeStart(JNIEnv *, jobject) {
    std::unique_lock<std::mutex> lk(g_lock);
    initSensor();
    if (!g_gyro)  return JNI_FALSE;
    if (g_started) return JNI_TRUE;

    calReq.store(false);
    calRun.store(false);
    latMs.store(0.f);
    lastMs   = 0;
    g_looper = nullptr;
    g_running.store(true);

    g_thr = std::thread(sensorThread);
    g_ready.wait(lk, [] { return g_looper != nullptr; });
    g_started = true;
    return JNI_TRUE;
}

JNIEXPORT void JNICALL
Java_com_fastgyro_app_MainActivity_nativeStop(JNIEnv *, jobject) {
    {
        std::lock_guard<std::mutex> lk(g_lock);
        if (!g_started) return;
        g_running.store(false);
        ALooper_wake(g_looper);
    }
    g_thr.join();
    std::lock_guard<std::mutex> lk(g_lock);
    g_started = false;
    g_looper  = nullptr;
}

JNIEXPORT void JNICALL
Java_com_fastgyro_app_MainActivity_nativeCalibrate(JNIEnv *, jobject) {
    calReq.store(true);
}

// out[9] = gx gy gz bx by bz inHz latMs calRun
JNIEXPORT void JNICALL
Java_com_fastgyro_app_MainActivity_nativeSnapshot(JNIEnv *env, jobject, jfloatArray out) {
    const long long nowMs = nowBootNs() / 1000000LL;
    const unsigned long long cnt = inCount.load(std::memory_order_relaxed);
    if (lastMs == 0) { lastMs = nowMs; lastIn = cnt; }
    const long long dt = nowMs - lastMs;
    if (dt >= 1000) {
        inHz.store(static_cast<float>(cnt - lastIn) * 1000.0f / static_cast<float>(dt));
        lastMs = nowMs; lastIn = cnt;
    }
    const jfloat v[9] = { gx.load(), gy.load(), gz.load(),
                          bx.load(), by.load(), bz.load(),
                          inHz.load(), latMs.load(),
                          calRun.load() ? 1.0f : 0.0f };
    env->SetFloatArrayRegion(out, 0, 9, v);
}

JNIEXPORT jstring JNICALL
Java_com_fastgyro_app_MainActivity_nativeDescribe(JNIEnv *env, jobject) {
    std::lock_guard<std::mutex> lk(g_lock);
    initSensor();
    char s[256];
    if (!g_gyro)
        std::snprintf(s, sizeof s, "Perangkat ini tidak punya sensor giroskop.");
    else
        std::snprintf(s, sizeof s, "%s\nmin delay: %d us",
                      ASensor_getName(g_gyro), ASensor_getMinDelay(g_gyro));
    return env->NewStringUTF(s);
}

} // extern "C"
