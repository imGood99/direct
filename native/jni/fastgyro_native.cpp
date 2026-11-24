// Fast Gyro - jalur sensor native (NDK ASensorManager) v2
// - thread native + ALooper, maxBatchReportLatency = 0 (tanpa batching)
// - opsi CPU affinity (kunci ke core tercepat) dan busy-poll (bisa ON/OFF live)
// - mengukur "latensi kirim" = waktu terima - event.timestamp
#include <jni.h>
#include <android/looper.h>
#include <android/sensor.h>
#include <dlfcn.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/resource.h>
#include <unistd.h>
#include <atomic>
#include <stdint.h>

#define PKG "com.fastgyro.app"
#define LOOPER_ID 1

typedef ASensorManager* (*FnGetForPkg)(const char*);
typedef ASensorManager* (*FnGet)();
typedef int (*FnRegister)(ASensorEventQueue*, const ASensor*, int32_t, int64_t);
typedef int (*FnPollOnce)(int, int*, int*, void**);
typedef int (*FnPollAll)(int, int*, int*, void**);

static std::atomic<bool>     g_run{false};
static std::atomic<int>      g_state{0};          // 0 init, 1 ok, 2 gagal
static std::atomic<ALooper*> g_looper{nullptr};
static std::atomic<float>    g_hz{0.f};
static std::atomic<int>      g_total{0};
static std::atomic<int>      g_latUs{0};          // rata-rata latensi kirim (us)
static std::atomic<int>      g_mode{0};
static std::atomic<bool>     g_aff{true};         // kunci ke core cepat
static std::atomic<bool>     g_busy{false};       // busy-poll
static const int             g_batchUs = 0;       // maxReportLatency
static char                  g_cores[64] = "n/a"; // core terpilih (info)
static pthread_t             g_thread;
static bool                  g_started = false;

static int32_t periodFor(int mode, int32_t minDelay) {
    int32_t p;
    switch (mode) {
        case 0:  p = 0;      break;   // FASTEST
        case 1:  p = 20000;  break;   // GAME
        case 2:  p = 66667;  break;   // UI
        default: p = 200000; break;   // NORMAL
    }
    return p < minDelay ? minDelay : p;
}

static int64_t nowBoottimeNs() {
    timespec ts; clock_gettime(CLOCK_BOOTTIME, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

// Pilih core dengan max freq >= 75% dari yang tertinggi.
static bool buildBigMask(cpu_set_t* big, cpu_set_t* all) {
    int n = (int)sysconf(_SC_NPROCESSORS_CONF);
    if (n < 1) n = 1;
    if (n > CPU_SETSIZE) n = CPU_SETSIZE;
    long freq[CPU_SETSIZE]; long top = 0;
    CPU_ZERO(all); CPU_ZERO(big);
    for (int i = 0; i < n; i++) {
        CPU_SET(i, all);
        char path[96];
        snprintf(path, sizeof path, "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_max_freq", i);
        freq[i] = 0;
        FILE* f = fopen(path, "r");
        if (f) { long v = 0; if (fscanf(f, "%ld", &v) == 1) freq[i] = v; fclose(f); }
        if (freq[i] > top) top = freq[i];
    }
    if (top <= 0) return false;
    char* p = g_cores; size_t left = sizeof g_cores; g_cores[0] = 0;
    for (int i = 0; i < n; i++) {
        if (freq[i] * 4 >= top * 3) {
            CPU_SET(i, big);
            int w = snprintf(p, left, "%s%d", p == g_cores ? "" : ",", i);
            if (w > 0 && (size_t)w < left) { p += w; left -= w; }
        }
    }
    return true;
}

static void* worker(void*) {
    setpriority(PRIO_PROCESS, 0, -4);

    cpu_set_t big, all;
    bool haveBig = buildBigMask(&big, &all);

    ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    g_looper.store(looper);

    void* lib = dlopen("libandroid.so", RTLD_NOW);
    ASensorManager* mgr = nullptr;
    FnRegister regSensor = nullptr;
    FnPollOnce pollOnce = nullptr;
    FnPollAll  pollAll  = nullptr;
    if (lib) {
        FnGetForPkg f1 = (FnGetForPkg)dlsym(lib, "ASensorManager_getInstanceForPackage");
        if (f1) mgr = f1(PKG);
        if (!mgr) { FnGet f2 = (FnGet)dlsym(lib, "ASensorManager_getInstance"); if (f2) mgr = f2(); }
        regSensor = (FnRegister)dlsym(lib, "ASensorEventQueue_registerSensor");
        pollOnce  = (FnPollOnce)dlsym(lib, "ALooper_pollOnce");
        pollAll   = (FnPollAll)dlsym(lib, "ALooper_pollAll");
    }

    const ASensor* gyro = mgr ? ASensorManager_getDefaultSensor(mgr, ASENSOR_TYPE_GYROSCOPE) : nullptr;
    ASensorEventQueue* q = gyro ? ASensorManager_createEventQueue(mgr, looper, LOOPER_ID, nullptr, nullptr) : nullptr;
    if (!q || (!pollOnce && !pollAll)) {
        g_looper.store(nullptr);                 // perbaikan bug: jangan tinggalkan pointer basi
        g_state.store(2);
        return nullptr;
    }

    int32_t period = periodFor(g_mode.load(), ASensor_getMinDelay(gyro));
    int rc;
    if (regSensor) {
        rc = regSensor(q, gyro, period, g_batchUs);      // latensi batch = 0
    } else {
        rc = ASensorEventQueue_enableSensor(q, gyro);
        if (rc >= 0) rc = ASensorEventQueue_setEventRate(q, gyro, period);
    }
    if (rc < 0) {
        ASensorManager_destroyEventQueue(mgr, q);
        g_looper.store(nullptr);                 // perbaikan bug
        g_state.store(2);
        return nullptr;
    }

    g_hz.store(0.f); g_total.store(0); g_latUs.store(0);
    g_state.store(1);

    ASensorEvent ev[64];
    int64_t winStart = 0; int winCount = 0;
    int64_t latSumNs = 0; int latCount = 0;
    int affApplied = -1;

    while (g_run.load()) {
        bool wantAff = g_aff.load();
        if ((int)wantAff != affApplied) {
            if (wantAff && haveBig) sched_setaffinity(0, sizeof big, &big);
            else                    sched_setaffinity(0, sizeof all, &all);
            affApplied = (int)wantAff;
        }
        int timeout = g_busy.load() ? 0 : -1;
        int id = pollOnce ? pollOnce(timeout, nullptr, nullptr, nullptr)
                          : pollAll(timeout, nullptr, nullptr, nullptr);
        if (id != LOOPER_ID) continue;
        ssize_t n;
        while ((n = ASensorEventQueue_getEvents(q, ev, 64)) > 0) {
            int64_t now = nowBoottimeNs();
            for (ssize_t i = 0; i < n; i++) {
                if (ev[i].type != ASENSOR_TYPE_GYROSCOPE) continue;
                // >>> proses data gyro di sini: ev[i].gyro.x/y/z (rad/s)
                int64_t t = ev[i].timestamp;
                int64_t lat = now - t;
                if (lat >= 0 && lat < 1000000000LL) { latSumNs += lat; latCount++; }
                if (winStart == 0) { winStart = t; winCount = 0; continue; }
                winCount++;
                int64_t dt = t - winStart;
                if (dt >= 1000000000LL) {
                    g_hz.store((float)winCount * 1e9f / (float)dt);
                    g_total.fetch_add(winCount);
                    if (latCount > 0) g_latUs.store((int)(latSumNs / latCount / 1000));
                    latSumNs = 0; latCount = 0;
                    winStart = t; winCount = 0;
                }
            }
        }
    }
    ASensorEventQueue_disableSensor(q, gyro);
    ASensorManager_destroyEventQueue(mgr, q);
    g_looper.store(nullptr);
    return nullptr;
}

static void stopNative() {
    if (!g_started) return;
    g_run.store(false);
    ALooper* l = g_looper.load();
    if (l) ALooper_wake(l);
    pthread_join(g_thread, nullptr);
    g_started = false;
}

extern "C" {

JNIEXPORT jboolean JNICALL
Java_com_fastgyro_app_GyroService_nativeStart(JNIEnv*, jclass, jint mode) {
    stopNative();
    g_mode.store(mode);
    g_state.store(0);
    g_run.store(true);
    if (pthread_create(&g_thread, nullptr, worker, nullptr) != 0) { g_run.store(false); return JNI_FALSE; }
    g_started = true;
    for (int i = 0; i < 1000 && g_state.load() == 0; i++) usleep(1000);
    if (g_state.load() != 1) { stopNative(); return JNI_FALSE; }
    return JNI_TRUE;
}

JNIEXPORT void JNICALL
Java_com_fastgyro_app_GyroService_nativeStop(JNIEnv*, jclass) { stopNative(); }

JNIEXPORT jfloat JNICALL
Java_com_fastgyro_app_GyroService_nativeHz(JNIEnv*, jclass) { return g_hz.load(); }

JNIEXPORT jint JNICALL
Java_com_fastgyro_app_GyroService_nativeTotal(JNIEnv*, jclass) { return g_total.load(); }

JNIEXPORT void JNICALL
Java_com_fastgyro_app_GyroService_nativeSetAffinity(JNIEnv*, jclass, jboolean on) { g_aff.store(on != 0); }

JNIEXPORT void JNICALL
Java_com_fastgyro_app_GyroService_nativeSetBusy(JNIEnv*, jclass, jboolean on) { g_busy.store(on != 0); }

JNIEXPORT jstring JNICALL
Java_com_fastgyro_app_GyroService_nativeStatus(JNIEnv* env, jclass) {
    char buf[512];
    bool aff = g_aff.load();
    snprintf(buf, sizeof buf,
        "Laju sampel    : %.0f Hz\n"
        "Total sampel   : %d\n"
        "Report latency : %d us%s\n"
        "Latensi kirim  : %d us\n"
        "Affinity       : %s%s%s\n"
        "Busy-poll      : %s",
        (double)g_hz.load(), g_total.load(),
        g_batchUs, g_batchUs == 0 ? " (tanpa batching)" : "",
        g_latUs.load(),
        aff ? "ON (core " : "OFF", aff ? g_cores : "", aff ? ")" : "",
        g_busy.load() ? "ON" : "OFF");
    return env->NewStringUTF(buf);
}

}
