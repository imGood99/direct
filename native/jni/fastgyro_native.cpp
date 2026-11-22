// Fast Gyro - jalur sensor native (NDK)
// Jalur 1: ASensorManager + ALooper (lewat sensorservice), report latency = 0.
// Jalur 2 (mode DIRECT): direct channel shared memory (API 26+), tanpa sensorservice/socket.
//          Kalau tidak didukung -> otomatis fallback ke jalur 1.
// Statistik latensi: min / rata-rata / p99 / max per jendela 1 detik.
#include <jni.h>
#include <android/looper.h>
#include <android/sensor.h>
#include <dlfcn.h>
#include <pthread.h>
#include <sys/resource.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <atomic>
#include <stdint.h>

#define PKG "com.fastgyro.app"
#define LOOPER_ID 1
#define MODE_DIRECT 4

// konstanta direct channel (didefinisikan manual agar tetap build di API 21)
#define DC_TYPE_SHARED_MEMORY 1
#define DC_RATE_STOP 0
#define EV_SIZE 104          // sizeof(sensors_event_t) di format direct channel
#define EV_SLOTS 512
#define LAT_MAX 4096

typedef ASensorManager* (*FnGetForPkg)(const char*);
typedef ASensorManager* (*FnGet)();
typedef int (*FnRegister)(ASensorEventQueue*, const ASensor*, int32_t, int64_t);
typedef int (*FnPollOnce)(int, int*, int*, void**);
typedef int (*FnPollAll)(int, int*, int*, void**);
typedef int (*FnShmCreate)(const char*, size_t);
typedef int (*FnDcCreate)(ASensorManager*, int, size_t);
typedef int (*FnDcConfig)(ASensorManager*, const ASensor*, int, int);
typedef void (*FnDcDestroy)(ASensorManager*, int);
typedef bool (*FnDcSupported)(const ASensor*, int);
typedef int (*FnDcHighest)(const ASensor*);

static std::atomic<bool>   g_run{false};
static std::atomic<int>    g_state{0};          // 0 init, 1 ok, 2 gagal
static std::atomic<ALooper*> g_looper{nullptr};
static std::atomic<float>  g_hz{0.f};
static std::atomic<int>    g_total{0};
static std::atomic<float>  g_lat{0.f};          // rata-rata (us)
static std::atomic<int>    g_lmin{0}, g_lmax{0}, g_lp99{0};
static std::atomic<int>    g_path{0};           // 0 -, 1 sensorservice, 2 direct, 3 direct gagal -> fallback
static std::atomic<int>    g_mode{0};
static pthread_t           g_thread;
static bool                g_started = false;

static const int64_t kMaxReportLatencyUs = 0;

static inline int64_t nowNs() {
    timespec ts; clock_gettime(CLOCK_BOOTTIME, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static int32_t periodFor(int mode, int32_t minDelay) {
    int32_t p;
    switch (mode) {
        case 0:  p = 0;      break;
        case 1:  p = 20000;  break;
        case 2:  p = 66667;  break;
        case 3:  p = 200000; break;
        default: p = 0;      break;   // DIRECT yang gagal -> fallback FASTEST
    }
    return p < minDelay ? minDelay : p;
}

// ---- statistik jendela 1 detik ----
struct Stats {
    int64_t winStart = 0;
    int cnt = 0;
    int nstore = 0;
    double sum = 0;
    int32_t mn = 0x7fffffff, mx = 0;
    int32_t lat[LAT_MAX];

    void add(int64_t now, int64_t t) {
        g_total.fetch_add(1);
        int64_t d = (now - t) / 1000;                 // us
        int32_t l = d < 0 ? 0 : (d > 2000000 ? 2000000 : (int32_t)d);
        if (winStart == 0) { winStart = t; return; }
        cnt++; sum += l;
        if (l < mn) mn = l;
        if (l > mx) mx = l;
        if (nstore < LAT_MAX) lat[nstore++] = l;
        int64_t dt = t - winStart;
        if (dt >= 1000000000LL) {
            g_hz.store((float)cnt * 1e9f / (float)dt);
            g_lat.store((float)(sum / cnt));
            g_lmin.store(mn); g_lmax.store(mx);
            std::sort(lat, lat + nstore);
            g_lp99.store(lat[(nstore * 99) / 100 >= nstore ? nstore - 1 : (nstore * 99) / 100]);
            winStart = t; cnt = 0; nstore = 0; sum = 0; mn = 0x7fffffff; mx = 0;
        }
    }
};

static void setHighPriority() {
    // coba dari paling tinggi; yang ditolak sistem dilewati
    static const int prios[] = {-19, -16, -10, -4};
    for (int p : prios) if (setpriority(PRIO_PROCESS, 0, p) == 0) return;
}

// ---- jalur DIRECT CHANNEL ----
// Return true kalau berhasil jalan sampai g_run=false. false = tidak didukung/gagal (caller fallback).
static bool runDirect(void* lib, ASensorManager* mgr, const ASensor* gyro, Stats& st) {
    FnShmCreate shmCreate = (FnShmCreate)dlsym(lib, "ASharedMemory_create");
    FnDcCreate  dcCreate  = (FnDcCreate)dlsym(lib, "ASensorManager_createSharedMemoryDirectChannel");
    FnDcConfig  dcConfig  = (FnDcConfig)dlsym(lib, "ASensorManager_configureDirectReport");
    FnDcDestroy dcDestroy = (FnDcDestroy)dlsym(lib, "ASensorManager_destroyDirectChannel");
    FnDcSupported dcSup   = (FnDcSupported)dlsym(lib, "ASensor_isDirectChannelTypeSupported");
    FnDcHighest dcHigh    = (FnDcHighest)dlsym(lib, "ASensor_getHighestDirectReportRateLevel");
    if (!shmCreate || !dcCreate || !dcConfig || !dcDestroy || !dcSup || !dcHigh) return false;
    if (!dcSup(gyro, DC_TYPE_SHARED_MEMORY)) return false;
    int rate = dcHigh(gyro);
    if (rate <= 0) return false;

    const size_t size = (size_t)EV_SIZE * EV_SLOTS;
    int fd = shmCreate("fastgyro_dc", size);
    if (fd < 0) return false;
    void* mem = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mem == MAP_FAILED) { close(fd); return false; }
    memset(mem, 0, size);

    int ch = dcCreate(mgr, fd, size);
    if (ch <= 0) { munmap(mem, size); close(fd); return false; }
    int token = dcConfig(mgr, gyro, ch, rate);
    if (token <= 0) { dcDestroy(mgr, ch); munmap(mem, size); close(fd); return false; }

    g_hz.store(0.f); g_total.store(0); g_lat.store(0.f);
    g_lmin.store(0); g_lmax.store(0); g_lp99.store(0);
    g_path.store(2);
    g_state.store(1);

    // Format tiap slot (104 byte): [0]size [4]token [8]type [12]atomic counter [16]timestamp(int64) [24..]data
    uint8_t* base = (uint8_t*)mem;
    uint32_t expected = 1;
    struct timespec nap = {0, 100000};            // 100 us
    while (g_run.load()) {
        bool got = false;
        for (;;) {
            uint8_t* slot = base + (size_t)((expected - 1) % EV_SLOTS) * EV_SIZE;
            uint32_t c = __atomic_load_n((uint32_t*)(slot + 12), __ATOMIC_ACQUIRE);
            if (c == 0) break;
            if (c != expected) {
                if ((int32_t)(c - expected) > 0) expected = c;   // ketinggalan -> sinkron ulang
                else break;                                       // belum ditulis
            }
            int64_t t; memcpy(&t, slot + 16, sizeof(t));
            st.add(nowNs(), t);
            // gyro: float x,y,z di slot+24, slot+28, slot+32 (rad/s)
            expected++;
            got = true;
        }
        if (!got) nanosleep(&nap, nullptr);
    }

    dcConfig(mgr, gyro, ch, DC_RATE_STOP);
    dcDestroy(mgr, ch);
    munmap(mem, size);
    close(fd);
    return true;
}

static void* worker(void*) {
    setHighPriority();

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
    if (!gyro) { g_state.store(2); return nullptr; }

    static Stats st;                                // di luar stack thread (array besar)
    st = Stats();

    int mode = g_mode.load();
    if (mode == MODE_DIRECT) {
        if (runDirect(lib, mgr, gyro, st)) { g_looper.store(nullptr); return nullptr; }
        g_path.store(3);                            // direct gagal -> lanjut jalur biasa
    } else {
        g_path.store(1);
    }

    ASensorEventQueue* q = ASensorManager_createEventQueue(mgr, looper, LOOPER_ID, nullptr, nullptr);
    if (!q || (!pollOnce && !pollAll)) { g_state.store(2); return nullptr; }

    int32_t period = periodFor(mode, ASensor_getMinDelay(gyro));
    int rc;
    if (regSensor) {
        rc = regSensor(q, gyro, period, kMaxReportLatencyUs);
    } else {
        rc = ASensorEventQueue_enableSensor(q, gyro);
        if (rc >= 0) rc = ASensorEventQueue_setEventRate(q, gyro, period);
    }
    if (rc < 0) { ASensorManager_destroyEventQueue(mgr, q); g_state.store(2); return nullptr; }

    g_hz.store(0.f); g_total.store(0); g_lat.store(0.f);
    g_lmin.store(0); g_lmax.store(0); g_lp99.store(0);
    g_state.store(1);

    ASensorEvent ev[64];
    while (g_run.load()) {
        int id = pollOnce ? pollOnce(-1, nullptr, nullptr, nullptr)
                          : pollAll(-1, nullptr, nullptr, nullptr);
        if (id != LOOPER_ID) continue;
        ssize_t n;
        while ((n = ASensorEventQueue_getEvents(q, ev, 64)) > 0) {
            const int64_t now = nowNs();
            for (ssize_t i = 0; i < n; i++) {
                if (ev[i].type != ASENSOR_TYPE_GYROSCOPE) continue;
                // >>> proses data gyro di sini: ev[i].gyro.x/y/z (rad/s), ev[i].timestamp (ns)
                st.add(now, ev[i].timestamp);
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
    g_path.store(0);
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

JNIEXPORT jfloat JNICALL
Java_com_fastgyro_app_GyroService_nativeLatencyUs(JNIEnv*, jclass) { return g_lat.load(); }

JNIEXPORT jint JNICALL
Java_com_fastgyro_app_GyroService_nativeTotal(JNIEnv*, jclass) { return g_total.load(); }

// Teks tambahan untuk layar: min/p99/max dan jalur yang dipakai
JNIEXPORT jstring JNICALL
Java_com_fastgyro_app_GyroService_nativeInfo(JNIEnv* env, jclass) {
    const char* path;
    switch (g_path.load()) {
        case 1:  path = "sensorservice"; break;
        case 2:  path = "DIRECT channel"; break;
        case 3:  path = "sensorservice (direct tidak didukung)"; break;
        default: path = "-"; break;
    }
    char buf[256];
    snprintf(buf, sizeof(buf), "Min / P99 / Max  : %d / %d / %d us\nJalur            : %s",
             g_lmin.load(), g_lp99.load(), g_lmax.load(), path);
    return env->NewStringUTF(buf);
}

}
