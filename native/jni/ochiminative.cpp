// ASensorManager + ALooper (event-driven) untuk gyroscope - versi hemat CPU/baterai.
// Lib: libochiminative.so  |  Kelas Java: com.ochimii.nativegyro.MainActivity
//
// - Sensor tetap disampel 400 Hz (2500 us) oleh hardware.
// - Event dikirim ke app dalam batch (hardware FIFO, max latency ~300 ms) -> CPU
//   hanya bangun ~3x/detik, bukan 400x/detik. Kalau HAL tidak mendukung batching,
//   otomatis fallback ke mode kontinu 400 Hz.
// - Thread sensor TIDAK memanggil JNI/Java sama sekali: hanya update counter.
//   UI (Java) mengambil snapshot via nativeSnapshot() tiap 300 ms.
// - Thread tidur total (ALooper_pollOnce(-1)) saat tidak ada event.
#include <jni.h>
#include <android/looper.h>
#include <android/sensor.h>
#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include <atomic>

#define LOOPER_ID_SENSOR 1
#define PKG_NAME  "com.ochimii.nativegyro"
#define RATE_US   2500      // 400 Hz
#define BATCH_US  300000    // max batch report latency = 300 ms (sama dgn refresh UI)

static pthread_t g_thread;
static std::atomic<bool> g_running(false);
static std::atomic<bool> g_started(false);
static ALooper*           g_looper = nullptr;
static ASensorEventQueue* g_queue  = nullptr;

// handshake inisialisasi thread
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cond = PTHREAD_COND_INITIALIZER;
static bool g_ready = false;
static bool g_ok    = false;

// data terakhir (ditulis thread sensor, dibaca UI)
static pthread_mutex_t g_snap_lock = PTHREAD_MUTEX_INITIALIZER;
static int64_t g_count = 0;
static int64_t g_ts    = 0;
static float   g_x = 0, g_y = 0, g_z = 0;

// Dipanggil ALooper saat ada event. Kerja minimal: hitung + simpan sampel terakhir.
static int sensor_cb(int /*fd*/, int /*events*/, void* /*data*/) {
    ASensorEventQueue* q = g_queue;
    if (!q) return 1;
    ASensorEvent ev[64];
    int64_t add = 0, ts = 0;
    float x = 0, y = 0, z = 0;
    ssize_t n;
    while ((n = ASensorEventQueue_getEvents(q, ev, 64)) > 0) {
        for (ssize_t i = 0; i < n; i++) {
            if (ev[i].type != ASENSOR_TYPE_GYROSCOPE) continue;
            add++;
            ts = ev[i].timestamp;
            x = ev[i].vector.x; y = ev[i].vector.y; z = ev[i].vector.z;
        }
    }
    if (add > 0) {
        pthread_mutex_lock(&g_snap_lock);
        g_count += add; g_ts = ts; g_x = x; g_y = y; g_z = z;
        pthread_mutex_unlock(&g_snap_lock);
    }
    return 1; // tetap terdaftar
}

static void* sensor_thread(void*) {
    ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    ASensorManager* mgr = ASensorManager_getInstanceForPackage(PKG_NAME);
    const ASensor* gyro = mgr ? ASensorManager_getDefaultSensor(mgr, ASENSOR_TYPE_GYROSCOPE) : nullptr;
    ASensorEventQueue* queue = nullptr;
    bool ok = false;

    if (looper && mgr && gyro) {
        queue = ASensorManager_createEventQueue(mgr, looper, LOOPER_ID_SENSOR, sensor_cb, nullptr);
        if (queue) {
            g_queue = queue;
            // 400 Hz di hardware + batching ~300 ms
            int r = ASensorEventQueue_registerSensor(queue, gyro, RATE_US, BATCH_US);
            if (r < 0) {
                // fallback: kontinu 400 Hz
                r = ASensorEventQueue_enableSensor(queue, gyro);
                if (r >= 0) ASensorEventQueue_setEventRate(queue, gyro, RATE_US);
            }
            ok = (r >= 0);
        }
    }

    pthread_mutex_lock(&g_lock);
    g_looper = looper; g_ok = ok; g_ready = true;
    pthread_cond_signal(&g_cond);
    pthread_mutex_unlock(&g_lock);

    if (ok) {
        while (g_running.load()) {
            // tidur sampai ada event / ALooper_wake()
            int ident = ALooper_pollOnce(-1, nullptr, nullptr, nullptr);
            if (ident == ALOOPER_POLL_ERROR) break;
        }
        ASensorEventQueue_disableSensor(queue, gyro);
    }
    g_queue = nullptr;
    if (queue) ASensorManager_destroyEventQueue(mgr, queue);
    return nullptr;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_ochimii_nativegyro_MainActivity_nativeStart(JNIEnv*, jobject) {
    if (g_started.exchange(true)) return JNI_TRUE;

    g_ready = false; g_ok = false; g_running = true;
    if (pthread_create(&g_thread, nullptr, sensor_thread, nullptr) != 0) {
        g_running = false; g_started = false;
        return JNI_FALSE;
    }
    pthread_mutex_lock(&g_lock);
    while (!g_ready) pthread_cond_wait(&g_cond, &g_lock);
    bool ok = g_ok;
    pthread_mutex_unlock(&g_lock);

    if (!ok) {
        pthread_join(g_thread, nullptr);
        g_running = false; g_started = false;
        return JNI_FALSE;
    }
    return JNI_TRUE;
}

extern "C" JNIEXPORT void JNICALL
Java_com_ochimii_nativegyro_MainActivity_nativeStop(JNIEnv*, jobject) {
    if (!g_started.load()) return;
    g_running = false;
    if (g_looper) ALooper_wake(g_looper);
    pthread_join(g_thread, nullptr);
    g_looper = nullptr;
    g_started = false;
}

// out[0]=count, out[1]=timestamp(ns), out[2..4]=bit float x,y,z
extern "C" JNIEXPORT void JNICALL
Java_com_ochimii_nativegyro_MainActivity_nativeSnapshot(JNIEnv* env, jobject, jlongArray out) {
    if (!out || env->GetArrayLength(out) < 5) return;
    int32_t xb, yb, zb;
    jlong v[5];
    pthread_mutex_lock(&g_snap_lock);
    v[0] = g_count; v[1] = g_ts;
    memcpy(&xb, &g_x, 4); memcpy(&yb, &g_y, 4); memcpy(&zb, &g_z, 4);
    pthread_mutex_unlock(&g_snap_lock);
    v[2] = xb; v[3] = yb; v[4] = zb;
    env->SetLongArrayRegion(out, 0, 5, v);
}
