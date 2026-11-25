// ASensorManager + ALooper (event-driven) untuk gyroscope.
// Lib: libochiminative.so  |  Kelas Java: com.ochimii.nativegyro.MainActivity
#include <jni.h>
#include <android/looper.h>
#include <android/sensor.h>
#include <pthread.h>
#include <atomic>

#define LOOPER_ID_SENSOR 1
#define PKG_NAME "com.ochimii.nativegyro"
#define RATE_US 2500   // 400 Hz nominal (sama seperti versi Java: 0x9c4)

static JavaVM*   g_vm        = nullptr;
static jobject   g_activity  = nullptr;   // global ref
static jmethodID g_onSample  = nullptr;   // onNativeSample(JFFF)V
static pthread_t g_thread;
static std::atomic<bool> g_running(false);
static std::atomic<bool> g_started(false);
static ALooper*  g_looper    = nullptr;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cond = PTHREAD_COND_INITIALIZER;
static bool g_ready = false;   // looper+queue siap
static bool g_ok    = false;   // inisialisasi sukses

static JNIEnv* g_env = nullptr; // env milik thread sensor
static ASensorEventQueue* g_queue = nullptr;

// Dipanggil ALooper saat ada event (event-driven, tanpa polling/sleep).
static int sensor_cb(int /*fd*/, int /*events*/, void* /*data*/) {
    ASensorEventQueue* q = g_queue;
    if (!q) return 1;
    ASensorEvent ev[32];
    ssize_t n;
    while ((n = ASensorEventQueue_getEvents(q, ev, 32)) > 0) {
        for (ssize_t i = 0; i < n; i++) {
            if (ev[i].type != ASENSOR_TYPE_GYROSCOPE) continue;
            g_env->CallVoidMethod(g_activity, g_onSample,
                                  (jlong)ev[i].timestamp,
                                  (jfloat)ev[i].vector.x,
                                  (jfloat)ev[i].vector.y,
                                  (jfloat)ev[i].vector.z);
            if (g_env->ExceptionCheck()) g_env->ExceptionClear();
        }
    }
    return 1; // tetap terdaftar
}

static void* sensor_thread(void*) {
    JNIEnv* env = nullptr;
    if (g_vm->AttachCurrentThread(&env, nullptr) != JNI_OK) {
        pthread_mutex_lock(&g_lock); g_ready = true; g_ok = false;
        pthread_cond_signal(&g_cond); pthread_mutex_unlock(&g_lock);
        return nullptr;
    }
    g_env = env;

    ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    ASensorManager* mgr = ASensorManager_getInstanceForPackage(PKG_NAME);
    const ASensor* gyro = mgr ? ASensorManager_getDefaultSensor(mgr, ASENSOR_TYPE_GYROSCOPE) : nullptr;
    ASensorEventQueue* queue = nullptr;
    bool ok = false;

    if (looper && mgr && gyro) {
        queue = ASensorManager_createEventQueue(mgr, looper, LOOPER_ID_SENSOR, sensor_cb, nullptr);
        if (queue) {
            g_queue = queue;
            if (ASensorEventQueue_enableSensor(queue, gyro) >= 0) {
                ASensorEventQueue_setEventRate(queue, gyro, RATE_US);
                ok = true;
            }
        }
    }

    pthread_mutex_lock(&g_lock);
    g_looper = looper; g_ok = ok; g_ready = true;
    pthread_cond_signal(&g_cond);
    pthread_mutex_unlock(&g_lock);

    if (ok) {
        while (g_running.load()) {
            // blok sampai ada event / ALooper_wake()
            int ident = ALooper_pollOnce(-1, nullptr, nullptr, nullptr);
            if (ident == ALOOPER_POLL_ERROR) break;
        }
        ASensorEventQueue_disableSensor(queue, gyro);
    }
    g_queue = nullptr;
    if (queue) ASensorManager_destroyEventQueue(mgr, queue);

    g_env = nullptr;
    g_vm->DetachCurrentThread();
    return nullptr;
}

extern "C" JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void*) {
    g_vm = vm;
    return JNI_VERSION_1_6;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_ochimii_nativegyro_MainActivity_nativeStart(JNIEnv* env, jobject thiz) {
    if (g_started.exchange(true)) return JNI_TRUE;

    jclass cls = env->GetObjectClass(thiz);
    g_onSample = env->GetMethodID(cls, "onNativeSample", "(JFFF)V");
    if (!g_onSample) { env->ExceptionClear(); g_started = false; return JNI_FALSE; }
    g_activity = env->NewGlobalRef(thiz);

    g_ready = false; g_ok = false; g_running = true;
    if (pthread_create(&g_thread, nullptr, sensor_thread, nullptr) != 0) {
        env->DeleteGlobalRef(g_activity); g_activity = nullptr;
        g_running = false; g_started = false;
        return JNI_FALSE;
    }
    pthread_mutex_lock(&g_lock);
    while (!g_ready) pthread_cond_wait(&g_cond, &g_lock);
    bool ok = g_ok;
    pthread_mutex_unlock(&g_lock);

    if (!ok) {
        pthread_join(g_thread, nullptr);
        env->DeleteGlobalRef(g_activity); g_activity = nullptr;
        g_running = false; g_started = false;
        return JNI_FALSE;
    }
    return JNI_TRUE;
}

extern "C" JNIEXPORT void JNICALL
Java_com_ochimii_nativegyro_MainActivity_nativeStop(JNIEnv* env, jobject) {
    if (!g_started.load()) return;
    g_running = false;
    if (g_looper) ALooper_wake(g_looper);
    pthread_join(g_thread, nullptr);
    if (g_activity) { env->DeleteGlobalRef(g_activity); g_activity = nullptr; }
    g_looper = nullptr;
    g_started = false;
}
