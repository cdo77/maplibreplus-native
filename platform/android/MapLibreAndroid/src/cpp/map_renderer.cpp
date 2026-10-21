#include <algorithm>
#include <mln/renderer/update_parameters.hpp>
#include <chrono>
#include <GLES3/gl3.h>
#include <mln/util/render_stage.hpp>
#include "map_renderer.hpp"

#include <mln/gfx/backend_scope.hpp>
#include <mln/renderer/renderer.hpp>
#include <mln/util/instrumentation.hpp>
#include <mln/util/logging.hpp>
#include <mln/util/run_loop.hpp>

#include <string>
#include <android/native_window_jni.h>

#include "attach_env.hpp"
#include "android_renderer_backend.hpp"
#include "map_renderer_runnable.hpp"

#if MLN_RENDER_BACKEND_OPENGL
#include <sys/system_properties.h>

namespace {
std::string androidSysProp(const char* key) {
    assert(strlen(key) < PROP_NAME_MAX);
    if (__system_property_find(key) == nullptr) {
        return "";
    }
    char prop[PROP_VALUE_MAX + 1];
    __system_property_get(key, prop);
    return prop;
}

bool inEmulator() {
    return androidSysProp("ro.kernel.qemu") == "1" || androidSysProp("ro.boot.qemu") == "1" ||
           androidSysProp("ro.hardware.egl") == "emulation";
}
} // namespace
#endif

namespace mln {
namespace android {

namespace {
// Traza de cuadros (diagnostico de campo): los ultimos 64 cuadros con su costo, camara, error de GL y estado de GL al terminar. Se vuelca al log ante una anomalia (excepcion, error de GL,
// cuadro de mas de 250 ms o un FBO distinto del de pantalla al terminar) para ver el estado en el momento del fallo y no solo el mensaje.
struct FrameRecord {
    uint64_t number = 0;
    int64_t micros = 0;
    double zoom = 0.0;
    double pitchDeg = 0.0;
    uint32_t glError = 0;
    int viewport[4] = {0, 0, 0, 0};
    int scissorTest = 0;
    int framebuffer = 0;
    std::string stage;
};
constexpr size_t kTraceSize = 64;
FrameRecord traceRing[kTraceSize];
uint64_t traceFrames = 0;
int64_t lastTraceDumpMs = 0;
int64_t lastDeepEnableMs = 0;
int cleanFramesSinceGlError = 0;

int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void dumpTrace(const std::string& reason) {
    const int64_t now = nowMs();
    if (now - lastTraceDumpMs < 5000) return;
    lastTraceDumpMs = now;
    Log::Error(Event::Android, "TRAZA-RENDER motivo: " + reason + " | " + util::RenderStage::describe() + " | " + util::RenderStage::describeGl());
    const size_t count = std::min<uint64_t>(traceFrames, 24);
    for (size_t i = 0; i < count; ++i) {
        const FrameRecord& r = traceRing[(traceFrames - count + i) % kTraceSize];
        Log::Error(Event::Android,
                   "TRAZA-RENDER cuadro=" + std::to_string(r.number) + " us=" + std::to_string(r.micros) + " zoom=" + std::to_string(r.zoom) + " inclin=" + std::to_string(r.pitchDeg) +
                       " gl_error=" + std::to_string(r.glError) + " vp=" + std::to_string(r.viewport[0]) + "," + std::to_string(r.viewport[1]) + "," + std::to_string(r.viewport[2]) + "," +
                       std::to_string(r.viewport[3]) + " scissor=" + std::to_string(r.scissorTest) + " fbo=" + std::to_string(r.framebuffer) + " etapa=" + r.stage);
    }
}
} // namespace

MapRenderer::MapRenderer(jni::JNIEnv& _env,
                         const jni::Object<MapRenderer>& obj,
                         jni::jfloat pixelRatio_,
                         const jni::String& localIdeographFontFamily_)
    : javaPeer(_env, obj),
      pixelRatio(pixelRatio_),
      localIdeographFontFamily(localIdeographFontFamily_ ? jni::Make<std::string>(_env, localIdeographFontFamily_)
                                                         : std::optional<std::string>{}),
      threadPool(Scheduler::GetBackground(), {}),
      mailboxData(this) {}

MapRenderer::MailboxData::MailboxData(Scheduler* scheduler_)
    : scheduler(scheduler_) {
    assert(scheduler);
}

std::shared_ptr<Mailbox> MapRenderer::MailboxData::getMailbox() const noexcept {
    if (!mailbox) {
        mailbox = std::make_shared<Mailbox>(*scheduler);
    }
    return mailbox;
}

MapRenderer::~MapRenderer() = default;

void MapRenderer::reset() {
    try {
        destroyed = true;

        if (renderer) {
            // Make sure to destroy the renderer on the GL Thread
            auto self = ActorRef<MapRenderer>(*this, mailboxData.getMailbox());
            self.ask(&MapRenderer::resetRenderer).wait();
        }

        // Lock to make sure there is no concurrent initialisation on the gl thread
        std::scoped_lock lock(initialisationMutex);
        rendererObserver.reset();
    } catch (const std::exception& exception) {
        Log::Error(Event::Android, std::string("MapRenderer::reset failed: ") + exception.what());
    }
}

ActorRef<Renderer> MapRenderer::actor() const {
    return *rendererRef;
}

void MapRenderer::schedule(std::function<void()>&& scheduled) {
    MLN_TRACE_FUNC();
    try {
        // Create a runnable
        android::UniqueEnv _env = android::AttachEnv();
        auto runnable = std::make_unique<MapRendererRunnable>(*_env, std::move(scheduled));

        // Obtain ownership of the peer (gets transferred to the MapRenderer on the JVM for later GC)
        auto peer = runnable->peer();

        // Queue the event on the Java Peer
        static auto& javaClass = jni::Class<MapRenderer>::Singleton(*_env);
        static auto queueEvent = javaClass.GetMethod<void(jni::Object<MapRendererRunnable>)>(*_env, "queueEvent");
        auto weakReference = javaPeer.get(*_env);
        if (weakReference) {
            MLN_TRACE_ZONE(java);
            weakReference.Call(*_env, queueEvent, peer);
        }

        // Release the c++ peer as it will be destroyed on GC of the Java Peer
        runnable.release();
    } catch (const std::exception& exception) {
        Log::Error(Event::Android, std::string("MapRenderer::schedule failed: ") + exception.what());
    }
}

void MapRenderer::waitForEmpty([[maybe_unused]] const util::SimpleIdentity tag) {
    try {
        android::UniqueEnv _env = android::AttachEnv();
        static auto& javaClass = jni::Class<MapRenderer>::Singleton(*_env);
        static auto waitForEmpty = javaClass.GetMethod<void()>(*_env, "waitForEmpty");
        if (auto weakReference = javaPeer.get(*_env)) {
            return weakReference.Call(*_env, waitForEmpty);
        }
    } catch (...) {
        Log::Error(Event::Android, "MapRenderer::waitForEmpty failed");
        jni::ThrowJavaError(*android::AttachEnv(), std::current_exception());
    }
}

void MapRenderer::requestRender() {
    try {
        android::UniqueEnv _env = android::AttachEnv();
        static auto& javaClass = jni::Class<MapRenderer>::Singleton(*_env);
        static auto onInvalidate = javaClass.GetMethod<void()>(*_env, "requestRender");
        auto weakReference = javaPeer.get(*_env);
        if (weakReference) {
            weakReference.Call(*_env, onInvalidate);
        }
    } catch (const std::exception& exception) {
        Log::Error(Event::Android, std::string("MapRenderer::requestRender failed: ") + exception.what());
    }
}

void MapRenderer::requestRender(JNIEnv& env, jni::Local<jni::Object<MapRenderer>>& ref) {
    try {
        static auto& javaClass = jni::Class<MapRenderer>::Singleton(env);
        static auto onInvalidate = javaClass.GetMethod<void()>(env, "requestRender");
        if (ref) {
            ref.Call(env, onInvalidate);
        }
    } catch (const std::exception& exception) {
        Log::Error(Event::Android, std::string("MapRenderer::requestRender failed: ") + exception.what());
    }
}

void MapRenderer::update(std::shared_ptr<UpdateParameters> params) {
    try {
        // Lock on the parameters
        std::scoped_lock lock(updateMutex);
        updateParameters = std::move(params);
    } catch (const std::exception& exception) {
        Log::Error(Event::Android, std::string("MapRenderer::update failed: ") + exception.what());
    }
}

void MapRenderer::setObserver(std::shared_ptr<RendererObserver> _rendererObserver) {
    try {
        // Lock as the initialization can come from the main thread or the GL thread first
        std::scoped_lock lock(initialisationMutex);

        rendererObserver = std::move(_rendererObserver);

        // Set the new observer on the Renderer implementation
        if (renderer) {
            renderer->setObserver(rendererObserver.get());
        }
    } catch (const std::exception& exception) {
        Log::Error(Event::Android, std::string("MapRenderer::setObserver failed: ") + exception.what());
    }
}

void MapRenderer::requestSnapshot(SnapshotCallback callback) {
    auto self = ActorRef<MapRenderer>(*this, mailboxData.getMailbox());
    self.invoke(
        &MapRenderer::scheduleSnapshot,
        std::make_unique<SnapshotCallback>(
            [&, callback = std::move(callback), runloop = util::RunLoop::Get()](PremultipliedImage image) {
                runloop->invoke(
                    [callback = std::move(callback), image = std::move(image), renderer = std::move(this)]() mutable {
                        if (renderer && !renderer->destroyed) {
                            callback(std::move(image));
                        }
                    });
                snapshotCallback.reset();
            }));
}

// Called on OpenGL thread //

void MapRenderer::resetRenderer() {
    renderer.reset();

    if (!asyncRendererCleanup) {
        backend.reset();
    }

    window.reset();
    swapBehaviorFlush = false;
}

void MapRenderer::scheduleSnapshot(std::unique_ptr<SnapshotCallback> callback) {
    snapshotCallback = std::move(callback);

    if (backend) {
        gfx::BackendScope backendGuard{backend->getImpl()};
        backend->enableFramebufferRead(true);
    }

    requestRender();
}

void MapRenderer::render(JNIEnv&) {
    assert(renderer);

    std::shared_ptr<UpdateParameters> params;
    {
        // Lock on the parameters
        std::unique_lock<std::mutex> lock(updateMutex);
        if (!updateParameters) return;

        // Hold on to the update parameters during render
        params = updateParameters;
    }

    // Activate the backend
    assert(backend);
    gfx::BackendScope backendGuard{backend->getImpl()};

    // Ensure that the "current" scheduler on the render thread is
    // this scheduler.
    Scheduler::SetCurrent(this);

    if (framebufferSizeChanged) {
        backend->updateViewPort();
        framebufferSizeChanged = false;
    }

    const auto frameStart = std::chrono::steady_clock::now();
    try {
        renderer->render(params);
        consecutiveRenderFailures = 0;
    } catch (const std::exception& exception) {
        // El log solo decia "std::bad_alloc": se registra en que etapa del render estaba y con que cantidades. Antes se relanzaba la excepcion: el cuadro quedaba sin dibujar (memoria sin
        // inicializar en pantalla) y, sin nadie que pidiera otro cuadro, la pantalla quedaba congelada con basura hasta el siguiente toque (medido en campo 2026-10-20). Ahora se pide otro
        // cuadro (hasta 30 seguidos; si todos fallan se corta para no girar en vacio).
        ++consecutiveRenderFailures;
        Log::Error(Event::Android,
                   std::string("MapRenderer::render fallo (") + exception.what() + ") en la etapa: " + util::RenderStage::describe() +
                       " seguidos=" + std::to_string(consecutiveRenderFailures));
        dumpTrace(std::string("excepcion ") + exception.what());
        if (consecutiveRenderFailures <= 30) requestRender();
        return;
    }

    // Traza del cuadro: estado de GL al terminar y costo.
    {
        FrameRecord& r = traceRing[traceFrames % kTraceSize];
        r.number = ++traceFrames;
        r.micros = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - frameStart).count();
        r.zoom = params->transformState.getZoom();
        r.pitchDeg = params->transformState.getPitch() * 180.0 / 3.14159265358979;
        r.glError = 0;
        for (int guard = 0; guard < 8; ++guard) {
            const GLenum e = glGetError();
            if (e == GL_NO_ERROR) break;
            if (r.glError == 0) r.glError = e;
        }
        GLint vp[4] = {0, 0, 0, 0};
        glGetIntegerv(GL_VIEWPORT, vp);
        GLint fbo = 0;
        glGetIntegerv(GL_FRAMEBUFFER_BINDING, &fbo);
        for (int i = 0; i < 4; ++i) r.viewport[i] = vp[i];
        r.scissorTest = glIsEnabled(GL_SCISSOR_TEST) ? 1 : 0;
        r.framebuffer = fbo;
        r.stage = util::RenderStage::describe();
        if (r.glError != 0) util::RenderStage::noteGlError("fin del cuadro", r.glError);
        // Verificacion profunda: tras un cuadro con error de GL se enciende para los proximos (cada dibujable comprueba glGetError y registra cual produce el error); se apaga tras 300 cuadros limpios.
        if (r.glError != 0) {
            // Se vuelve a encender como mucho una vez cada 10 s (con un error persistente cada cuadro, sin esto el log se llenaria).
            if (!util::RenderStage::deepGlCheck() && nowMs() - lastDeepEnableMs > 10000) {
                lastDeepEnableMs = nowMs();
                Log::Error(Event::Android, "GL-PROFUNDO encendido tras un error de GL (" + std::to_string(r.glError) + ") en la etapa: " + util::RenderStage::describe());
                util::RenderStage::enableDeepGl(60);
            }
            cleanFramesSinceGlError = 0;
        } else if (util::RenderStage::deepGlCheck() && ++cleanFramesSinceGlError > 300) {
            util::RenderStage::disableDeepGl();
        }
        if (r.glError != 0) dumpTrace("error de GL al terminar el cuadro");
        else if (r.micros > 250000) dumpTrace("cuadro lento");
        else if (fbo != 0) dumpTrace("FBO distinto del de pantalla al terminar");
    }

    // Deliver the snapshot if requested
    if (snapshotCallback) {
        snapshotCallback->operator()(backend->readFramebuffer());
        snapshotCallback.reset();
    }
}

void MapRenderer::onSurfaceCreated(JNIEnv& env, const jni::Object<AndroidSurface>& surface) {
    // Lock as the initialization can come from the main thread or the GL thread first
    std::scoped_lock lock(initialisationMutex);

    UniqueANativeWindow window_ = nullptr;

    if (surface) {
        window_ = std::unique_ptr<ANativeWindow, std::function<void(ANativeWindow*)>>(
            ANativeWindow_fromSurface(&env, reinterpret_cast<jobject>(surface.get())),
            [](ANativeWindow* window_) { ANativeWindow_release(window_); });
    }

    if (backend) {
        if (backend->createSurface(window_.get())) {
            window = std::move(window_);
            return;
        }

        // The android system will have already destroyed the underlying
        // GL resources if this is not the first initialization and an
        // attempt to clean them up will fail
        gfx::BackendScope backendGuard{backend->getImpl()};
        backend->markContextLost();
    }

    if (renderer) {
        renderer->markContextLost();
    }

    // Reset in opposite order
    renderer.reset();
    backend.reset();
    window = std::move(window_);

    // Create the new backend and renderer
    backend = AndroidRendererBackend::Create(window.get());
    renderer = std::make_unique<Renderer>(backend->getImpl(), pixelRatio, localIdeographFontFamily);
    rendererRef = std::make_unique<ActorRef<Renderer>>(*renderer, mailboxData.getMailbox());

#if MLN_RENDER_BACKEND_OPENGL
    // If we're running the emulator with the OpenGL backend, we're going to crash eventually,
    // unless we enable this mitigation.
    if (inEmulator()) {
        renderer->enableAndroidEmulatorGoldfishMitigation(true);
    }
#endif

    backend->setSwapBehavior(swapBehaviorFlush ? gfx::Renderable::SwapBehaviour::Flush
                                               : gfx::Renderable::SwapBehaviour::NoFlush);

    // Set the observer on the new Renderer implementation
    if (rendererObserver) {
        renderer->setObserver(rendererObserver.get());
    }
}

void MapRenderer::onSurfaceChanged([[maybe_unused]] JNIEnv& env, jint width, jint height) {
#if MLN_RENDER_BACKEND_OPENGL
    if (!renderer) {
        // In case the surface has been destroyed (due to app back-grounding)
        jni::jobject* nullObj = nullptr;
        onSurfaceCreated(env, jni::Object<AndroidSurface>(nullObj));
    }
#endif

    backend->resizeFramebuffer(width, height);
    framebufferSizeChanged = true;
    requestRender();
}

void MapRenderer::onRendererReset(JNIEnv&) {
    // Make sure to destroy the renderer on the GL Thread
    auto self = ActorRef<MapRenderer>(*this, mailboxData.getMailbox());
    self.ask(&MapRenderer::resetRenderer).wait();
}

// needs to be called on GL thread
void MapRenderer::onSurfaceDestroyed(JNIEnv&) {
    if (backend) {
        backend->destroySurface();
    }

    window.reset();
}

void MapRenderer::setSwapBehaviorFlush(JNIEnv&, jboolean flush) {
    swapBehaviorFlush = flush;

    if (backend) {
        backend->setSwapBehavior(flush ? gfx::Renderable::SwapBehaviour::Flush
                                       : gfx::Renderable::SwapBehaviour::NoFlush);
    }
}

// Static methods //

void MapRenderer::registerNative(jni::JNIEnv& env) {
    // Lookup the class
    static auto& javaClass = jni::Class<MapRenderer>::Singleton(env);

#define METHOD(MethodPtr, name) jni::MakeNativePeerMethod<decltype(MethodPtr), (MethodPtr)>(name)

    // Register the peer
    jni::RegisterNativePeer<MapRenderer>(
        env,
        javaClass,
        "nativePtr",
        jni::MakePeer<MapRenderer, const jni::Object<MapRenderer>&, jni::jfloat, const jni::String&>,
        "nativeInitialize",
        "finalize",
        METHOD(&MapRenderer::render, "nativeRender"),
        METHOD(&MapRenderer::onRendererReset, "nativeReset"),
        METHOD(&MapRenderer::onSurfaceCreated, "nativeOnSurfaceCreated"),
        METHOD(&MapRenderer::onSurfaceChanged, "nativeOnSurfaceChanged"),
        METHOD(&MapRenderer::onSurfaceDestroyed, "nativeOnSurfaceDestroyed"),
        METHOD(&MapRenderer::setSwapBehaviorFlush, "nativeSetSwapBehaviorFlush"));
}

MapRenderer& MapRenderer::getNativePeer(JNIEnv& env, const jni::Object<MapRenderer>& jObject) {
    static auto& javaClass = jni::Class<MapRenderer>::Singleton(env);
    static auto field = javaClass.GetField<jlong>(env, "nativePtr");
    MapRenderer* mapRenderer = reinterpret_cast<MapRenderer*>(jObject.Get(env, field));
    assert(mapRenderer != nullptr);
    return *mapRenderer;
}

} // namespace android
} // namespace mln
