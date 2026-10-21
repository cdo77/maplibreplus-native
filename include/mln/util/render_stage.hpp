#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>

namespace mln {
namespace util {

/// Ultima etapa de render marcada con su contexto (diagnostico de campo): si el renderer lanza una excepcion (p. ej. std::bad_alloc, que en el log solo decia "std::bad_alloc"), `MapRenderer::render` registra
/// en que etapa estaba y con que cantidades. Es una marca barata (tres atomicos); no cambia ningun comportamiento.
class RenderStage {
public:
    static void set(const char* name, uint64_t a = 0, uint64_t b = 0) {
        auto& s = state();
        s.a.store(a, std::memory_order_relaxed);
        s.b.store(b, std::memory_order_relaxed);
        s.name.store(name, std::memory_order_relaxed);
    }
    /// Error de GL detectado (glGetError) y donde: se acumula la cuenta y se guarda el ultimo. Lo lee la traza de cuadros de MapRenderer.
    static void noteGlError(const char* where, uint32_t error) {
        auto& s = state();
        s.glWhere.store(where, std::memory_order_relaxed);
        s.glError.store(error, std::memory_order_relaxed);
        s.glCount.fetch_add(1, std::memory_order_relaxed);
    }
    /// Verificacion profunda de GL: despues de un cuadro que termino con error de GL, `MapRenderer` la enciende y cada dibujable comprueba glGetError antes y despues de enlazar y dibujar,
    /// registrando cual dibujable (nombre, tesela, programa, pase) produce el error. Se apaga sola tras 300 cuadros sin error. Es barata: un glGetError por dibujable solo mientras esta encendida.
    static bool deepGlCheck() { return state().deepSlots.load(std::memory_order_relaxed) > 0; }
    static void enableDeepGl(int slots) { state().deepSlots.store(slots, std::memory_order_relaxed); }
    static void disableDeepGl() { state().deepSlots.store(0, std::memory_order_relaxed); }
    /// Toma uno de los cupos de registro de la verificacion profunda (devuelve false si ya se agotaron).
    static bool takeDeepLogSlot() { return state().deepSlots.fetch_sub(1, std::memory_order_relaxed) > 0; }
    static uint64_t glErrorCount() { return state().glCount.load(std::memory_order_relaxed); }
    static std::string describeGl() {
        auto& s = state();
        const char* w = s.glWhere.load(std::memory_order_relaxed);
        return std::string("gl_errores=") + std::to_string(s.glCount.load(std::memory_order_relaxed)) + " ultimo=0x" +
               [](uint32_t v) { char b[16]; std::snprintf(b, sizeof b, "%x", v); return std::string(b); }(s.glError.load(std::memory_order_relaxed)) +
               " en=" + (w ? w : "-");
    }
    static std::string describe() {
        auto& s = state();
        const char* n = s.name.load(std::memory_order_relaxed);
        return std::string(n ? n : "(sin etapa)") + " a=" + std::to_string(s.a.load(std::memory_order_relaxed)) + " b=" + std::to_string(s.b.load(std::memory_order_relaxed));
    }

private:
    struct State {
        std::atomic<const char*> name{nullptr};
        std::atomic<uint64_t> a{0};
        std::atomic<uint64_t> b{0};
        std::atomic<const char*> glWhere{nullptr};
        std::atomic<uint32_t> glError{0};
        std::atomic<uint64_t> glCount{0};
        std::atomic<int> deepSlots{0};
    };
    static State& state() {
        static State s;
        return s;
    }
};

} // namespace util
} // namespace mln
