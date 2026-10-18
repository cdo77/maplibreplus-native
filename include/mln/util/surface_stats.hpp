#pragma once

#include <atomic>
#include <cstdint>

namespace mln {
namespace util {

/// Costo de las pasadas de superficie por celda (ADR 0059, plan E2, F2): las suma `TileRenderTarget::render` y las lee y reinicia `RenderTerrain` en su log de estadisticas.
/// Con `SurfaceMode` 1 es el tiempo de CPU que tarda en armar la pasada; con 3 se espera a que la GPU termine (`glFinish`) y es el tiempo real de la pasada.
class SurfaceStats {
public:
    static SurfaceStats& instance() {
        static SurfaceStats s;
        return s;
    }
    void add(int64_t micros) {
        passes.fetch_add(1);
        totalMicros.fetch_add(micros);
        int64_t prev = maxMicros.load();
        while (micros > prev && !maxMicros.compare_exchange_weak(prev, micros)) {
        }
    }
    struct Window {
        uint64_t passes;
        int64_t totalMicros;
        int64_t maxMicros;
    };
    Window takeWindow() { return {passes.exchange(0), totalMicros.exchange(0), maxMicros.exchange(0)}; }

private:
    std::atomic<uint64_t> passes{0};
    std::atomic<int64_t> totalMicros{0};
    std::atomic<int64_t> maxMicros{0};
};

} // namespace util
} // namespace mln
