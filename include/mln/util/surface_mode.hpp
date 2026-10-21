#pragma once

#include <atomic>

namespace mln {
namespace util {

/// Superficie del terreno 3D (ADR 0059, plan E2): `0` = clasica con drapes (por defecto, el camino de siempre); cualquier valor > 0 = celdas: cada celda del terreno tiene su textura,
/// la imagen la piden las celdas visibles, los drapes no se renderizan y las teselas de imagen se suben en RGB565. Se fija desde la app (Configuraciones > Mapa 3D) con `setSurfaceMode`.
class SurfaceMode {
public:
    static SurfaceMode& instance() {
        static SurfaceMode m;
        return m;
    }
    void set(int value) { mode.store(value < 0 ? 0 : value); }
    int value() const { return mode.load(); }
    bool cells() const { return mode.load() > 0; }

private:
    std::atomic<int> mode{0};
};

} // namespace util
} // namespace mln
