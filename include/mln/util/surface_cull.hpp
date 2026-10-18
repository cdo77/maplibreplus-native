#pragma once

#include <mln/tile/tile_id.hpp>

#include <cstdint>

namespace mln {
namespace util {

/// Recorte por celda (ADR 0059, plan E2, F2): mientras una pasada de superficie renderiza la celda `z/x/y`, los grupos de capas por tesela solo recorren las teselas que se
/// solapan con ella (el ancestro, ella misma o un descendiente), no todas las retenidas (~1600): sin esto cada pasada por celda costaria lo mismo que una de drape.
/// Es un puntero por hilo que fija `TileRenderTarget::render` con un `SurfaceCullScope`; sin alcance activo (todo lo demas) no filtra nada.
struct SurfaceCull {
    uint8_t z = 0;
    uint32_t x = 0;
    uint32_t y = 0;
    int16_t wrap = 0;

    bool intersects(const OverscaledTileID& id) const {
        if (id.wrap != wrap) return false;
        const auto& t = id.canonical;
        if (t.z >= z) {
            const unsigned dz = static_cast<unsigned>(t.z - z);
            return (static_cast<uint64_t>(t.x) >> dz) == x && (static_cast<uint64_t>(t.y) >> dz) == y;
        }
        const unsigned dz = static_cast<unsigned>(z - t.z);
        return (static_cast<uint64_t>(x) >> dz) == t.x && (static_cast<uint64_t>(y) >> dz) == t.y;
    }

    /// Celda cuya pasada esta corriendo, para recortar cada dibujable a la huella de su tesela (fase P2 de la pasada propia). Aparte de `current()` porque este rige para todos los
    /// grupos (el recorte de dibujables solo aplica a los de imagen).
    static const SurfaceCull*& clip() {
        static thread_local const SurfaceCull* value = nullptr;
        return value;
    }

    static const SurfaceCull*& current() {
        static thread_local const SurfaceCull* value = nullptr;
        return value;
    }
};

class SurfaceClipScope {
public:
    explicit SurfaceClipScope(const SurfaceCull* cell) : previous(SurfaceCull::clip()) { SurfaceCull::clip() = cell; }
    ~SurfaceClipScope() { SurfaceCull::clip() = previous; }
    SurfaceClipScope(const SurfaceClipScope&) = delete;
    SurfaceClipScope& operator=(const SurfaceClipScope&) = delete;

private:
    const SurfaceCull* previous;
};

class SurfaceCullScope {
public:
    explicit SurfaceCullScope(const SurfaceCull* cull) : previous(SurfaceCull::current()) { SurfaceCull::current() = cull; }
    ~SurfaceCullScope() { SurfaceCull::current() = previous; }
    SurfaceCullScope(const SurfaceCullScope&) = delete;
    SurfaceCullScope& operator=(const SurfaceCullScope&) = delete;

private:
    const SurfaceCull* previous;
};

} // namespace util
} // namespace mln
