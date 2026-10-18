#pragma once

#include <atomic>

namespace mln {
namespace util {

/// Modo de la superficie del terreno 3D (ADR 0059, plan E2): interruptor de la capacidad nueva (invariante 9).
///   0 = DRAPES  (por defecto): el camino de siempre, sin una linea tocada.
///   1 = CELLS_DRY (F2): ademas de los drapes, renderiza las celdas al pool de texturas SIN usarlas (solo para medir costo y memoria).
///   2 = CELLS (F3): el terreno muestra la superficie por celda.
class SurfaceMode {
public:
    static SurfaceMode& instance() {
        static SurfaceMode m;
        return m;
    }
    void set(int value) { mode.store(value < 0 ? 0 : value); }
    int value() const { return mode.load(); }

private:
    std::atomic<int> mode{0};
};

} // namespace util
} // namespace mln
