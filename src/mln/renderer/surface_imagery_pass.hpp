#pragma once

#include <mln/util/surface_cull.hpp>

namespace mln {

class PaintParameters;
class RenderOrchestrator;

/// Pasada de superficie propia (plan `pasada-propia-superficie`, fase P1): dibuja la imagen base de una celda directo sobre su textura, sin pasar por los grupos de capas, los
/// tweakers ni los UBO por dibujable. Es la logica del pase "Surface" de ATAK: cada cuadro de tesela de imagen que solapa la celda va como un quad con su textura ya subida,
/// de la mas gruesa a la mas fina. Solo OpenGL; en otros backends no hace nada. Lo usa `TileRenderTarget::render` con `SurfaceMode` 5.
namespace surface_pass {

void drawImagery(RenderOrchestrator&, PaintParameters&, const util::SurfaceCull& cell);

} // namespace surface_pass
} // namespace mln
