#pragma once

#include <mln/gfx/context.hpp>
#include <mln/util/surface_cull.hpp>

namespace mln {

class PaintParameters;
class RenderOrchestrator;

/// Pasada de superficie propia (plan `pasada-propia-superficie`, fase P1): dibuja la imagen base de una celda directo sobre su textura, sin pasar por los grupos de capas, los
/// tweakers ni los UBO por dibujable. Es la logica del pase "Surface" de ATAK: cada cuadro de tesela de imagen que solapa la celda va como un quad con su textura ya subida,
/// de la mas gruesa a la mas fina. Solo OpenGL; en otros backends no hace nada. Lo usa `TileRenderTarget::render` con `SurfaceMode` 5.
namespace surface_pass {

void drawImagery(RenderOrchestrator&, PaintParameters&, const util::SurfaceCull& cell);

/// Genera la cadena de mipmaps de la textura de una celda recien renderizada y activa el filtro trilineal y, si el driver lo da, el anisotropico (hasta 8x): sin ellos los techos
/// lejanos de una vista inclinada se submuestrean y muestran rayas. `PaintParameters` solo se usa para declarar sucio el estado de GL. Fase P3; lo usa `SurfaceMode` 7.
void generateCellMipmaps(PaintParameters&, const gfx::Texture2DPtr&);

} // namespace surface_pass
} // namespace mln
