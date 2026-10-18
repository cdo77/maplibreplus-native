#pragma once

#include <mln/shaders/layer_ubo.hpp>

namespace mln {
namespace shaders {

struct alignas(16) TerrainDrawableUBO {
    /*   0 */ std::array<float, 4 * 4> matrix;
    /*  64 */ std::array<float, 4 * 4> terrain_matrix;
    // Drapes multi-resolucion (ADR 0037): tile-local [0,EXTENT] de ESTE sub-tile -> UV de cada
    // render target compartido por TODOS los sub-tiles del frame (1x/4x/32x, ver
    // RenderTerrain::drapes). Solo se usan en modo camara con altura real (ecef_mode); en modo
    // planar el shader sigue con u_terrain_image (page vieja, MegaTexture por sub-tile).
    /* 128 */ std::array<float, 4 * 4> drape_matrix0;
    /* 192 */ std::array<float, 4 * 4> drape_matrix1;
    /* 256 */ std::array<float, 4 * 4> drape_matrix2;
    /* 320 */ std::array<float, 4> terrain_unpack;
    /* 336 */ float terrain_dim;
    /* 340 */ float terrain_exaggeration;
    /* 344 */ float ele_delta;
    // Cuantos drapes son el planisferio entero (vista de globo), contando desde el 32x: 0 a 3. En esos la u se
    // envuelve (el planisferio es un mundo exacto) y la v se recorta al borde en los polos, como ATAK.
    /* 348 */ float drape_global;
    // Camara con altura real (ADR 0034): 1.0 si este drawable usa a_ecef_pos/a_ecef_normal
    // (posicion sobre el elipsoide WGS84 real) y los drapes (ADR 0037); 0.0 en modo planar de
    // siempre (page propia, u_terrain_image).
    /* 352 */ float ecef_mode;
    // Color plano del casquete polar (ecef_mode == 2): la malla del polo no tiene drapes (el mercator no llega).
    /* 356 */ float cap_r;
    /* 360 */ float cap_g;
    /* 364 */ float cap_b;
    // Superficie por celda (ADR 0059, F3): tile-local de este sub-tile -> UV de la textura de la celda (propia o la del ancestro mas cercano).
    // cell_valid = 1.0 solo con SurfaceMode >= 2 y celda disponible; con 0.0 el terreno sigue pintando solo con los drapes.
    /* 368 */ std::array<float, 4 * 4> cell_matrix;
    /* 432 */ float cell_valid;
    /* 436 */ float cell_only;   // 1.0 = solo celdas: el shader no muestrea los drapes (plan P4, SurfaceMode >= 5)
    /* 440 */ float cell_pad1;
    /* 444 */ float cell_pad2;
    /* 448 */
};
static_assert(sizeof(TerrainDrawableUBO) == 28 * 16);

using TerrainDepthDrawableUBO = TerrainDrawableUBO;

} // namespace shaders
} // namespace mln
