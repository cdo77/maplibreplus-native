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
    /* 348 */ float center_elevation;
    // Camara con altura real (ADR 0034): 1.0 si este drawable usa a_ecef_pos/a_ecef_normal
    // (posicion sobre el elipsoide WGS84 real) y los drapes (ADR 0037); 0.0 en modo planar de
    // siempre (page propia, u_terrain_image).
    /* 352 */ float ecef_mode;
    /* 356 */ float pad0;
    /* 360 */ float pad1;
    /* 364 */ float pad2;
    /* 368 */
};
static_assert(sizeof(TerrainDrawableUBO) == 23 * 16);

using TerrainDepthDrawableUBO = TerrainDrawableUBO;

} // namespace shaders
} // namespace mln
