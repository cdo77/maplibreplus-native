#pragma once

#include <mln/shaders/layer_ubo.hpp>

namespace mln {
namespace shaders {

struct alignas(16) TerrainDrawableUBO {
    /*   0 */ std::array<float, 4 * 4> matrix;
    /*  64 */ std::array<float, 4 * 4> terrain_matrix;
    /* 128 */ std::array<float, 4> terrain_unpack;
    /* 144 */ float terrain_dim;
    /* 148 */ float terrain_exaggeration;
    /* 152 */ float ele_delta;
    /* 156 */ float center_elevation;
    /* 160 */
};
static_assert(sizeof(TerrainDrawableUBO) == 10 * 16);

using TerrainDepthDrawableUBO = TerrainDrawableUBO;

} // namespace shaders
} // namespace mln
