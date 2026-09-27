#pragma once

#include <mln/gfx/drawable_data.hpp>
#include <mln/util/tileset.hpp>

#include <array>
#include <memory>

namespace mln {
namespace gfx {

class TerrainDrawableData : public DrawableData {
public:
    TerrainDrawableData(int32_t dim_, std::array<float, 4> unpack_, float exaggeration_, float eleDelta_,
                        float demScale_ = 1.0f, float demOffsetX_ = 0.0f, float demOffsetY_ = 0.0f)
        : dim(dim_),
          unpack(unpack_),
          exaggeration(exaggeration_),
          eleDelta(eleDelta_),
          demScale(demScale_),
          demOffsetX(demOffsetX_),
          demOffsetY(demOffsetY_) {}

    int32_t dim;
    std::array<float, 4> unpack;
    float exaggeration;
    float eleDelta;
    // Ventana del DEM del tile padre que corresponde a este sub-tile (subdivision del
    // drape, ADR 0034). Con demScale=1 y offset 0 es el tile completo (sin subdividir).
    float demScale;
    float demOffsetX;
    float demOffsetY;
};

using UniqueTerrainDrawableData = std::unique_ptr<TerrainDrawableData>;

} // namespace gfx
} // namespace mln
