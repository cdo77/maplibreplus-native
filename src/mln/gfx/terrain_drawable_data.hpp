#pragma once

#include <mln/gfx/drawable_data.hpp>
#include <mln/util/tileset.hpp>
#include <mln/util/vectors.hpp>

#include <array>
#include <memory>
#include <optional>

namespace mln {
namespace gfx {

class TerrainDrawableData : public DrawableData {
public:
    TerrainDrawableData(int32_t dim_, std::array<float, 4> unpack_, float exaggeration_, float eleDelta_,
                        float demScale_ = 1.0f, float demOffsetX_ = 0.0f, float demOffsetY_ = 0.0f,
                        std::optional<vec3> ecefOrigin_ = std::nullopt)
        : dim(dim_),
          unpack(unpack_),
          exaggeration(exaggeration_),
          eleDelta(eleDelta_),
          demScale(demScale_),
          demOffsetX(demOffsetX_),
          demOffsetY(demOffsetY_),
          ecefOrigin(ecefOrigin_) {}

    int32_t dim;
    std::array<float, 4> unpack;
    float exaggeration;
    float eleDelta;
    // Ventana del DEM del tile padre que corresponde a este sub-tile (subdivision del
    // drape, ADR 0034). Con demScale=1 y offset 0 es el tile completo (sin subdividir).
    float demScale;
    float demOffsetX;
    float demOffsetY;
    // Origen ECEF real (double, WGS84) del sub-tile, presente solo en modo camara con altura
    // real. updateUniforms lo usa para pedir la matriz RTE (TransformState::getEcefTileMatrix)
    // en vez de la matriz mercator de siempre.
    std::optional<vec3> ecefOrigin;
};

using UniqueTerrainDrawableData = std::unique_ptr<TerrainDrawableData>;

} // namespace gfx
} // namespace mln
