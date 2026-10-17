#pragma once

#include <mln/map/transform_state.hpp>
#include <mln/math/angles.hpp>
#include <mln/renderer/paint_parameters.hpp>
#include <mln/util/constants.hpp>
#include <mln/util/ecef.hpp>
#include <mln/util/globe.hpp>

#include <cmath>

namespace mln {

// Tile-local (x, y en [0,EXTENT], z = metros sobre el elipsoide) -> clip de la camara con altura real
// (ADR 0038): matriz RTE de la camara (TransformState::getEcefTileMatrix) por una afin local alrededor
// de la esquina NO del tile -- este y sur en metros por unidad de tile, arriba en metros. En un tile
// z14 (~2,4 km) el error por curvatura es de ~0,5 m.
inline mat4 ecefSpriteTileMatrix(const UnwrappedTileID& tileID,
                          const TransformState& state,
                          const TransformState::EcefCamera& camera) {
    namespace ecef = util::ecef;
    namespace globe = util::globe;

    const LatLng northWest(tileID.canonical);
    const vec3 origin = ecef::llaToEcef(northWest, 0.0);
    const vec3 up = ecef::surfaceNormal(northWest);
    const vec3 east = globe::normalize(globe::cross(vec3{0.0, 0.0, 1.0}, up));
    const vec3 north = globe::cross(up, east);
    const double metersPerUnit = util::M2PI * util::EARTH_RADIUS_M * std::cos(util::deg2rad(northWest.latitude())) /
                                 std::pow(2.0, tileID.canonical.z) / util::EXTENT;

    mat4 local = matrix::identity4();
    for (size_t row = 0; row < 3; ++row) {
        local[0 * 4 + row] = east[row] * metersPerUnit;
        local[1 * 4 + row] = -north[row] * metersPerUnit;
        local[2 * 4 + row] = up[row];
    }
    mat4 result;
    matrix::multiply(result, state.getEcefTileMatrix(origin, camera), local);
    return result;
}


/// Anclaje de un tile al mapa de alturas del terreno (ADR 0038): escala y corrimiento tile-local -> UV.
struct EcefHeightmapAnchor {
    float scale;
    std::array<float, 2> offset;
    /// Origen mercator del tile (con el desplazamiento de su copia del mundo) y tamano por unidad de tile: para ubicar un ancla exacta (ADR 0058).
    double tx0, ty0, mercatorPerUnit;
};

inline EcefHeightmapAnchor ecefHeightmapAnchor(const UnwrappedTileID& tileID, const std::array<double, 3>& heightmapBounds) {
    const double tilesAtZ = std::pow(2.0, tileID.canonical.z);
    const double tx0 = (tileID.canonical.x + (tileID.wrap * tilesAtZ)) / tilesAtZ;
    const double ty0 = tileID.canonical.y / tilesAtZ;
    return {static_cast<float>(1.0 / (util::EXTENT * tilesAtZ * heightmapBounds[2])),
            {static_cast<float>((tx0 - heightmapBounds[0]) / heightmapBounds[2]),
             static_cast<float>((ty0 - heightmapBounds[1]) / heightmapBounds[2])},
            tx0,
            ty0,
            1.0 / (tilesAtZ * util::EXTENT)};
}

} // namespace mln
