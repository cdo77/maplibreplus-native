#include <mln/math/clamp.hpp>
#include <mln/math/log2.hpp>
#include <mln/util/bounding_volumes.hpp>
#include <mln/util/globe.hpp>
#include <mln/util/constants.hpp>
#include <mln/util/interpolate.hpp>
#include <mln/util/tile_coordinate.hpp>
#include <mln/util/tile_cover.hpp>
#include <mln/util/tile_cover_impl.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <list>

using namespace std::numbers;

namespace mln {

namespace {

using ScanLine = const std::function<void(int32_t x0, int32_t x1, int32_t y)>;

// Taken from polymaps src/Layer.js
// https://github.com/simplegeo/polymaps/blob/master/src/Layer.js#L333-L383
struct edge {
    double x0 = 0, y0 = 0;
    double x1 = 0, y1 = 0;
    double dx = 0, dy = 0;

    edge(Point<double> a, Point<double> b) {
        if (a.y > b.y) std::swap(a, b);
        x0 = a.x;
        y0 = a.y;
        x1 = b.x;
        y1 = b.y;
        dx = b.x - a.x;
        dy = b.y - a.y;
    }
};

// scan-line conversion
void scanSpans(edge e0, edge e1, int32_t ymin, int32_t ymax, ScanLine& scanLine) {
    const double y0 = ::fmax(ymin, std::floor(e1.y0));
    const double y1 = ::fmin(ymax, std::ceil(e1.y1));

    // sort edges by x-coordinate
    if ((e0.x0 == e1.x0 && e0.y0 == e1.y0) ? (e0.x0 + e1.dy / e0.dy * e0.dx < e1.x1)
                                           : (e0.x1 - e1.dy / e0.dy * e0.dx < e1.x0)) {
        std::swap(e0, e1);
    }

    // scan lines!
    const double m0 = e0.dx / e0.dy;
    const double m1 = e1.dx / e1.dy;
    const double d0 = e0.dx > 0;       // use y + 1 to compute x0
    const double d1 = e1.dx < 0;       // use y + 1 to compute x1
    for (double y = y0; y < y1; y++) { // NOLINT(clang-analyzer-security.FloatLoopCounter)
        double x0 = m0 * ::fmax(0, ::fmin(e0.dy, y + d0 - e0.y0)) + e0.x0;
        double x1 = m1 * ::fmax(0, ::fmin(e1.dy, y + d1 - e1.y0)) + e1.x0;
        scanLine(static_cast<int32_t>(std::floor(x1)), static_cast<int32_t>(std::ceil(x0)), static_cast<int32_t>(y));
    }
}

// scan-line conversion
void scanTriangle(const Point<double>& a,
                  const Point<double>& b,
                  const Point<double>& c,
                  int32_t ymin,
                  int32_t ymax,
                  ScanLine& scanLine) {
    edge ab = edge(a, b);
    edge bc = edge(b, c);
    edge ca = edge(c, a);

    // sort edges by y-length
    if (ab.dy > bc.dy) {
        std::swap(ab, bc);
    }
    if (ab.dy > ca.dy) {
        std::swap(ab, ca);
    }
    if (bc.dy > ca.dy) {
        std::swap(bc, ca);
    }

    // scan span! scan span!
    if (ab.dy) scanSpans(ca, ab, ymin, ymax, scanLine);
    if (bc.dy) scanSpans(ca, bc, ymin, ymax, scanLine);
}

} // namespace

namespace util {

namespace {

std::vector<UnwrappedTileID> tileCover(const Point<double>& tl,
                                       const Point<double>& tr,
                                       const Point<double>& br,
                                       const Point<double>& bl,
                                       const Point<double>& c,
                                       uint8_t z) {
    const int32_t tiles = 1 << z;

    struct ID {
        int32_t x, y;
        double sqDist;
    };

    std::vector<ID> t;

    // skip the first few allocations, assuming we usually end up with at least a few tiles
    t.reserve(8);

    auto scanLine = [&](int32_t x0, int32_t x1, int32_t y) {
        int32_t x;
        if (y >= 0 && y <= tiles) {
            for (x = x0; x < x1; ++x) {
                const auto dx = x + 0.5 - c.x;
                const auto dy = y + 0.5 - c.y;
                t.emplace_back(ID{.x = x, .y = y, .sqDist = dx * dx + dy * dy});
            }
        }
    };

    // Divide the screen up in two triangles and scan each of them:
    // \---+
    // | \ |
    // +---\.
    scanTriangle(tl, tr, br, 0, tiles, scanLine);
    scanTriangle(br, bl, tl, 0, tiles, scanLine);

    // Sort first by distance, then by x/y.
    std::sort(t.begin(), t.end(), [](const ID& a, const ID& b) noexcept {
        return std::tie(a.sqDist, a.x, a.y) < std::tie(b.sqDist, b.x, b.y);
    });

    // Erase duplicate tile IDs (they typically occur at the common side of both triangles).
    t.erase(std::unique(t.begin(), t.end(), [](const ID& a, const ID& b) { return a.x == b.x && a.y == b.y; }),
            t.end());

    std::vector<UnwrappedTileID> result;
    result.reserve(t.size());
    for (const auto& id : t) {
        result.emplace_back(z, id.x, id.y);
    }
    return result;
}

} // namespace

int32_t coveringZoomLevel(double zoom, style::SourceType type, uint16_t size) noexcept {
    zoom += util::log2(util::tileSize_D / size);
    if (type == style::SourceType::Raster || type == style::SourceType::Video) {
        return static_cast<int32_t>(std::round(zoom));
    } else {
        return static_cast<int32_t>(std::floor(zoom));
    }
}

namespace {

constexpr double globeTilePixelTarget = 512.0;

struct GlobeTileSample {
    bool visible;
    double screenSize;
};

GlobeTileSample sampleGlobeTile(const TransformState& transformState, uint8_t zoom, uint32_t x, uint32_t y) {
    namespace globe = mln::util::globe;

    const vec4& plane = transformState.getGlobeClippingPlane();
    const mat4& matrix = transformState.getGlobeMatrix();
    const Size size = transformState.getSize();
    const double halfWidth = size.width * 0.5;
    const double halfHeight = size.height * 0.5;

    bool anyInFront = false;
    bool anyInside = false;
    bool anyBehindCamera = false;
    bool allLeft = true, allRight = true, allAbove = true, allBelow = true;

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();

    for (int32_t sy = 0; sy <= 2; sy++) {
        for (int32_t sx = 0; sx <= 2; sx++) {
            const double inTileX = sx * util::EXTENT * 0.5;
            const double inTileY = sy * util::EXTENT * 0.5;
            const vec3 spherePos = globe::projectTileCoordinatesToSphere(
                inTileX, inTileY, static_cast<int32_t>(x), static_cast<int32_t>(y), zoom);

            if (globe::pointPlaneSignedDistance(plane, spherePos) >= 0.0) {
                anyInFront = true;
            }

            vec4 projected;
            matrix::transformMat4(projected, vec4{spherePos[0], spherePos[1], spherePos[2], 1.0}, matrix);
            if (projected[3] <= 0.0) {
                anyBehindCamera = true;
                continue;
            }

            const double ndcX = projected[0] / projected[3];
            const double ndcY = projected[1] / projected[3];
            allLeft = allLeft && ndcX < -1.0;
            allRight = allRight && ndcX > 1.0;
            allBelow = allBelow && ndcY < -1.0;
            allAbove = allAbove && ndcY > 1.0;
            if (std::abs(ndcX) <= 1.0 && std::abs(ndcY) <= 1.0) {
                anyInside = true;
            }

            const double px = ndcX * halfWidth;
            const double py = ndcY * halfHeight;
            minX = std::min(minX, px);
            maxX = std::max(maxX, px);
            minY = std::min(minY, py);
            maxY = std::max(maxY, py);
        }
    }

    if (size.isEmpty() || !anyInFront) {
        return {.visible = false, .screenSize = 0.0};
    }

    if (anyBehindCamera || minX > maxX) {
        return {.visible = true, .screenSize = std::numeric_limits<double>::max()};
    }

    const bool offscreen = !anyInside && (allLeft || allRight || allAbove || allBelow);
    if (offscreen) {
        return {.visible = false, .screenSize = 0.0};
    }

    return {.visible = true, .screenSize = std::max(maxX - minX, maxY - minY)};
}

} // namespace

std::vector<OverscaledTileID> globeTileCover(const TransformState& transformState,
                                             uint8_t z,
                                             const Range<uint8_t>& zoomRange,
                                             uint8_t overscaledZ) {
    std::vector<OverscaledTileID> result;
    const uint8_t targetZ = util::clamp<uint8_t>(z, zoomRange.min, zoomRange.max);

    std::function<void(uint8_t, uint32_t, uint32_t)> visit = [&](uint8_t zoom, uint32_t x, uint32_t y) {
        const GlobeTileSample sample = sampleGlobeTile(transformState, zoom, x, y);
        if (!sample.visible) {
            return;
        }
        if (zoom >= targetZ) {
            result.emplace_back(overscaledZ, 0, zoom, x, y);
            return;
        }
        if (zoom >= zoomRange.min && sample.screenSize <= globeTilePixelTarget) {
            result.emplace_back(zoom, 0, zoom, x, y);
            return;
        }
        for (uint32_t i = 0; i < 4; i++) {
            visit(static_cast<uint8_t>(zoom + 1), (x << 1) + (i % 2), (y << 1) + (i >> 1));
        }
    };

    visit(0, 0, 0);
    return result;
}

namespace {
// Camara con altura real (ADR 0034): cobertura de tiles del DEM por RADIO GEOGRAFICO REAL
// alrededor del centro, en 360 grados, en vez de por el frustum de la vista mercator clasica --
// la camara puede girar el bearing en cualquier momento hacia una direccion que ese frustum no
// anticipa (ver TileCoverParameters::omnidirectional).
//
// RESOLUCION VARIABLE por distancia real (mismo patron que ATAK usa en
// ElMgrTerrainRenderService::radiusOfMaxLvlLodInTiles, y el mismo quadtree adaptativo por
// distancia que ya usa el LOD del propio terreno en RenderTerrain::update) -- un z fijo para todo
// el radio de 7km se probo en campo y mostraba, cerca de la camara, un solo texel de una imagen
// satelital/DEM de 2.4km estirado sobre unos pocos metros de terreno real (la geometria del LOD
// del terreno llegaba bien a esa escala, pero la textura que tenia disponible para pintarla no).
// Aca cada tile candidato se subdivide mientras su propio tamano real siga siendo grande frente a
// su propia distancia real al punto donde esta la camara, hasta kOmniMaxZoom -- da tiles chicos
// (mas detalle de imagen) cerca y tiles grandes lejos, sin fijar una unica resolucion para todo
// el radio.
std::vector<OverscaledTileID> omnidirectionalTileCover(const TransformState& transform,
                                                       uint8_t requestedZoom,
                                                       const LatLng& cameraLatLng,
                                                       double farM) {
    constexpr uint8_t kOmniMinZoom = 10; // ~39km de lado: de sobra para cubrir el radio real desde cualquier borde
    constexpr uint8_t kOmniMaxZoom = 18; // ~150m de lado en el ecuador: buena textura a nivel de calle
    constexpr double kSplitRatio = 1.2;  // subdivide mientras el tile sea > 1.2x su propia distancia real
    const uint8_t maxZ = clamp<uint8_t>(requestedZoom, kOmniMinZoom, kOmniMaxZoom);

    (void)transform;
    const double earthCircumferenceM = 2.0 * pi * util::EARTH_RADIUS_M;

    std::vector<OverscaledTileID> result;
    const std::function<void(uint8_t, int64_t, int64_t)> visit = [&](uint8_t z, int64_t x, int64_t y) {
        const double numTiles = std::pow(2.0, z);
        const auto tileCount = static_cast<int64_t>(numTiles);
        if (y < 0 || y >= tileCount) {
            return; // fuera del mundo en latitud (mercator no envuelve en Y)
        }
        const double tileSizeM = earthCircumferenceM / numTiles;
        const TileCoordinate camAtZ = TileCoordinate::fromLatLng(z, cameraLatLng);
        const double dx = (static_cast<double>(x) + 0.5) - camAtZ.p.x;
        const double dy = (static_cast<double>(y) + 0.5) - camAtZ.p.y;
        const double distM = std::sqrt((dx * dx) + (dy * dy)) * tileSizeM;
        // Radio circunscrito del tile (mitad de su diagonal) como cota conservadora: si ni el
        // punto mas cercano del tile entra en el far-plane real, se descarta sin seguir bajando.
        if (distM - (tileSizeM * 0.71) > farM) {
            return;
        }
        if (z < maxZ && tileSizeM > distM * kSplitRatio) {
            for (int i = 0; i < 4; ++i) {
                visit(static_cast<uint8_t>(z + 1), (x << 1) + (i % 2), (y << 1) + (i >> 1));
            }
            return;
        }
        // Wrap del mundo: x fuera de [0,tileCount) es otra copia del planeta en longitud, igual
        // que las raices wrap=-3..3 del camino normal mas abajo en este archivo.
        const int64_t wrap = x >= 0 ? x / tileCount : (x - tileCount + 1) / tileCount;
        const auto wrappedX = static_cast<uint32_t>(x - wrap * tileCount);
        result.emplace_back(z, static_cast<int16_t>(wrap), z, wrappedX, static_cast<uint32_t>(y));
    };

    // Arranca en un area 3x3 de tiles raiz (kOmniMinZoom) alrededor de la camara -- por si el
    // punto real cae cerca de un borde, y a ese zoom cada tile ya mide decenas de km, de sobra
    // para cubrir el radio real desde cualquiera de los 9 vecinos.
    const double startNumTiles = std::pow(2.0, kOmniMinZoom);
    const TileCoordinate camAtStart = TileCoordinate::fromLatLng(kOmniMinZoom, cameraLatLng);
    const auto startX = static_cast<int64_t>(std::floor(camAtStart.p.x));
    const auto startY = static_cast<int64_t>(std::floor(camAtStart.p.y));
    for (int64_t oy = startY - 1; oy <= startY + 1; ++oy) {
        if (oy < 0 || oy >= static_cast<int64_t>(startNumTiles)) {
            continue;
        }
        for (int64_t ox = startX - 1; ox <= startX + 1; ++ox) {
            visit(kOmniMinZoom, ox, oy);
        }
    }
    return result;
}

// Dos tiles se solapan si estan en la misma copia del mundo y uno es ancestro (o igual) del otro.
bool tilesOverlap(const OverscaledTileID& a, const OverscaledTileID& b) {
    if (a.wrap != b.wrap) {
        return false;
    }
    const auto& ca = a.canonical;
    const auto& cb = b.canonical;
    const uint8_t zMin = std::min(ca.z, cb.z);
    return (ca.x >> (ca.z - zMin)) == (cb.x >> (cb.z - zMin)) && (ca.y >> (ca.z - zMin)) == (cb.y >> (cb.z - zMin));
}

// Tiles de alrededor del OJO hasta su far (camara orbital, ADR 0040; estimacion plana, sin relieve) y, si se pide, los
// de alrededor del FOCO en un radio chico, sin solaparse con los anteriores.
std::vector<OverscaledTileID> ecefOmnidirectionalTileCover(const TransformState& transform,
                                                          uint8_t requestedZoom,
                                                          bool includeFocus) {
    const TransformState::EcefCamera camera = transform.computeEcefCamera(0.0);
    std::vector<OverscaledTileID> result = omnidirectionalTileCover(transform, requestedZoom, camera.eyeLatLng, camera.farM);
    if (!includeFocus) {
        return result;
    }
    constexpr double kFocusRadiusM = 20000.0;
    for (const OverscaledTileID& tile : omnidirectionalTileCover(transform, requestedZoom, camera.focus, kFocusRadiusM)) {
        const bool overlaps = std::any_of(result.begin(), result.end(), [&](const OverscaledTileID& existing) {
            return tilesOverlap(existing, tile);
        });
        if (!overlaps) {
            result.push_back(tile);
        }
    }
    return result;
}
} // namespace

std::vector<OverscaledTileID> tileCover(const TileCoverParameters& state,
                                        uint8_t z,
                                        const Range<uint8_t> zoomRange,
                                        const std::optional<uint8_t>& overscaledZ) {
    if (state.transformState.isGlobeRendering()) {
        return globeTileCover(state.transformState, z, zoomRange, overscaledZ.value_or(z));
    }
    if (state.omnidirectional) {
        return ecefOmnidirectionalTileCover(state.transformState, z, state.includeFocus);
    }

    struct Node {
        AABB aabb;
        uint8_t zoom;
        uint32_t x, y;
        int16_t wrap;
        bool fullyVisible;
    };

    struct ResultTile {
        OverscaledTileID id;
        double sqrDist;
    };

    auto childrenOf = [](const Node& node) -> std::vector<Node> {
        std::vector<Node> children(4);
        for (int i = 0; i < 4; i++) {
            const uint32_t childX = (node.x << 1) + (i % 2);
            const uint32_t childY = (node.y << 1) + (i >> 1);

            children[i] = node;
            children[i].aabb = node.aabb.quadrant(i);
            children[i].zoom = node.zoom + 1;
            children[i].x = childX;
            children[i].y = childY;
        }
        return children;
    };

    const auto& transform = state.transformState;
    const double numTiles = std::pow(2.0, z);
    const double worldSize = Projection::worldSize(transform.getScale());
    const bool allowVariableZoom = transform.getPitch() > state.tileLodPitchThreshold;
    const uint8_t minZoom = allowVariableZoom ? zoomRange.min : z;
    const uint8_t maxZoom = ((state.tileLodMode == TileLodMode::Distance) && allowVariableZoom) ? zoomRange.max : z;
    const uint8_t overscaledZoom = std::max(overscaledZ.value_or(z), maxZoom);
    const bool flippedY = transform.getViewportMode() == ViewportMode::FlippedY;

    const auto centerPoint = TileCoordinate::fromScreenCoordinate(
                                 transform, z, {transform.getSize().width / 2.0, transform.getSize().height / 2.0})
                                 .p;

    const vec3 centerCoord = {{centerPoint.x, centerPoint.y, 0.0}};

    assert(transform.getFreeCameraOptions().position);
    const vec3 cameraPositionMercator = *transform.getFreeCameraOptions().position;
    const double nominalScale = std::pow(2.0, z);
    const vec3 cameraCoord = vec3Scale(cameraPositionMercator, nominalScale);
    const double cameraToCenterDistanceMercator = vec3Length(vec3Sub(cameraCoord, centerCoord)) / worldSize;

    const Frustum frustum = Frustum::fromInvProjMatrix(transform.getInvProjectionMatrix(), worldSize, z, flippedY);

    // There should always be a certain number of maximum zoom level tiles
    // surrounding the center location
    assert(state.tileLodMinRadius >= 1);
    const double radiusOfMaxLvlLodInTiles = std::max(1.0, state.tileLodMinRadius);

    const auto newRootTile = [&](int16_t wrap) -> Node {
        return {.aabb = AABB({{wrap * numTiles, 0.0, 0.0}}, {{(wrap + 1) * numTiles, numTiles, 0.0}}),
                .zoom = uint8_t(0),
                .x = uint16_t(0),
                .y = uint16_t(0),
                .wrap = wrap,
                .fullyVisible = false};
    };

    // Perform depth-first traversal on tile tree to find visible tiles
    std::vector<Node> stack;
    std::vector<ResultTile> result;
    stack.reserve(128);

    // World copies shall be rendered three times on both sides from closest to farthest
    for (int i = 1; i <= 3; i++) {
        stack.push_back(newRootTile(-i));
        stack.push_back(newRootTile(i));
    }

    stack.push_back(newRootTile(0));

    while (!stack.empty()) {
        Node node = stack.back();
        stack.pop_back();

        // Use cached visibility information of ancestor nodes
        if (!node.fullyVisible) {
            const IntersectionResult intersection = frustum.intersects(node.aabb);

            if (intersection == IntersectionResult::Separate) continue;

            node.fullyVisible = intersection == IntersectionResult::Contains;
        }

        bool shouldSplitTile;
        if (state.tileLodMode == TileLodMode::Distance) {
            const vec3 camToTileMercator = vec3Scale(node.aabb.distanceXYZ(cameraCoord), 1.0 / worldSize);
            const double distanceToTileMercator = vec3Length(camToTileMercator);
            const double cosPitchToTile = std::max(0.0, camToTileMercator[2] / distanceToTileMercator);
            const double pitchExponent =
                0.5; // 0: constant screen width, 1/2: constant screen area, 1: constant screen height
            double tileScale = std::pow(2.0, node.zoom);
            shouldSplitTile = distanceToTileMercator * tileScale < std::pow(cosPitchToTile, pitchExponent) *
                                                                       cameraToCenterDistanceMercator /
                                                                       state.tileLodScale * nominalScale;
        } else {
            const vec3 distanceXyz = node.aabb.distanceXYZ(centerCoord);
            const double* longestDim = std::max_element(distanceXyz.data(), distanceXyz.data() + distanceXyz.size());
            assert(longestDim);

            // We're using distance based heuristics to determine if a tile should
            // be split into quadrants or not. radiusOfMaxLvlLodInTiles defines that
            // there's always a certain number of maxLevel tiles next to the map
            // center. Using the fact that a parent node in quadtree is twice the
            // size of its children (per dimension) we can define distance
            // thresholds for each relative level:
            // f(k) = offset + 2 + 4 + 8 + 16 + ... + 2^k
            // This is the same as:
            // f(k) = offset + 2^(k+1)-2
            const double distToSplit = radiusOfMaxLvlLodInTiles + (1 << (maxZoom - node.zoom)) - 2;
            shouldSplitTile = *longestDim * state.tileLodScale < distToSplit;
        }

        // Have we reached the target depth or is the tile too far away to be any split further?
        if (node.zoom == maxZoom || (!shouldSplitTile && node.zoom >= minZoom)) {
            // Perform precise intersection test between the frustum and aabb.
            // This will cull < 1% false positives missed by the original test
            if (node.fullyVisible || frustum.intersectsPrecise(node.aabb, true) != IntersectionResult::Separate) {
                const OverscaledTileID id = {
                    node.zoom == maxZoom ? overscaledZoom : node.zoom, node.wrap, node.zoom, node.x, node.y};
                vec3 coordToLoadFirst = (state.tileLodMode == TileLodMode::Distance) ? cameraCoord : centerCoord;
                const double dx = node.wrap * numTiles + node.x + 0.5 - coordToLoadFirst[0];
                const double dy = node.y + 0.5 - coordToLoadFirst[1];

                result.push_back({id, dx * dx + dy * dy});
            }
        } else {
            std::vector<Node> children = childrenOf(node);
            stack.insert(stack.end(), children.begin(), children.end());
        }
    }

    // Sort results by distance
    std::sort(
        result.begin(), result.end(), [](const ResultTile& a, const ResultTile& b) { return a.sqrDist < b.sqrDist; });

    std::vector<OverscaledTileID> ids;
    ids.reserve(result.size());

    for (const auto& tile : result) {
        ids.push_back(tile.id);
    }

    return ids;
}

std::vector<UnwrappedTileID> tileCover(const LatLngBounds& bounds_, uint8_t z) {
    if (bounds_.isEmpty() || bounds_.south() > util::LATITUDE_MAX || bounds_.north() < -util::LATITUDE_MAX) {
        return {};
    }

    const LatLngBounds bounds = LatLngBounds::hull({std::max(bounds_.south(), -util::LATITUDE_MAX), bounds_.west()},
                                                   {std::min(bounds_.north(), util::LATITUDE_MAX), bounds_.east()});

    return tileCover(Projection::project(bounds.northwest(), z),
                     Projection::project(bounds.northeast(), z),
                     Projection::project(bounds.southeast(), z),
                     Projection::project(bounds.southwest(), z),
                     Projection::project(bounds.center(), z),
                     z);
}

std::vector<UnwrappedTileID> tileCover(const Geometry<double>& geometry, uint8_t z) {
    std::vector<UnwrappedTileID> result;
    TileCover tc(geometry, z, true);
    while (tc.hasNext()) {
        result.push_back(*tc.next());
    };

    return result;
}

// Taken from https://github.com/mapbox/sphericalmercator#xyzbbox-zoom-tms_style-srs
// Computes the projected tiles for the lower left and upper right points of the bounds
// and uses that to compute the tile cover count
uint64_t tileCount(const LatLngBounds& bounds, uint8_t zoom) noexcept {
    if (zoom == 0) {
        return 1;
    }
    const auto sw = Projection::project(bounds.southwest(), zoom);
    const auto ne = Projection::project(bounds.northeast(), zoom);
    const auto maxTile = std::pow(2.0, zoom);
    const auto x1 = floor(sw.x);
    const auto x2 = ceil(ne.x) - 1;
    const auto y1 = util::clamp(floor(sw.y), 0.0, maxTile - 1);
    const auto y2 = util::clamp(floor(ne.y), 0.0, maxTile - 1);

    const auto dx = x1 > x2 ? (maxTile - x1) + x2 : x2 - x1;
    const auto dy = y1 - y2;
    return static_cast<uint64_t>((dx + 1) * (dy + 1));
}

uint64_t tileCount(const Geometry<double>& geometry, uint8_t z) {
    uint64_t tileCount = 0;

    TileCover tc(geometry, z, true);
    while (tc.next()) {
        tileCount++;
    };
    return tileCount;
}

TileCover::TileCover(const LatLngBounds& bounds_, uint8_t z) {
    LatLngBounds bounds = LatLngBounds::hull({std::max(bounds_.south(), -util::LATITUDE_MAX), bounds_.west()},
                                             {std::min(bounds_.north(), util::LATITUDE_MAX), bounds_.east()});

    if (bounds.isEmpty() || bounds.south() > util::LATITUDE_MAX || bounds.north() < -util::LATITUDE_MAX) {
        bounds = LatLngBounds::world();
    }

    const auto sw = Projection::project(bounds.southwest(), z);
    const auto ne = Projection::project(bounds.northeast(), z);
    const auto se = Projection::project(bounds.southeast(), z);
    const auto nw = Projection::project(bounds.northwest(), z);

    const Polygon<double> p({{sw, nw, ne, se, sw}});
    impl = std::make_unique<TileCover::Impl>(z, p, false);
}

TileCover::TileCover(const Geometry<double>& geom, uint8_t z, bool project /* = true*/)
    : impl(std::make_unique<TileCover::Impl>(z, geom, project)) {}

TileCover::~TileCover() = default;

std::optional<UnwrappedTileID> TileCover::next() {
    return impl->next();
}

bool TileCover::hasNext() {
    return impl->hasNext();
}

} // namespace util
} // namespace mln
