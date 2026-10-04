#include <mln/test/util.hpp>

#include <mln/renderer/terrain_ecef_lod.hpp>
#include <mln/util/ecef.hpp>

#include <cmath>
#include <set>
#include <tuple>

using namespace mln;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Camara de campo tipica del modo horizonte: ojo a 2 m sobre un suelo de 25 m, mirando 10 grados
// bajo el horizonte (pitch 80), FOV 45 y pantalla vertical de 1080x2400 px fisicos.
struct TestView {
    double latDeg = -34.6;
    double lonDeg = -58.4;
    double groundM = 25.0;
    double eyeM = 2.0;
    double pitchDeg = 80.0;
    double bearingDeg = 0.0;
};

vec3 normalize(const vec3& v) {
    const double l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return {v[0] / l, v[1] / l, v[2] / l};
}

vec3 cross(const vec3& a, const vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

EcefLodCamera makeCamera(const TestView& v) {
    constexpr double widthPx = 1080.0;
    constexpr double heightPx = 2400.0;
    constexpr double fov = 45.0 * kPi / 180.0;
    const double farM = 3570.0 * std::sqrt(v.eyeM) * 1.5;
    const double nearM = v.eyeM * 0.2;

    const LatLng ll(v.latDeg, v.lonDeg);
    const vec3 up = util::ecef::surfaceNormal(ll);
    const vec3 east = normalize(cross({0.0, 0.0, 1.0}, up));
    const vec3 north = normalize(cross(up, east));
    const double p = v.pitchDeg * kPi / 180.0;
    const double b = v.bearingDeg * kPi / 180.0;
    vec3 forward;
    for (int i = 0; i < 3; ++i) {
        forward[i] = east[i] * std::sin(b) * std::sin(p) + north[i] * std::cos(b) * std::sin(p) - up[i] * std::cos(p);
    }
    forward = normalize(forward);
    const vec3 right = normalize(cross(forward, up));
    const vec3 camUp = normalize(cross(right, forward));

    mat4 view{};
    for (int i = 0; i < 3; ++i) {
        view[i * 4 + 0] = right[i];
        view[i * 4 + 1] = camUp[i];
        view[i * 4 + 2] = -forward[i];
    }
    view[15] = 1.0;

    const double f = 1.0 / std::tan(fov / 2.0);
    mat4 proj{};
    proj[0] = f / (widthPx / heightPx);
    proj[5] = f;
    proj[10] = (farM + nearM) / (nearM - farM);
    proj[11] = -1.0;
    proj[14] = 2.0 * farM * nearM / (nearM - farM);

    EcefLodCamera camera;
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) {
                sum += proj[k * 4 + r] * view[c * 4 + k];
            }
            camera.viewProjRte[c * 4 + r] = sum;
        }
    }
    camera.originEcef = util::ecef::llaToEcef(ll, v.groundM + v.eyeM);
    camera.mercatorX = (v.lonDeg + 180.0) / 360.0;
    camera.mercatorY = (1.0 - std::log(std::tan(kPi / 4.0 + v.latDeg * kPi / 360.0)) / kPi) / 2.0;
    camera.farM = farM;
    camera.lambda = (heightPx / 2.0) / std::tan(fov / 2.0);
    return camera;
}

// Tiles DEM cargados como los trae la cobertura omnidireccional: z15 bajo la camara y su padre z10.
EcefDemIndex makeDems(const EcefLodCamera& camera) {
    EcefDemIndex dems;
    for (const uint8_t z : {uint8_t{10}, uint8_t{15}}) {
        const double count = std::pow(2.0, z);
        const EcefLodTileKey key{z, static_cast<uint32_t>(camera.mercatorX * count),
                                 static_cast<uint32_t>(camera.mercatorY * count)};
        dems.emplace(key, EcefDemRange{20.0, 30.0});
    }
    return dems;
}

struct Box {
    double x0, y0, x1, y1;
};

Box boxOf(const EcefLodCell& c) {
    const double count = std::pow(2.0, c.tile.z);
    const double x = c.tile.x + c.wrap * count;
    return {x / count, c.tile.y / count, (x + 1.0) / count, (c.tile.y + 1.0) / count};
}

bool shareEdge(const Box& a, const Box& b) {
    constexpr double eps = 1e-12;
    const bool touchX = std::abs(a.x1 - b.x0) < eps || std::abs(b.x1 - a.x0) < eps;
    const bool touchY = std::abs(a.y1 - b.y0) < eps || std::abs(b.y1 - a.y0) < eps;
    const bool overlapY = std::min(a.y1, b.y1) - std::max(a.y0, b.y0) > eps;
    const bool overlapX = std::min(a.x1, b.x1) - std::max(a.x0, b.x0) > eps;
    return (touchX && overlapY) || (touchY && overlapX);
}

} // namespace

TEST(TerrainEcefLod, GroundUnderTheEyeIsCovered) {
    const EcefLodCamera camera = makeCamera({});
    const EcefLodResult result = selectEcefTerrainCells(camera, makeDems(camera));
    ASSERT_FALSE(result.cells.empty());
    EXPECT_LT(result.cells.front().distanceM, 10.0);
    EXPECT_GE(result.cells.front().tile.z, 20);
}

TEST(TerrainEcefLod, CellCountInAtakRange) {
    for (const double pitch : {45.0, 80.0, 84.0}) {
        TestView view;
        view.pitchDeg = pitch;
        const EcefLodCamera camera = makeCamera(view);
        const EcefLodResult result = selectEcefTerrainCells(camera, makeDems(camera));
        EXPECT_FALSE(result.fuseTripped) << "pitch " << pitch;
        EXPECT_GT(result.cells.size(), 4u) << "pitch " << pitch;
        EXPECT_LT(result.cells.size(), 400u) << "pitch " << pitch;
    }
}

// Mirando al piso desde 2 m la caja de cada celda baja 500 m (skirt, igual que ATAK), asi que entran
// mas celdas; tiene que seguir lejos del fusible.
TEST(TerrainEcefLod, NadirStaysBelowTheFuse) {
    TestView view;
    view.pitchDeg = 0.0;
    const EcefLodCamera camera = makeCamera(view);
    const EcefLodResult result = selectEcefTerrainCells(camera, makeDems(camera));
    EXPECT_FALSE(result.fuseTripped);
    EXPECT_LT(result.cells.size(), 1000u);
}

TEST(TerrainEcefLod, NoCellBeyondFarPlane) {
    const EcefLodCamera camera = makeCamera({});
    for (const EcefLodCell& cell : selectEcefTerrainCells(camera, makeDems(camera)).cells) {
        EXPECT_LE(cell.distanceM, camera.farM);
    }
}

TEST(TerrainEcefLod, NeighborsDifferByAtMostOneLevel) {
    const EcefLodCamera camera = makeCamera({});
    const auto cells = selectEcefTerrainCells(camera, makeDems(camera)).cells;
    int maxZ = 0;
    for (const auto& c : cells) maxZ = std::max<int>(maxZ, c.tile.z);
    for (const auto& a : cells) {
        if (maxZ - a.tile.z >= 7) continue;  // exentas, como en ATAK
        for (const auto& b : cells) {
            if (b.tile.z >= a.tile.z + 2 && shareEdge(boxOf(a), boxOf(b))) {
                ADD_FAILURE() << "z" << int(a.tile.z) << " pegada a z" << int(b.tile.z);
            }
        }
    }
}

TEST(TerrainEcefLod, CellsWithoutDemAreFlatButPresent) {
    const EcefLodCamera camera = makeCamera({});
    const EcefLodResult result = selectEcefTerrainCells(camera, {});
    ASSERT_FALSE(result.cells.empty());
    EXPECT_LT(result.cells.front().distanceM, 60.0);
    for (const auto& cell : result.cells) {
        EXPECT_FALSE(cell.dem.has_value());
    }
}

TEST(TerrainEcefLod, DemWindowIsInsideTheAncestor) {
    const EcefLodCamera camera = makeCamera({});
    const auto cells = selectEcefTerrainCells(camera, makeDems(camera)).cells;
    size_t withDem = 0;
    for (const auto& cell : cells) {
        if (!cell.dem) continue;
        ++withDem;
        const int shift = cell.tile.z - cell.dem->z;
        ASSERT_GE(shift, 0);
        EXPECT_EQ(cell.tile.x >> shift, cell.dem->x);
        EXPECT_EQ(cell.tile.y >> shift, cell.dem->y);
        EXPECT_FLOAT_EQ(cell.demScale, 1.0f / static_cast<float>(1u << shift));
        EXPECT_GE(cell.demOffsetX, 0.0f);
        EXPECT_LE(cell.demOffsetX + cell.demScale, 1.0f + 1e-6f);
        EXPECT_GE(cell.demOffsetY, 0.0f);
        EXPECT_LE(cell.demOffsetY + cell.demScale, 1.0f + 1e-6f);
    }
    EXPECT_GT(withDem, 0u);
}

TEST(TerrainEcefLod, GlobeIsDrawnOnce) {
    // Vista de globo (ojo a 20.000 km). En ECEF hay un solo planeta: con tres copias del mundo el globo se dibujaba
    // tres veces en el mismo lugar, con la imagen corrida un mundo entero (globo rayado, field-test 01-10).
    TestView view;
    view.eyeM = 2.0e7;
    view.pitchDeg = 5.0;
    EcefLodCamera camera = makeCamera(view);
    camera.farM = 6.0e7; // far de la camara orbital alejada: alcanza todo el hemisferio a la vista
    const auto cells = selectEcefTerrainCells(camera, {}).cells;
    ASSERT_FALSE(cells.empty());
    std::set<std::tuple<int, uint32_t, uint32_t>> keys;
    for (const auto& c : cells) {
        EXPECT_TRUE(keys.insert({c.tile.z, c.tile.x, c.tile.y}).second) << "celda repetida z" << int(c.tile.z);
    }
    for (const auto& c : cells) {
        for (int z = 0; z < c.tile.z; ++z) {
            const int shift = c.tile.z - z;
            EXPECT_EQ(keys.count({z, c.tile.x >> shift, c.tile.y >> shift}), 0u) << "celda dentro de otra";
        }
    }
}

TEST(TerrainEcefLod, CellsAcrossTheAntimeridianUseTheFocusCopy) {
    // Camara sobre el antimeridiano mirando al este: las celdas del otro lado son vecinas (distancia real) y se
    // entregan en la copia del mundo del foco, para que su UV caiga dentro de los drapes.
    TestView view;
    view.latDeg = 0.0;
    view.lonDeg = 179.99;
    view.eyeM = 2000.0;
    view.pitchDeg = 60.0;
    view.bearingDeg = 90.0;
    EcefLodCamera camera = makeCamera(view);
    camera.focusMercatorX = camera.mercatorX;
    const auto cells = selectEcefTerrainCells(camera, {}).cells;
    bool nearAcross = false;
    for (const auto& c : cells) {
        const double count = std::pow(2.0, c.tile.z);
        const double centerX = (c.tile.x + 0.5) / count + c.wrap;
        EXPECT_LE(std::abs(centerX - camera.focusMercatorX), 0.5);
        nearAcross = nearAcross || (c.wrap == 1 && c.distanceM < 5000.0);
    }
    EXPECT_TRUE(nearAcross);
}
