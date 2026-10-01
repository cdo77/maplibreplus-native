#include <mln/test/util.hpp>

#include <mln/map/transform_state.hpp>
#include <mln/util/ecef.hpp>

#include <cmath>

using namespace mln;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFocusElevationM = 280.0;

// Estado de un telefono vertical (1080x2400) mirando Neuquen, con la camara real (orbital) prendida.
TransformState makeState(double zoom, double pitchDeg, double bearingDeg) {
    TransformState state;
    state.setSize({1080, 2400});
    state.setLatLngZoom(LatLng(-38.95, -68.05), zoom);
    state.setPitch(pitchDeg * kPi / 180.0);
    // Convencion interna de MapLibre: -rumbo de brujula (Transform: deg2rad(-camera.bearing)).
    state.setBearing(-bearingDeg * kPi / 180.0);
    state.setRealAltitudeMode(true);
    return state;
}

double distance(const vec3& a, const vec3& b) {
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

} // namespace

TEST(EcefCamera, EyeIsAtRangeFromFocus) {
    const auto cam = makeState(16.0, 60.0, 30.0).computeEcefCamera(kFocusElevationM);
    EXPECT_FALSE(cam.collided);
    EXPECT_NEAR(distance(cam.eyeEcef, cam.focusEcef), cam.rangeM, 1e-6 * cam.rangeM);
    // El foco es el centro del mapa, sobre el terreno.
    EXPECT_NEAR(cam.focus.latitude(), -38.95, 1e-9);
    EXPECT_NEAR(cam.focus.longitude(), -68.05, 1e-9);
    // Tilt 60: el ojo queda a range x cos(60) sobre el foco (el suelo plano a la altura del foco).
    EXPECT_NEAR(cam.eyeAglM, cam.rangeM * 0.5, 0.01 * cam.rangeM);
}

TEST(EcefCamera, NadirEyeIsAboveTheFocus) {
    const auto cam = makeState(16.0, 0.0, 0.0).computeEcefCamera(kFocusElevationM);
    EXPECT_NEAR(cam.eyeLatLng.latitude(), cam.focus.latitude(), 1e-6);
    EXPECT_NEAR(cam.eyeLatLng.longitude(), cam.focus.longitude(), 1e-6);
    EXPECT_NEAR(cam.eyeAglM, cam.rangeM, 0.01);
}

TEST(EcefCamera, RangeFollowsZoomLikeAtakGsd) {
    const auto near = makeState(17.0, 45.0, 0.0).computeEcefCamera(kFocusElevationM);
    const auto far = makeState(16.0, 45.0, 0.0).computeEcefCamera(kFocusElevationM);
    // Un nivel de zoom menos = el doble de distancia (range = gsd x (H/2) / tan(fov/2)).
    EXPECT_NEAR(far.rangeM / near.rangeM, 2.0, 1e-6);
    // fov 45 en modo camara real: range = 1200 px / tan(22,5) x metros/px.
    const double metersPerPixel = Projection::getMetersPerPixelAtLatitude(-38.95, 16.0);
    EXPECT_NEAR(far.rangeM, 1200.0 / std::tan(22.5 * kPi / 180.0) * metersPerPixel, 1e-3 * far.rangeM);
}

TEST(EcefCamera, CollisionRaisesTheEyeAndKeepsTheFocus) {
    const auto state = makeState(20.0, 85.0, 0.0);
    const auto free = state.computeEcefCamera(kFocusElevationM);
    // Un cerro de 100 m detras del foco, justo donde cae el ojo.
    const auto cam = state.computeEcefCamera(kFocusElevationM, [](const LatLng&) { return kFocusElevationM + 100.0; });
    EXPECT_TRUE(cam.collided);
    EXPECT_NEAR(cam.eyeAglM, 10.0, 1e-3);
    EXPECT_NEAR(distance(cam.focusEcef, free.focusEcef), 0.0, 1e-9);
    // La vista sigue apuntando al foco.
    const double toFocus = distance(cam.eyeEcef, cam.focusEcef);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(cam.eyeEcef[i] + cam.forward[i] * toFocus, cam.focusEcef[i], 1e-3);
    }
}

TEST(EcefCamera, CompassBearingTurnsLikeTheMercatorCamera) {
    // Rumbo de brujula 90 (este) mirando al horizonte: la vista apunta al este y el ojo queda al oeste del foco.
    const auto cam = makeState(16.0, 89.0, 90.0).computeEcefCamera(kFocusElevationM);
    const vec3 east = {-std::sin(-68.05 * kPi / 180.0), std::cos(-68.05 * kPi / 180.0), 0.0};
    const double forwardEast = cam.forward[0] * east[0] + cam.forward[1] * east[1] + cam.forward[2] * east[2];
    EXPECT_GT(forwardEast, 0.99);
    EXPECT_LT(cam.eyeLatLng.longitude(), cam.focus.longitude());
}

TEST(EcefCamera, NadirViewKeepsTheBearing) {
    // 2D (tilt 0) con rumbo 90: arriba de la pantalla es el este. Al nadir la derecha no puede salir de
    // forward x up (se anula); tiene que salir del rumbo.
    const auto state = makeState(16.0, 0.0, 90.0);
    const auto cam = state.computeEcefCamera(kFocusElevationM);
    const mat4 m = state.getEcefTileMatrix(cam.focusEcef, cam);
    const vec3 east = util::ecef::llaToEcef(LatLng(-38.95, -68.04), kFocusElevationM);
    const double p[3] = {east[0] - cam.focusEcef[0], east[1] - cam.focusEcef[1], east[2] - cam.focusEcef[2]};
    const double x = m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12];
    const double y = m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13];
    const double w = m[3] * p[0] + m[7] * p[1] + m[11] * p[2] + m[15];
    EXPECT_GT(y / w, 0.05);
    EXPECT_LT(std::abs(x / w), 0.01);
}

TEST(EcefCamera, DrapeAreasStayValidWhenZoomedOut) {
    // Muy alejado el radio del drape superaba el de la Tierra, la cuantizacion mandaba el centro al centro de la
    // Tierra y la latitud salia NaN en cada frame (render trabado, field-test 01-10).
    for (const double zoom : {0.0, 1.0, 2.0, 3.0, 4.0}) {
        for (const double pitch : {0.0, 60.0, 84.0}) {
            const auto state = makeState(zoom, pitch, 328.0);
            std::array<TransformState::EcefDrapeArea, TransformState::kEcefDrapeCount> areas{};
            ASSERT_NO_THROW(areas = state.computeEcefDrapeAreas()) << "zoom " << zoom << " pitch " << pitch;
            for (const auto& area : areas) {
                EXPECT_TRUE(std::isfinite(area.center.latitude()));
                EXPECT_LE(area.radiusMeters, 2.0 * kPi * 6378137.0 / 4.0 + 1.0);
            }
        }
    }
}

TEST(EcefCamera, PolarAxisConvertsWithoutNaN) {
    double height = 0.0;
    const LatLng north = util::ecef::ecefToLatLng({0.0, 0.0, 7000000.0}, &height);
    EXPECT_DOUBLE_EQ(north.latitude(), 90.0);
    EXPECT_NEAR(height, 7000000.0 - util::ecef::WGS84_SEMI_MINOR_M, 1e-6);
    EXPECT_NO_THROW(util::ecef::ecefToLatLng({0.0, 0.0, 0.0}));
}

TEST(EcefCamera, PlanesFollowTheEyeHeight) {
    const auto cam = makeState(16.0, 60.0, 0.0).computeEcefCamera(kFocusElevationM);
    EXPECT_NEAR(cam.nearM, cam.eyeAglM * 0.2, 1e-9);
    EXPECT_GT(cam.farM, cam.rangeM);
}
