#include <mln/test/util.hpp>

#include <mln/map/transform_state.hpp>
#include <mln/util/ecef.hpp>
#include <mln/util/globe.hpp>
#include <mln/util/mat4.hpp>

#include <cmath>

using namespace mln;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFocusElevationM = 280.0;

// Estado de un telefono vertical (1080x2400) mirando Neuquen, con la camara real (orbital) prendida.
TransformState makeState(double zoom, double pitchDeg, double bearingDeg, double latDeg = -38.95) {
    TransformState state;
    state.setSize({1080, 2400});
    state.setLatLngZoom(LatLng(latDeg, -68.05), zoom);
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
    // ADR 0043: el GSD de ATAK (MapSceneModel2_gsd) es geometria pura, sin termino de latitud -- range
    // siempre con el ecuador como referencia, NUNCA con la latitud del foco (-38,95 en este test), para que
    // un pan (que cambia la latitud) nunca mueva la distancia real de la camara.
    // fov 45 en modo camara real: range = 1200 px / tan(22,5) x metros/px.
    const double metersPerPixel = Projection::getMetersPerPixelAtLatitude(0.0, 16.0);
    EXPECT_NEAR(far.rangeM, 1200.0 / std::tan(22.5 * kPi / 180.0) * metersPerPixel, 1e-3 * far.rangeM);
}

TEST(EcefCamera, RangeIsIndependentOfFocusLatitude) {
    // ADR 0043: nuestro bug de origen -- range dependia de cos(latitud del foco), asi que CUALQUIER pan que
    // cambiara la latitud corria la distancia real de la camara sin que el zoom guardado se tocara (field-test
    // 04-10, "con un dedo hace zoom", "se mueve al reves e impreciso"). Mismo zoom/tilt/bearing, focos en
    // latitudes muy distintas: el range tiene que ser EXACTAMENTE el mismo.
    const double rangeEquator = makeState(14.0, 30.0, 0.0, 0.0).computeEcefCamera(0.0).rangeM;
    for (const double lat : {10.0, 45.0, -34.6, 80.0, -89.0}) {
        const auto cam = makeState(14.0, 30.0, 0.0, lat).computeEcefCamera(0.0);
        EXPECT_NEAR(cam.rangeM, rangeEquator, 1e-6 * rangeEquator) << lat  << lat;
    }
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
                EXPECT_LE(area.radiusMeters, kPi * 6378137.0 + 1.0);
            }
        }
    }
}

TEST(EcefCamera, ZoomedOutDrapeIsTheWholeWorld) {
    // Vista de globo: el drape grueso es el planisferio entero (centro en el ecuador, medio meridiano de radio), asi
    // su recorte mercator cubre lon +-180 y lat +-85 y no queda borde estirado sobre el hemisferio a la vista.
    // Zoom 3: con una pantalla de 2400 px el mapa no deja alejarse mas sin mover el centro al ecuador.
    const auto far = makeState(3.0, 0.0, 0.0).computeEcefDrapeAreas().back();
    EXPECT_TRUE(far.global);
    // Centro fijo en (0, 0): el covering del planisferio no cambia con los gestos.
    EXPECT_DOUBLE_EQ(far.center.latitude(), 0.0);
    EXPECT_DOUBLE_EQ(far.center.longitude(), 0.0);
    EXPECT_NEAR(far.radiusMeters, kPi * 6378137.0, 1.0);
    // De cerca sigue centrado en el foco.
    for (const auto& area : makeState(16.0, 60.0, 0.0).computeEcefDrapeAreas()) {
        EXPECT_FALSE(area.global);
    }
}

TEST(EcefCamera, NoNonGlobalDrapeCollapsesToZoomZeroNearThePoles) {
    // Cerca de un polo cos(lat)->0 y el zoom mercator del covering sintetico cae a 0 con radios menores al cuarto de
    // meridiano (imagen borrosa: tesela z0 para un area de resolucion moderada). Invariante: un drape que no es el
    // planisferio siempre puede pedir mas que la tesela z0 a la latitud de su centro.
    for (const double lat : {60.0, 75.0, 80.0, 84.0, 85.0, -84.0}) {
        for (double zoom = 2.0; zoom <= 16.0; zoom += 1.0) {
            const auto areas = makeState(zoom, 0.0, 0.0, lat).computeEcefDrapeAreas();
            for (size_t i = 0; i < areas.size(); ++i) {
                if (areas[i].global) {
                    // Contiguos desde el mas grueso: los drapes mas gruesos que uno global tambien lo son.
                    for (size_t j = i + 1; j < areas.size(); ++j) {
                        EXPECT_TRUE(areas[j].global) << "lat " << lat << " zoom " << zoom << " drape " << j;
                    }
                    continue;
                }
                const double mppZoom0 = 2.0 * kPi * 6378137.0 * std::cos(areas[i].center.latitude() * kPi / 180.0) / 512.0;
                const double mpp = 2.0 * areas[i].radiusMeters / TransformState::kEcefDrapeTextureSizesPx[i];
                EXPECT_GT(mppZoom0, mpp * 0.95) << "lat " << lat << " zoom " << zoom << " drape " << i;
            }
        }
    }
}

TEST(EcefCamera, GlobeFocusCanReachAnyLatitude) {
    // Con la camara real el mundo mercator no tiene que llenar la pantalla: con zoom bajo el foco no se traba en el
    // ecuador (no se llegaba al hemisferio norte, field-test 01-10) ni se fuerza un zoom minimo.
    TransformState state;
    state.setSize({1080, 2400});
    state.setRealAltitudeMode(true);
    state.setLatLngZoom(LatLng(60.0, 30.0), 0.5);
    EXPECT_NEAR(state.getLatLng().latitude(), 60.0, 1e-6);
    EXPECT_NEAR(state.getLatLng().longitude(), 30.0, 1e-6);
    EXPECT_NEAR(state.getZoom(), 0.5, 1e-9);
    // Sin la camara real sigue el comportamiento mercator de siempre.
    TransformState flat;
    flat.setSize({1080, 2400});
    flat.setLatLngZoom(LatLng(60.0, 30.0), 0.5);
    EXPECT_LT(flat.getLatLng().latitude(), 60.0);
}

TEST(EcefCamera, SyntheticDrapeViewSpansTheWholeDrape) {
    // La vista sintetica de cada drape (de la que sale el covering de su imagen) tiene que abarcar el diametro entero
    // del drape: con 256 px por tile abarcaba la mitad (media Tierra sin imagen en el globo, field-test 01-10).
    const auto state = makeState(16.0, 60.0, 0.0);
    for (const auto& area : {TransformState::EcefDrapeArea{.center = LatLng(0.0, 0.0), .radiusMeters = kPi * 6378137.0},
                             TransformState::EcefDrapeArea{.center = LatLng(-38.95, -68.05), .radiusMeters = 20000.0}}) {
        for (const uint32_t sizePx : {1024u, 2048u}) {
            const auto synthetic = TransformState::makeSyntheticDrapeState(state, area, sizePx);
            const double worldPx = 512.0 * std::pow(2.0, synthetic.getZoom());
            const double viewWidthM = sizePx / worldPx * 2.0 * kPi * 6378137.0 * std::cos(area.center.latitude() * kPi / 180.0);
            EXPECT_NEAR(viewWidthM, 2.0 * area.radiusMeters, 1e-6 * area.radiusMeters) << sizePx;
            EXPECT_NEAR(synthetic.getLatLng().latitude(), area.center.latitude(), 1e-9);
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

TEST(EcefCamera, PickingNadirMatchesFocus) {
    // ADR 0043: el picking (screenCoordinateToLatLng con la camara real) usa la MISMA camara que dibuja la
    // escena. Al nadir, el centro de pantalla tiene que caer exactamente sobre el foco.
    const auto state = makeState(10.0, 0.0, 0.0);
    const auto size = state.getSize();
    const auto hit = state.screenCoordinateToLatLng({static_cast<float>(size.width) / 2.0f, static_cast<float>(size.height) / 2.0f});
    EXPECT_NEAR(hit.latitude(), -38.95, 1e-6);
    EXPECT_NEAR(hit.longitude(), -68.05, 1e-6);
}

TEST(EcefCamera, PickingNeverNaN) {
    // Barrido de camaras (cerca del polo, muy alejada, con tilt alto) y de toda la pantalla: el picking nunca
    // puede devolver NaN/infinito, ni con el rayo mirando al espacio mas alla del horizonte (ADR 0043).
    for (const double lat : {0.0, 60.0, 85.0, 89.9}) {
        for (const double zoom : {1.0, 5.0, 16.0}) {
            for (const double pitch : {0.0, 45.0, 84.0}) {
                TransformState state;
                state.setSize({1080, 2340});
                state.setLatLngZoom(LatLng(lat, 10.0), zoom);
                state.setPitch(pitch * kPi / 180.0);
                state.setRealAltitudeMode(true);
                for (const float y : {0.0f, 1170.0f, 2340.0f}) {
                    const auto hit = state.screenCoordinateToLatLng({540.0f, y});
                    ASSERT_TRUE(std::isfinite(hit.latitude())) << "lat " << lat << " zoom " << zoom << " pitch " << pitch << " y " << y;
                    ASSERT_TRUE(std::isfinite(hit.longitude()));
                    EXPECT_GE(hit.latitude(), -90.0);
                    EXPECT_LE(hit.latitude(), 90.0);
                }
            }
        }
    }
}

TEST(EcefCamera, PickingMatchesRenderedCameraNotFlatMercator) {
    // La causa raiz del field-test 04-10 ("con un dedo hace zoom", "se queda pegado en el polo"): el picking
    // usaba la proyeccion mercator plana, una camara DISTINTA de la que dibuja la escena real. Con tilt alto
    // cerca de un polo las dos divergen fuerte; este test fija que el picking real (ECEF) no coincida con el
    // mercator plano ahi -- si algun dia coinciden otra vez es porque alguien volvio a mezclar las camaras.
    TransformState state;
    state.setSize({1080, 2340});
    state.setLatLngZoom(LatLng(80.0, 0.0), 2.0);
    state.setPitch(10.0 * kPi / 180.0);
    state.setRealAltitudeMode(true);
    // y=2250 (cerca del borde inferior): con el signo de ndcY corregido (field-test 04-10, "norte y sur esta
    // invertido"), un punto de pantalla cercano al centro puede caer justo en un cruce casual entre los dos
    // caminos -- lejos del centro la divergencia real es grande y estable (~26 grados aca).
    const auto ecefHit = state.screenCoordinateToLatLng({540.0f, 2250.0f});

    TransformState flat = state;
    flat.setRealAltitudeMode(false);
    const auto flatHit = flat.screenCoordinateToLatLng({540.0f, 2250.0f});

    EXPECT_GT(std::abs(ecefHit.latitude() - flatHit.latitude()), 10.0);
}

TEST(EcefCamera, PickingHorizontalSignMatchesFlatMercator) {
    // Field-test 04-10: el picking ECEF daba longitud al reves del mercator plano en los puntos descentrados
    // (izquierda del centro => longitud ESTE en vez de OESTE), y el pan "se movia al reves en los 4 sentidos".
    // De cerca (zoom/tilt moderados) los dos caminos tienen que coincidir en el SIGNO del desplazamiento,
    // aunque no en la magnitud exacta (la curvatura real no es la del mercator plano).
    TransformState state;
    state.setSize({1080, 2340});
    state.setLatLngZoom(LatLng(-34.6, -58.4), 15.0);
    state.setPitch(30.0 * kPi / 180.0);
    state.setRealAltitudeMode(true);
    const auto ecefCenter = state.screenCoordinateToLatLng({540.0f, 1170.0f});
    const auto ecefLeft = state.screenCoordinateToLatLng({300.0f, 1170.0f});
    const auto ecefRight = state.screenCoordinateToLatLng({780.0f, 1170.0f});

    state.setRealAltitudeMode(false);
    const auto flatCenter = state.screenCoordinateToLatLng({540.0f, 1170.0f});
    const auto flatLeft = state.screenCoordinateToLatLng({300.0f, 1170.0f});
    const auto flatRight = state.screenCoordinateToLatLng({780.0f, 1170.0f});

    // Izquierda del centro tiene que quedar al OESTE (longitud menor) en los dos caminos, y derecha al ESTE.
    EXPECT_LT(ecefLeft.longitude(), ecefCenter.longitude());
    EXPECT_GT(ecefRight.longitude(), ecefCenter.longitude());
    EXPECT_LT(flatLeft.longitude(), flatCenter.longitude());
    EXPECT_GT(flatRight.longitude(), flatCenter.longitude());
}

TEST(EcefCamera, PickingVerticalSignMatchesFlatMercator) {
    // Field-test 04-10 (ronda siguiente a la horizontal): arriba de pantalla daba NORTE en vez de sur -- el
    // eje vertical nunca se habia verificado con la misma sonda que el horizontal. Mismo patron que el test
    // de arriba, en Y.
    TransformState state;
    state.setSize({1080, 2340});
    state.setLatLngZoom(LatLng(-34.6, -58.4), 15.0);
    state.setPitch(30.0 * kPi / 180.0);
    state.setRealAltitudeMode(true);
    const auto ecefCenter = state.screenCoordinateToLatLng({540.0f, 1170.0f});
    const auto ecefTop = state.screenCoordinateToLatLng({540.0f, 900.0f});
    const auto ecefBottom = state.screenCoordinateToLatLng({540.0f, 1440.0f});

    state.setRealAltitudeMode(false);
    const auto flatCenter = state.screenCoordinateToLatLng({540.0f, 1170.0f});
    const auto flatTop = state.screenCoordinateToLatLng({540.0f, 900.0f});
    const auto flatBottom = state.screenCoordinateToLatLng({540.0f, 1440.0f});

    // Arriba de pantalla tiene que quedar al SUR (latitud menor) en los dos caminos, y abajo al NORTE.
    EXPECT_LT(ecefTop.latitude(), ecefCenter.latitude());
    EXPECT_GT(ecefBottom.latitude(), ecefCenter.latitude());
    EXPECT_LT(flatTop.latitude(), flatCenter.latitude());
    EXPECT_GT(flatBottom.latitude(), flatCenter.latitude());
}

TEST(EcefCamera, EyeMatchesAtakFourRotationConstruction) {
    // ADR 0043 §6.5: verificacion numerica de que nuestra formula vectorial directa del ojo/up
    // (getCameraForwardEcef + base por rumbo) coincide EXACTO con la construccion de ATAK
    // (MapSceneModel2::computeCameraEllipsoidal: traslade al foco + 4 rotaciones de matriz encadenadas
    // -- lon+90 en Z, 90-lat en X, -azimut en Z, tilt en X -- solo lectura GPLv3, no copiado). Repite,
    // con las utilidades de matriz propias, la MISMA secuencia de rotaciones para comparar.
    struct Case { double lat, lon, azimuthDeg, tiltDeg; };
    const Case cases[] = {
        {-34.6, -58.4, 0.0, 0.0}, {-34.6, -58.4, 0.0, 45.0}, {-34.6, -58.4, 90.0, 30.0},
        {-34.6, -58.4, 200.0, 60.0}, {60.0, 30.0, 45.0, 70.0}, {85.0, 10.0, 300.0, 80.0},
        {0.0, 0.0, 15.0, 20.0}, {-85.0, 170.0, 250.0, 10.0},
    };
    namespace globe = util::globe;
    constexpr double range = 5000.0;
    for (const auto& c : cases) {
        TransformState state;
        state.setSize({1080, 2400});
        state.setLatLngZoom(LatLng(c.lat, c.lon), 16.0);
        state.setPitch(c.tiltDeg * kPi / 180.0);
        state.setBearing(-c.azimuthDeg * kPi / 180.0);
        state.setRealAltitudeMode(true);

        const vec3 focoEcef = util::ecef::llaToEcef(LatLng(c.lat, c.lon), 0.0);
        const vec3 forward = state.getCameraForwardEcef();
        const vec3 ourEye = globe::add(focoEcef, globe::scale(forward, -range));

        mat4 m = matrix::identity4();
        matrix::translate(m, m, focoEcef[0], focoEcef[1], focoEcef[2]);
        matrix::rotate_z(m, m, (c.lon + 90.0) * kPi / 180.0);
        matrix::rotate_x(m, m, (90.0 - c.lat) * kPi / 180.0);
        matrix::rotate_z(m, m, -c.azimuthDeg * kPi / 180.0);
        matrix::rotate_x(m, m, c.tiltDeg * kPi / 180.0);
        const vec3 atakEye = globe::add(focoEcef, globe::scale(vec3{m[8], m[9], m[10]}, range));

        EXPECT_NEAR(globe::length(globe::subtract(ourEye, atakEye)), 0.0, 1e-6)
            << "lat " << c.lat << " lon " << c.lon << " az " << c.azimuthDeg << " tilt " << c.tiltDeg;
    }
}


TEST(EcefCamera, RoundTripScreenToLatLngToScreenMatchesForOnGlobeHits) {
    // La deteccion del folback de picking (AtakNavigation.kt, panTo) compara el pixel original contra
    // `toScreenLocation(fromScreenLocation(pixel))`: si el pick fue un hit real (no el folback al punto mas
    // cercano del horizonte), la vuelta tiene que caer MUY cerca del pixel de entrada. Esto solo es valido si
    // la proyeccion inversa (latLngToScreenCoordinate) usa la MISMA camara real -- antes caia siempre por el
    // camino mercator plano, asi que CUALQUIER punto con la camara real activa daba una vuelta mal, no solo
    // los del folback (field-test 04-10: el pan de un dedo dejo de responder por completo al agregar el
    // chequeo de ida y vuelta, porque la vuelta estaba rota en general).
    const auto state = makeState(10.0, 30.0, 0.0);
    const auto size = state.getSize();
    const ScreenCoordinate pixels[] = {
        {static_cast<float>(size.width) / 2.0f, static_cast<float>(size.height) / 2.0f},
        {static_cast<float>(size.width) * 0.3f, static_cast<float>(size.height) * 0.4f},
        {static_cast<float>(size.width) * 0.7f, static_cast<float>(size.height) * 0.6f},
    };
    for (const auto& px : pixels) {
        const auto hit = state.screenCoordinateToLatLng(px);
        const auto back = state.latLngToScreenCoordinate(hit);
        EXPECT_NEAR(back.x, px.x, 1.0) << "x pixel=" << px.x << "," << px.y;
        EXPECT_NEAR(back.y, px.y, 1.0) << "y pixel=" << px.x << "," << px.y;
    }
}

TEST(EcefCamera, RoundTripDivergesWhenPickFallsBackToTheHorizon) {
    // Contraparte del test anterior: un pixel que mira mas alla del horizonte (camara muy alejada, tilt alto,
    // borde de pantalla) cae en el folback "punto mas cercano del elipsoide", que NO esta sobre el rayo
    // original -- la vuelta tiene que caer lejos del pixel de entrada, para que AtakNavigation.kt pueda usar
    // esta divergencia como señal de "no hubo hit real, no usar este pick para panTo" (igual criterio que
    // ATAK: `sm.inverse(...) != TE_Ok` en panToImpl_perspective, solo lectura GPLv3, no copiado).
    TransformState state;
    state.setSize({1080, 2400});
    state.setLatLngZoom(LatLng(85.0, -60.0), 2.5);
    state.setPitch(60.0 * kPi / 180.0);
    state.setRealAltitudeMode(true);
    const auto size = state.getSize();
    // Borde superior de pantalla: con tilt alto y zoom de globo, el rayo apunta al espacio por encima del
    // horizonte -- cae en el folback.
    const ScreenCoordinate edgeTop{static_cast<float>(size.width) / 2.0f, 5.0f};
    const auto hit = state.screenCoordinateToLatLng(edgeTop);
    const auto back = state.latLngToScreenCoordinate(hit);
    const double dx = back.x - edgeTop.x;
    const double dy = back.y - edgeTop.y;
    EXPECT_GT(std::sqrt(dx * dx + dy * dy), 10.0);
}
