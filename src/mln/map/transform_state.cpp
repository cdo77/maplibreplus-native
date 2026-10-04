#include <mln/map/transform_state.hpp>
#include <mln/math/angles.hpp>
#include <mln/math/clamp.hpp>
#include <mln/math/log2.hpp>
#include <mln/tile/tile_id.hpp>
#include <mln/util/constants.hpp>
#include <mln/util/interpolate.hpp>
#include <mln/util/logging.hpp>
#include <mln/util/projection.hpp>
#include <mln/util/globe.hpp>
#include <mln/util/tile_coordinate.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

using namespace std::numbers;

namespace mln {

/*
 * The maximum angle to use for the Mercator horizon. This must be less than 90
 * to prevent errors in `MercatorTransform::_calcMatrices()`. It shouldn't be too close
 * to 90, or the distance to the horizon will become very large, unnecessarily increasing
 * the number of tiles needed to render the map.
 */
const double maxMercatorHorizonAngle = util::deg2rad(89.25);

namespace {
LatLng latLngFromMercator(Point<double> mercatorCoordinate, LatLng::WrapMode wrapMode = LatLng::WrapMode::Unwrapped) {
    return {util::rad2deg(2 * std::atan(std::exp(pi - mercatorCoordinate.y * 2 * pi)) - (pi / 2)),
            mercatorCoordinate.x * 360.0 - 180.0,
            wrapMode};
}
constexpr double kEpsilon = 1e-9;
// To avoid flickering issue due to "zoom = 13.9999999..".
double roundForAccuracy(double x) {
    double round_x = std::round(x);
    double diff = std::abs(round_x - x);
    if (diff < kEpsilon && diff > 0) {
        return round_x;
    } else {
        return x;
    }
}
} // namespace

TransformState::TransformState(ConstrainMode constrainMode_, ViewportMode viewportMode_)
    : bounds(LatLngBounds()),
      constrainMode(constrainMode_),
      viewportMode(viewportMode_) {}

void TransformState::setProperties(const TransformStateProperties& properties) {
    if (properties.x) {
        setX(*properties.x);
    }
    if (properties.y) {
        setY(*properties.y);
    }
    if (properties.z) {
        setZ(*properties.z);
    }
    if (properties.scale) {
        setScale(*properties.scale);
    }
    if (properties.bearing) {
        setBearing(*properties.bearing);
    }
    if (properties.fov) {
        setFieldOfView(*properties.fov);
    }
    if (properties.pitch) {
        setPitch(*properties.pitch);
    }
    if (properties.roll) {
        setRoll(*properties.roll);
    }
    if (properties.xSkew) {
        setXSkew(*properties.xSkew);
    }
    if (properties.ySkew) {
        setYSkew(*properties.ySkew);
    }
    if (properties.axonometric) {
        setAxonometric(*properties.axonometric);
    }
    if (properties.panning) {
        setPanningInProgress(*properties.panning);
    }
    if (properties.scaling) {
        setScalingInProgress(*properties.scaling);
    }
    if (properties.rotating) {
        setRotatingInProgress(*properties.rotating);
    }
    if (properties.edgeInsets) {
        setEdgeInsets(*properties.edgeInsets);
    }
    if (properties.size) {
        setSize(*properties.size);
    }
    if (properties.constrain) {
        setConstrainMode(*properties.constrain);
    }
    if (properties.northOrientation) {
        setNorthOrientation(*properties.northOrientation);
    }
    if (properties.viewPortMode) {
        setViewportMode(*properties.viewPortMode);
    }
    if (properties.frustumOffset) {
        setFrustumOffset(*properties.frustumOffset);
    }
}

// MARK: - Matrix

void TransformState::matrixFor(mat4& matrix, const UnwrappedTileID& tileID) const {
    const uint64_t tileScale = 1ull << tileID.canonical.z;
    const double s = Projection::worldSize(scale) / tileScale;

    matrix::identity(matrix);
    matrix::translate(matrix,
                      matrix,
                      int64_t(tileID.canonical.x + tileID.wrap * static_cast<int64_t>(tileScale)) * s,
                      int64_t(tileID.canonical.y) * s,
                      0);
    matrix::scale(matrix, matrix, s / util::EXTENT, s / util::EXTENT, 1);
}

void TransformState::getProjMatrix(mat4& projMatrix, uint16_t nearZ, bool aligned) const {
    if (size.isEmpty()) {
        return;
    }

    const double cameraToCenterDistance = getCameraToCenterDistance();
    const ScreenCoordinate offset = getCenterOffset();

    const double limitedPitch = util::clamp(getPitch(), 0.0, maxMercatorHorizonAngle);
    const double cameraToSeaLevelDistance = cameraToCenterDistance + std::abs(z) / std::cos(limitedPitch);

    // Find the Z distance from the viewport center point
    // [width/2 + offset.x, height/2 + offset.y] to the top edge; to point
    // [width/2 + offset.x, 0] in Z units.
    // 1 Z unit is equivalent to 1 horizontal px at the center of the map
    // (the distance between[width/2, height/2] and [width/2 + 1, height/2])
    // See https://github.com/mapbox/mapbox-gl-native/pull/15195 for details.
    // See TransformState::fov description: fov = 2 * arctan((height / 2) / (height * 1.5)).
    const double tanFovAboveCenter = (0.5 + (offset.y - frustumOffset.top()) / size.height) * 2.0 *
                                     std::tan(fov / 2.0) *
                                     (std::abs(std::cos(roll)) + std::abs(std::sin(roll)) * size.width / size.height);
    const double tanMultiple = util::clamp(tanFovAboveCenter * std::tan(limitedPitch), 0.0, 0.99);
    assert(tanMultiple < 1);
    // Calculate z distance of the farthest fragment that should be rendered.
    const double furthestDistance = cameraToSeaLevelDistance / (1 - tanMultiple);
    // Margen del plano lejano. 1% para evitar problemas de precision en el borde; ademas hasta 15%
    // para incluir el relieve del terreno 3D (ADR 0034): sin esto, los puntos por debajo del nivel
    // del mar de referencia quedan mas lejos que farZ y se recortan (agujeros negros en 2D+terreno).
    const double farZ = furthestDistance * 1.15;

    // Make sure the camera state is up-to-date
    updateCameraState();

    mat4 worldToCamera = camera.getWorldToCamera(scale, viewportMode == ViewportMode::FlippedY);
    mat4 cameraToClip = camera.getCameraToClipPerspective(
        getFieldOfView(), static_cast<double>(size.width) / size.height, nearZ, farZ);

    // Move the center of perspective to center of specified edgeInsets.
    // Values are in range [-1, 1] where the upper and lower range values
    // position viewport center to the screen edges. This is overridden
    // if using axonometric perspective (not in public API yet, Issue #11882).
    // TODO(astojilj): Issue #11882 should take edge insets into account, too.
    if (!axonometric) {
        cameraToClip[8] = -offset.x * 2.0 / size.width;
        cameraToClip[9] = offset.y * 2.0 / size.height;
    }

    // Apply north orientation angle
    if (getNorthOrientation() != NorthOrientation::Upwards) {
        matrix::rotate_z(cameraToClip, cameraToClip, -getNorthOrientationAngle());
    }

    matrix::multiply(projMatrix, cameraToClip, worldToCamera);

    if (axonometric) {
        // mat[11] controls perspective
        projMatrix[11] = 0.0;

        // mat[8], mat[9] control x-skew, y-skew
        double pixelsPerMeter = 1.0 / Projection::getMetersPerPixelAtLatitude(getLatLng().latitude(), getZoom());
        projMatrix[8] = xSkew * pixelsPerMeter;
        projMatrix[9] = ySkew * pixelsPerMeter;
    }

    // Make a second projection matrix that is aligned to a pixel grid for
    // rendering raster tiles. We're rounding the (floating point) x/y values to
    // achieve to avoid rendering raster images to fractional coordinates.
    // Additionally, we adjust by half a pixel in either direction in case that
    // viewport dimension is an odd integer to preserve rendering to the pixel
    // grid. We're rotating this shift based on the angle of the transformation
    // so that 0°, 90°, 180°, and 270° rasters are crisp, and adjust the shift
    // so that it is always <= 0.5 pixels.

    if (aligned) {
        const double worldSize = Projection::worldSize(scale);
        const double dx = x - 0.5 * worldSize;
        const double dy = y - 0.5 * worldSize;

        const auto xShift = static_cast<double>(size.width % 2) / 2.0;
        const auto yShift = static_cast<double>(size.height % 2) / 2.0;
        const double bearingCos = std::cos(bearing);
        const double bearingSin = std::sin(bearing);
        double devNull;
        const double dxa = -std::modf(dx, &devNull) + bearingCos * xShift + bearingSin * yShift;
        const double dya = -std::modf(dy, &devNull) + bearingCos * yShift + bearingSin * xShift;
        matrix::translate(projMatrix, projMatrix, dxa > 0.5 ? dxa - 1 : dxa, dya > 0.5 ? dya - 1 : dya, 0);
    }
}

void TransformState::updateCameraState() const {
    if (!valid()) {
        return;
    }

    const double worldSize = Projection::worldSize(scale);
    const double cameraToCenterDistance = getCameraToCenterDistance();

    // x & y tracks the center of the map in pixels. However as rendering is
    // done in pixel coordinates the rendering origo is actually in the middle
    // of the map (0.5 * worldSize). x&y positions have to be negated because it
    // defines position of the map, not the camera. Moving map 10 units left has
    // the same effect as moving camera 10 units to the right.
    const double dx = 0.5 * worldSize - x;
    const double dy = 0.5 * worldSize - y;

    // Set camera orientation and move it to a proper distance from the map
    camera.setOrientation(getRoll(), getPitch(), getBearing());

    const vec3 forward = camera.forward();
    const vec3 orbitPosition = {{-forward[0] * cameraToCenterDistance,
                                 -forward[1] * cameraToCenterDistance,
                                 -forward[2] * cameraToCenterDistance}};
    vec3 cameraPosition = {{dx + orbitPosition[0], dy + orbitPosition[1], z + orbitPosition[2]}};

    cameraPosition[0] /= worldSize;
    cameraPosition[1] /= worldSize;
    cameraPosition[2] /= worldSize;

    camera.setPosition(cameraPosition);
}

void TransformState::updateStateFromCamera() {
    const vec3 position = camera.getPosition();
    const vec3 forward = camera.forward();

    const double dx = forward[0];
    const double dy = forward[1];
    const double dz = forward[2];

    // Compute bearing and pitch
    double newBearing;
    double newPitch;
    double newRoll;
    camera.getOrientation(newRoll, newPitch, newBearing);
    newPitch = util::clamp(newPitch, minPitch, maxPitch);

    // Compute zoom level from the camera altitude
    const double centerDistance = getCameraToCenterDistance();
    double zoom;
    double newScale;
    double travel;
    if (dz < -1.0e-9 && position[2] > 1.0e-9 && newPitch <= maxMercatorHorizonAngle) {
        zoom = util::log2(centerDistance / (position[2] / std::cos(newPitch) * util::tileSize_D));
        newScale = util::clamp(std::pow(2.0, zoom), min_scale, max_scale);
        travel = -position[2] / dz;
    } else {
        zoom = 14;
        newScale = util::clamp(std::pow(2.0, zoom), min_scale, max_scale);
        travel = centerDistance / newScale / util::tileSize_D;
    }

    // Compute center point of the map
    const Point<double> mercatorPoint = {position[0] + dx * travel, position[1] + dy * travel};
    setLatLngZoom(latLngFromMercator(mercatorPoint), scaleZoom(newScale));

    const double mercatorZ = position[2] + dz * travel;
    double alt_m = mercatorZ * Projection::getMetersPerPixelAtLatitude(getLatLng().latitude(), 0) * util::tileSize_D;
    setCenterAltitude(alt_m);
    setBearing(newBearing);
    setPitch(newPitch);
    setRoll(newRoll);
}

FreeCameraOptions TransformState::getFreeCameraOptions() const {
    updateCameraState();

    FreeCameraOptions options;
    options.position = camera.getPosition();
    options.orientation = camera.getOrientation().m;

    return options;
}

bool TransformState::setCameraPosition(const vec3& position) {
    if (std::isnan(position[0]) || std::isnan(position[1]) || std::isnan(position[2])) return false;

    const double maxWorldSize = Projection::worldSize(std::pow(2.0, getMaxZoom()));
    const double minWorldSize = Projection::worldSize(std::pow(2.0, getMinZoom()));
    const double distToCenter = getCameraToCenterDistance();

    const vec3 updatedPos = vec3{
        {position[0], position[1], util::clamp(position[2], distToCenter / maxWorldSize, distToCenter / minWorldSize)}};

    camera.setPosition(updatedPos);
    return true;
}

bool TransformState::setCameraOrientation(const Quaternion& orientation_) {
    const vec4& c = orientation_.m;
    if (std::isnan(c[0]) || std::isnan(c[1]) || std::isnan(c[2]) || std::isnan(c[3])) {
        return false;
    }

    // Zero-length quaternions are not valid
    if (orientation_.length() == 0.0) {
        return false;
    }

    Quaternion unitQuat = orientation_.normalized();
    const vec3 forward = unitQuat.transform({{0.0, 0.0, -1.0}});
    const vec3 up = unitQuat.transform({{0.0, -1.0, 0.0}});

    const std::optional<Quaternion> updatedOrientation = util::Camera::orientationFromFrame(forward, up);
    if (!updatedOrientation) return false;

    camera.setOrientation(updatedOrientation.value());
    return true;
}

void TransformState::setFreeCameraOptions(const FreeCameraOptions& options) {
    if (!valid()) {
        return;
    }

    if (!options.position && !options.orientation) return;

    // Check if the state is dirty and camera needs to be synchronized
    updateMatricesIfNeeded();

    bool changed = false;
    if (options.orientation && options.orientation.value() != camera.getOrientation().m) {
        changed |= setCameraOrientation(options.orientation.value());
    }

    if (options.position && options.position.value() != camera.getPosition()) {
        changed |= setCameraPosition(options.position.value());
    }

    if (changed) {
        updateStateFromCamera();
        requestMatricesUpdate = true;
    }
}

// MARK: - Camara con altura real (ADR 0034, arco Globo/ECEF)

void TransformState::setRealAltitudeMode(bool enabled, double heightMetersAboveEllipsoid) {
    if (realAltitudeEnabled != enabled || realAltitudeMeters != heightMetersAboveEllipsoid) {
        realAltitudeEnabled = enabled;
        realAltitudeMeters = heightMetersAboveEllipsoid;
        // Camara orbital (ADR 0040): la mercator usa el mismo fov que la ECEF, asi son la misma camara.
        setFieldOfView(enabled ? kEcefFieldOfViewRad : util::DEFAULT_FOV);
        requestMatricesUpdate = true;
    }
}

TransformState::EcefCamera TransformState::computeEcefCamera(
    double focusElevationM, const std::function<double(const LatLng&)>& groundAt) const {
    namespace globe = util::globe;
    namespace ecef = util::ecef;
    // Radio de colision de la camara de ATAK (GLGlobe): el ojo nunca queda a menos de 10 m del terreno.
    constexpr double kCollideRadiusM = 10.0;

    EcefCamera cam;
    cam.focus = getLatLng();
    cam.focusElevationM = focusElevationM;
    // GSD de ATAK (MapSceneModel2_gsd/_range): metros/pixel = range * tan(FOV/2) / (alto/2), geometria
    // PURA, sin ningun termino de latitud. Si aca se usara cam.focus.latitude(), el metros/pixel del pixel
    // mercator estandar escala por cos(lat) -- correcto para un mapa 2D plano, pero mal para la camara
    // orbital: cada pan que cambia la latitud del foco corria la distancia real de la camara aunque el
    // zoom guardado no se tocara. El ADR 0040 original no lo noto porque los gestos normales cambian la
    // latitud muy poco por cuadro; field-test 04-10, a escala de globo, lo mostro ("con un dedo hace
    // zoom", "se mueve al reves e impreciso") -- ATAK nunca re-deriva el ojo desde un zoom guardado, lo
    // mantiene fijo durante el pan, por eso no le pasa (ver CameraController_panTo/panBy, solo lectura
    // GPLv3). Fix de raiz: la conversion zoom->metros/pixel de la camara real SIEMPRE usa el ecuador como
    // referencia, nunca la latitud del foco -- igual que el GSD de ATAK, deja de depender de hacia donde
    // mira la camara.
    cam.rangeM = getCameraToCenterDistance() * Projection::getMetersPerPixelAtLatitude(0.0, getZoom());
    cam.focusEcef = ecef::llaToEcef(cam.focus, focusElevationM);
    cam.forward = getCameraForwardEcef();
    cam.eyeEcef = globe::subtract(cam.focusEcef, globe::scale(cam.forward, cam.rangeM));
    double eyeHeightM = 0.0;
    cam.eyeLatLng = ecef::ecefToLatLng(cam.eyeEcef, &eyeHeightM);
    const double groundM = groundAt ? groundAt(cam.eyeLatLng) : focusElevationM;
    if (eyeHeightM - kCollideRadiusM < groundM) {
        // AdjustCamera de ATAK: sube el ojo por encima del terreno conservando el foco; la vista pasa a ser
        // ojo -> foco (equivale a bajar el tilt).
        const double adjM = groundM + kCollideRadiusM - eyeHeightM;
        cam.eyeEcef = globe::add(cam.eyeEcef, globe::scale(ecef::surfaceNormal(cam.eyeLatLng), adjM));
        cam.forward = globe::normalize(globe::subtract(cam.focusEcef, cam.eyeEcef));
        eyeHeightM += adjM;
        cam.collided = true;
    }
    cam.eyeAglM = eyeHeightM - groundM;
    // Planos como ATAK con la camara baja: near = 0,2 x AGL; far = horizonte fisico desde el ojo
    // (3570 x raiz(AGL), con margen) mas la distancia al foco.
    cam.nearM = std::max(0.05, cam.eyeAglM * 0.2);
    cam.farM = std::max(3570.0 * std::sqrt(std::max(cam.eyeAglM, 2.0)) * 1.5 + cam.rangeM, 500.0);
    return cam;
}

vec3 TransformState::getCameraForwardEcef() const {
    namespace globe = util::globe;
    namespace ecef = util::ecef;

    const LatLng cameraLatLng = getLatLng();

    // Base local Este-Norte-Arriba (ENU) en la posicion de la camara: "arriba" es el normal
    // elipsoidal real (WGS84), este/norte perpendiculares entre si y al eje polar. Misma idea que
    // el frame local que arma updateGlobeMatrices para orientar bearing/pitch sobre la esfera,
    // pero aca en ECEF real, con RTE (relative-to-eye) en vez de escalar por globeRadiusPixels.
    const vec3 up = ecef::surfaceNormal(cameraLatLng);
    const vec3 polarAxis = {0.0, 0.0, 1.0};
    const vec3 east = globe::normalize(globe::cross(polarAxis, up));
    const vec3 north = globe::normalize(globe::cross(up, east));

    // Direccion de vista segun bearing (0=norte, sentido horario, igual que getBearing()) y pitch
    // (0=nadir mirando derecho al piso, 90=horizonte, misma convencion que getPitch()).
    const double sinP = std::sin(pitch);
    const double cosP = std::cos(pitch);
    // `bearing` interno de MapLibre = -rumbo de brujula (Transform::easeTo: deg2rad(-camera.bearing)). Usarlo
    // directo hacia girar la camara ECEF al reves que la mercator: gestos y dibujo invertidos (field-test 30-09).
    const double compass = -bearing;
    const double sinB = std::sin(compass);
    const double cosB = std::cos(compass);
    vec3 forward = globe::add(
        globe::add(globe::scale(east, sinB * sinP), globe::scale(north, cosB * sinP)), globe::scale(up, -cosP));
    return globe::normalize(forward);
}

std::array<TransformState::EcefDrapeArea, TransformState::kEcefDrapeCount> TransformState::computeEcefDrapeAreas()
    const {
    namespace globe = util::globe;
    namespace ecef = util::ecef;

    // Camara orbital (ADR 0040): el punto de mira ES el foco, a la distancia de la camara (estimacion plana).
    const EcefCamera orbit = computeEcefCamera(0.0);
    const double sinPitch = util::clamp<double>(std::sin(pitch), 0.0, 1.0);
    const double latRad = util::deg2rad(getLatLng().latitude());
    const double slantToFocusM = orbit.rangeM;

    // Resolucion base: la del view en el punto de mira (drawMapResolution = scene.gsd en ATAK),
    // con piso en la resolucion nativa de la imagen (z19) -- pedir mas fino es overzoom -- y el
    // scaleAdj de ATAK para camara perspectiva (GLMapView2.cpp, rama activa).
    constexpr double kImageryNativeZoom = 19.0;
    const double viewGsdAtFocus =
        slantToFocusM * 2.0 * std::tan(kEcefFieldOfViewRad / 2.0) / static_cast<double>(size.height);
    const double nativeGsd = util::M2PI * util::EARTH_RADIUS_M * std::cos(latRad) /
                             (256.0 * std::pow(2.0, kImageryNativeZoom));
    const double scaleAdj = 1.0 + (sinPitch * 1.1);
    const double baseGsd = std::max(viewGsdAtFocus, nativeGsd) * scaleAdj;

    const vec3 focusEcef = orbit.focusEcef;

    std::array<EcefDrapeArea, kEcefDrapeCount> areas{};
    for (size_t i = 0; i < kEcefDrapeCount; ++i) {
        const double continuousRadiusM =
            0.5 * kEcefDrapeTextureSizesPx[i] * baseGsd * kEcefDrapeResolutionMultipliers[i];

        // Radio y centro cuantizados (escalones geometricos 1.4x, grilla radio/8): sin esto el
        // zoom y el area del covering sintetico cambian con cada variacion minima de pitch o GPS
        // y los tiles pedidos nunca terminan de llegar (confirmado en campo: z=15 y z=16
        // mezclados con el pitch casi constante).
        constexpr double kRadiusStepFactor = 1.4;
        const double radiusMeters = std::pow(
            kRadiusStepFactor, std::ceil(std::log(std::max(continuousRadiusM, 1.0)) / std::log(kRadiusStepFactor)));
        // Mas alla de un cuarto de meridiano (hemisferio a la vista) el drape es el planisferio entero: centro en el
        // ecuador y medio meridiano ecuatorial de radio, asi su recorte mercator es el mundo completo (lon +-180,
        // lat +-85). Antes se topaba en un cuarto de meridiano alrededor del foco y lo que quedaba afuera del
        // cuadrado salia estirado (field-test 01-10). Con radios mayores la grilla de cuantizacion (radio/8)
        // ademas mandaba el centro al centro de la Tierra (latitud NaN, render trabado).
        constexpr double kQuarterMeridianM = util::M2PI * util::EARTH_RADIUS_M / 4.0;
        if (radiusMeters > kQuarterMeridianM) {
            // Centro fijo en (0, 0): el planisferio es siempre el mismo mundo (copia 0) y su covering no cambia con
            // los gestos. Si siguiera al foco, los tiles de los bordes cambiaban de copia del mundo, se cancelaban y
            // se volvian a pedir sin llegar nunca (zonas blancas, field-test 01-10). El shader envuelve la u.
            areas[i] = {.center = LatLng(0.0, 0.0), .radiusMeters = 2.0 * kQuarterMeridianM, .global = true};
            continue;
        }
        const double gridStepM = radiusMeters / 8.0;
        const vec3 centerEcef = {std::round(focusEcef[0] / gridStepM) * gridStepM,
                                 std::round(focusEcef[1] / gridStepM) * gridStepM,
                                 std::round(focusEcef[2] / gridStepM) * gridStepM};
        areas[i] = {.center = ecef::ecefToLatLng(centerEcef), .radiusMeters = radiusMeters};
    }
    return areas;
}

TransformState TransformState::makeSyntheticDrapeState(const TransformState& base,
                                                       const EcefDrapeArea& area,
                                                       uint32_t textureSizePx) {
    // Copiar `base` y solo pisar posicion/zoom/pitch/bearing (dejando el resto heredado de la
    // camara real) fue la causa real del manchon en campo: getCameraToCenterDistance() depende
    // de `fov` (0.5*size.height/tan(fov/2)), y el fov de la camara ECEF real (con altura real
    // activa) no tiene por que coincidir con el fov "de mapa 2D estandar" que la formula de zoom
    // de abajo asume implicitamente (256px de tile a zoom 0 cubriendo toda la circunferencia,
    // convencion mercator pura). El resultado: el frustum de PERSPECTIVA real que arma el
    // tileCover normal (Frustum::fromInvProjMatrix) cubria un area geografica muy distinta al
    // radio pedido -- confirmado en campo, drape con datos solo en una franja angosta. Fiel al
    // patron de ATAK (createOffscreenSceneModel arma la escena offscreen desde cero con sus
    // propios parametros, no copia la camara real y pisa 2-3 campos): se resetean tambien fov,
    // roll, z (altura mercator de camara) y skew a un estado "limpio" y conocido, coherente con
    // la formula de zoom de mas abajo.
    TransformState synthetic = base;
    synthetic.setRealAltitudeMode(false);
    // Sin la restriccion mercator de llenar la pantalla: cerca de los polos movia el centro del covering lejos del
    // area del drape (ADR 0042).
    synthetic.setConstrainMode(ConstrainMode::None);
    synthetic.setBearing(0.0);
    synthetic.setPitch(0.0);
    synthetic.setRoll(0.0);
    synthetic.setFieldOfView(util::DEFAULT_FOV);
    synthetic.setZ(0.0);
    synthetic.setXSkew(0.0);
    synthetic.setYSkew(1.0);
    synthetic.setEdgeInsets(EdgeInsets());
    synthetic.setFrustumOffset(EdgeInsets());
    synthetic.setSize(Size{textureSizePx, textureSizePx});

    // metros/pixel deseados -> zoom mercator equivalente, con el tamano de tile del motor (util::tileSize_D = 512 px
    // en zoom 0) y el achicamiento por latitud del resto del motor mercator. Con 256 px el zoom salia uno mas alto y
    // el covering traia imagen solo para la mitad central del drape: en el globo, media Tierra sin imagen (field-test
    // 01-10); de cerca, el borde de cada drape caia al drape mas grueso.
    const double metersPerPixel = (2.0 * area.radiusMeters) / static_cast<double>(textureSizePx);
    const double metersPerPixelAtZoom0 = (util::M2PI * util::EARTH_RADIUS_M *
                                          std::cos(util::deg2rad(area.center.latitude()))) /
                                         util::tileSize_D;
    const double zoom = std::log2(std::max(metersPerPixelAtZoom0 / std::max(metersPerPixel, 1e-6), 1.0));

    synthetic.setLatLngZoom(area.center, util::clamp<double>(zoom, base.getMinZoom(), base.getMaxZoom()));
    return synthetic;
}

mat4 TransformState::getEcefTileMatrix(const vec3& originEcef, const EcefCamera& orbit) const {
    namespace globe = util::globe;
    namespace ecef = util::ecef;

    // Orientacion en la base ENU del foco (la misma de getCameraForwardEcef y de la camara mercator).
    const vec3 cameraOrigin = orbit.eyeEcef;
    const vec3 up = ecef::surfaceNormal(orbit.focus);
    const vec3 polarAxis = {0.0, 0.0, 1.0};
    const vec3 east = globe::normalize(globe::cross(polarAxis, up));
    const vec3 forward = orbit.forward;

    // Base ortonormal de camara (OpenGL: X=derecha, Y=arriba de pantalla, Z hacia el espectador). La derecha sale
    // del rumbo, como el azimut de la camara de ATAK -- no de forward x up, que al nadir (2D) se anula: ahi se caia en
    // una derecha fija al este y la vista 2D ignoraba el rumbo (arrastre girado, field-test 01-10).
    const vec3 north = globe::cross(up, east);
    const double compass = -bearing; // bearing interno de MapLibre = -rumbo de brujula
    const vec3 rightFromHeading =
        globe::subtract(globe::scale(east, std::cos(compass)), globe::scale(north, std::sin(compass)));
    const vec3 cameraUp = globe::normalize(globe::cross(rightFromHeading, forward));
    const vec3 cameraRight = globe::normalize(globe::cross(forward, cameraUp));

    // ECEF-relativo-a-camara -> espacio de camara. Column-major (glMatrix), fila0=cameraRight,
    // fila1=cameraUp, fila2=-forward.
    mat4 view = matrix::identity4();
    view[0] = cameraRight[0];
    view[1] = cameraUp[0];
    view[2] = -forward[0];
    view[4] = cameraRight[1];
    view[5] = cameraUp[1];
    view[6] = -forward[1];
    view[8] = cameraRight[2];
    view[9] = cameraUp[2];
    view[10] = -forward[2];

    // Todo lo anterior es orientacion pura (sin magnitud ECEF). La unica resta de vectores de
    // magnitud ECEF completa (~6.378.000 m) pasa aca, en double, antes de bajar a una traslacion
    // de a lo sumo unos cientos de km -> el resultado cabe en float32 sin jitter (RTE).
    const vec3 delta = globe::subtract(originEcef, cameraOrigin);
    mat4 translated;
    matrix::translate(translated, view, delta[0], delta[1], delta[2]);

    const double farZ = orbit.farM;
    const double nearZ = orbit.nearM;
    mat4 proj;
    matrix::perspective(proj, kEcefFieldOfViewRad, static_cast<double>(size.width) / size.height, nearZ, farZ);

    mat4 result;
    matrix::multiply(result, proj, translated);
    return result;
}

void TransformState::setProjection(const style::ProjectionDefinition& projection_) {
    if (projection != projection_) {
        projection = projection_;
        requestMatricesUpdate = true;
    }
}

double TransformState::getGlobeness() const {
    return projection.transitionState(getZoom());
}

bool TransformState::isGlobeRendering() const {
    return getGlobeness() > 0.0;
}

void TransformState::updateGlobeMatrices() const {
    namespace globe = util::globe;

    const double worldSize = Projection::worldSize(scale);
    const LatLng center = getLatLng(LatLng::Unwrapped);
    const double centerLatRad = util::deg2rad(center.latitude());
    const double centerLngRad = util::deg2rad(center.longitude());
    const double cameraToCenterDistance = getCameraToCenterDistance();

    globeRadiusPixels = globe::getGlobeRadiusPixels(worldSize, center.latitude());

    const double nearZ = 0.5;
    const double farZ = cameraToCenterDistance + globeRadiusPixels * 2.0;

    matrix::perspective(globeMatrix, fov, static_cast<double>(size.width) / size.height, nearZ, farZ);

    const ScreenCoordinate offset = getCenterOffset();
    globeMatrix[8] = -offset.x * 2.0 / size.width;
    globeMatrix[9] = offset.y * 2.0 / size.height;

    if (getNorthOrientation() != NorthOrientation::Upwards) {
        matrix::rotate_z(globeMatrix, globeMatrix, -getNorthOrientationAngle());
    }

    matrix::translate(globeMatrix, globeMatrix, 0.0, 0.0, -cameraToCenterDistance);
    matrix::rotate_z(globeMatrix, globeMatrix, -roll);
    matrix::rotate_x(globeMatrix, globeMatrix, -pitch);
    matrix::rotate_z(globeMatrix, globeMatrix, -bearing);
    matrix::translate(globeMatrix, globeMatrix, 0.0, 0.0, -globeRadiusPixels);
    matrix::rotate_x(globeMatrix, globeMatrix, centerLatRad);
    matrix::rotate_y(globeMatrix, globeMatrix, -centerLngRad);
    matrix::scale(globeMatrix, globeMatrix, globeRadiusPixels, globeRadiusPixels, globeRadiusPixels);

    if (matrix::invert(invGlobeMatrix, globeMatrix)) {
        invGlobeMatrix = matrix::identity4();
    }

    vec3 cameraPosition = {0.0, 0.0, cameraToCenterDistance / globeRadiusPixels};
    cameraPosition = globe::rotateZ(cameraPosition, roll);
    cameraPosition = globe::rotateX(cameraPosition, pitch);
    cameraPosition = globe::rotateZ(cameraPosition, bearing);
    cameraPosition = globe::add(cameraPosition, vec3{0.0, 0.0, 1.0});
    cameraPosition = globe::rotateX(cameraPosition, -centerLatRad);
    cameraPosition = globe::rotateY(cameraPosition, centerLngRad);
    globeCameraPosition = cameraPosition;

    const double distanceCameraToB = cameraToCenterDistance / globeRadiusPixels;
    const double distanceCameraToA = std::sin(pitch) * distanceCameraToB;
    const double distanceAtoC = std::cos(pitch) * distanceCameraToB + 1.0;
    const double distanceCameraToC = std::sqrt(distanceCameraToA * distanceCameraToA + distanceAtoC * distanceAtoC);
    const double tangentPlaneDistanceToC = 1.0 / distanceCameraToC;

    double vectorCtoCamX = -distanceCameraToA;
    double vectorCtoCamY = distanceAtoC;
    const double vectorCtoCamLength = std::sqrt(vectorCtoCamX * vectorCtoCamX + vectorCtoCamY * vectorCtoCamY);
    vectorCtoCamX /= vectorCtoCamLength;
    vectorCtoCamY /= vectorCtoCamLength;

    vec3 planeVector = {0.0, vectorCtoCamX, vectorCtoCamY};
    planeVector = globe::rotateZ(planeVector, bearing);
    planeVector = globe::rotateX(planeVector, -centerLatRad);
    planeVector = globe::rotateY(planeVector, centerLngRad);
    const double planeScale = 1.0 / globe::length(planeVector);
    planeVector = globe::scale(planeVector, planeScale);
    globeClippingPlane = {planeVector[0], planeVector[1], planeVector[2], -tangentPlaneDistanceToC * planeScale};
}

const mat4& TransformState::getGlobeMatrix() const {
    updateMatricesIfNeeded();
    return globeMatrix;
}

const mat4& TransformState::getInvGlobeMatrix() const {
    updateMatricesIfNeeded();
    return invGlobeMatrix;
}

const vec4& TransformState::getGlobeClippingPlane() const {
    updateMatricesIfNeeded();
    return globeClippingPlane;
}

const vec3& TransformState::getGlobeCameraPosition() const {
    updateMatricesIfNeeded();
    return globeCameraPosition;
}

double TransformState::getGlobeRadiusPixels() const {
    updateMatricesIfNeeded();
    return globeRadiusPixels;
}

vec4 TransformState::getTileMercatorCoords(const UnwrappedTileID& tileID) {
    const double tileScale = static_cast<double>(1ull << tileID.canonical.z);
    return {tileID.canonical.x / tileScale,
            tileID.canonical.y / tileScale,
            1.0 / tileScale / util::EXTENT,
            1.0 / tileScale / util::EXTENT};
}

ScreenCoordinate TransformState::latLngToScreenCoordinateGlobe(const LatLng& latLng, bool& occluded) const {
    namespace globe = util::globe;

    const vec3 spherePos = globe::latLngToSurfaceVector(latLng);
    const vec4& plane = getGlobeClippingPlane();
    occluded = globe::pointPlaneSignedDistance(plane, spherePos) < 0.0;

    vec4 projected;
    const vec4 input = {spherePos[0], spherePos[1], spherePos[2], 1.0};
    matrix::transformMat4(projected, input, getGlobeMatrix());
    if (projected[3] == 0.0) {
        return {};
    }
    const double ndcX = projected[0] / projected[3];
    const double ndcY = projected[1] / projected[3];
    return {(ndcX * 0.5 + 0.5) * size.width, (1.0 - (ndcY * 0.5 + 0.5)) * size.height};
}

vec3 TransformState::getRayDirectionFromPixel(const ScreenCoordinate& point) const {
    namespace globe = util::globe;

    const double ndcX = point.x / size.width * 2.0 - 1.0;
    const double ndcY = 1.0 - point.y / size.height * 2.0;

    vec4 nearPoint;
    vec4 farPoint;
    matrix::transformMat4(nearPoint, vec4{ndcX, ndcY, -1.0, 1.0}, getInvGlobeMatrix());
    matrix::transformMat4(farPoint, vec4{ndcX, ndcY, 1.0, 1.0}, getInvGlobeMatrix());

    if (nearPoint[3] == 0.0 || farPoint[3] == 0.0) {
        return {0.0, 0.0, 0.0};
    }

    const vec3 a = {nearPoint[0] / nearPoint[3], nearPoint[1] / nearPoint[3], nearPoint[2] / nearPoint[3]};
    const vec3 b = {farPoint[0] / farPoint[3], farPoint[1] / farPoint[3], farPoint[2] / farPoint[3]};
    return globe::normalize(globe::subtract(b, a));
}

std::optional<LatLng> TransformState::screenCoordinateToLatLngGlobe(const ScreenCoordinate& point) const {
    namespace globe = util::globe;

    const vec3 origin = getGlobeCameraPosition();
    const vec3 direction = getRayDirectionFromPixel(point);
    if (globe::length(direction) == 0.0) {
        return std::nullopt;
    }

    double t = 0.0;
    if (!globe::raySphereIntersection(origin, direction, 1.0, t)) {
        return std::nullopt;
    }

    const vec3 hit = globe::add(origin, globe::scale(direction, t));
    return globe::surfaceVectorToLatLng(globe::normalize(hit));
}

void TransformState::updateMatricesIfNeeded() const {
    if (!needsMatricesUpdate() || size.isEmpty()) return;

    getProjMatrix(projectionMatrix);
    coordMatrix = coordinatePointMatrix(projectionMatrix);

    bool err = matrix::invert(invProjectionMatrix, projectionMatrix);
    if (err) throw std::runtime_error("failed to invert projectionMatrix");

    err = matrix::invert(invertedMatrix, coordMatrix);
    if (err) throw std::runtime_error("failed to invert coordinatePointMatrix");

    updateGlobeMatrices();

    requestMatricesUpdate = false;
}

const mat4& TransformState::getProjectionMatrix() const {
    updateMatricesIfNeeded();
    return projectionMatrix;
}

const mat4& TransformState::getInvProjectionMatrix() const {
    updateMatricesIfNeeded();
    return invProjectionMatrix;
}

const mat4& TransformState::getCoordMatrix() const {
    updateMatricesIfNeeded();
    return coordMatrix;
}

const mat4& TransformState::getInvertedMatrix() const {
    updateMatricesIfNeeded();
    return invertedMatrix;
}

// MARK: - Dimensions

Size TransformState::getSize() const {
    return size;
}

void TransformState::setSize(const Size& size_) {
    if (size != size_) {
        size = size_;
        requestMatricesUpdate = true;
    }
}

EdgeInsets TransformState::getFrustumOffset() const {
    return frustumOffset;
}

void TransformState::setFrustumOffset(const EdgeInsets& frustumOffset_) {
    if (frustumOffset != frustumOffset_) {
        frustumOffset = frustumOffset_;
        requestMatricesUpdate = true;
    }
}

// MARK: - North Orientation

NorthOrientation TransformState::getNorthOrientation() const {
    return orientation;
}

void TransformState::setNorthOrientation(const NorthOrientation val) {
    if (orientation != val) {
        orientation = val;
        requestMatricesUpdate = true;
    }
}

double TransformState::getNorthOrientationAngle() const {
    double angleOrientation = 0;
    if (orientation == NorthOrientation::Rightwards) {
        angleOrientation += pi / 2;
    } else if (orientation == NorthOrientation::Downwards) {
        angleOrientation += pi;
    } else if (orientation == NorthOrientation::Leftwards) {
        angleOrientation -= pi / 2;
    }
    return angleOrientation;
}

// MARK: - Constrain mode

ConstrainMode TransformState::getConstrainMode() const {
    return constrainMode;
}

void TransformState::setConstrainMode(const ConstrainMode val) {
    if (constrainMode != val) {
        constrainMode = val;
        requestMatricesUpdate = true;
    }
}

// MARK: - ViewportMode

ViewportMode TransformState::getViewportMode() const {
    return viewportMode;
}

void TransformState::setViewportMode(ViewportMode val) {
    if (viewportMode != val) {
        viewportMode = val;
        requestMatricesUpdate = true;
    }
}

// MARK: - Camera options

CameraOptions TransformState::getCameraOptions(const std::optional<EdgeInsets>& padding) const {
    return CameraOptions()
        .withCenter(getLatLng())
        .withCenterAltitude(getCenterAltitude())
        .withPadding(padding ? padding : edgeInsets)
        .withZoom(getZoom())
        .withBearing(util::rad2deg(-bearing))
        .withPitch(util::rad2deg(pitch))
        .withRoll(util::rad2deg(roll))
        .withFov(util::rad2deg(fov));
}

// MARK: - EdgeInsets

void TransformState::setEdgeInsets(const EdgeInsets& val) {
    if (edgeInsets != val) {
        edgeInsets = val;
        requestMatricesUpdate = true;
    }
}

// MARK: - Position

LatLng TransformState::getLatLng(LatLng::WrapMode wrapMode) const {
    return {util::rad2deg(2 * std::atan(std::exp(y / Cc)) - 0.5 * pi), -x / Bc, wrapMode};
}

double TransformState::getCenterAltitude() const {
    return z * Projection::getMetersPerPixelAtLatitude(getLatLng().latitude(), getZoom());
}

double TransformState::pixel_x() const {
    const double center = (size.width - Projection::worldSize(scale)) / 2;
    return center + x;
}

double TransformState::pixel_y() const {
    const double center = (size.height - Projection::worldSize(scale)) / 2;
    return center + y;
}

// MARK: - Zoom

double TransformState::getZoom() const {
    return scaleZoom(scale);
}

uint8_t TransformState::getIntegerZoom() const {
    return static_cast<uint8_t>(getZoom());
}

double TransformState::getZoomFraction() const {
    return getZoom() - getIntegerZoom();
}

// MARK: - Bounds

void TransformState::setLatLngBounds(LatLngBounds bounds_) {
    if (bounds_ != bounds) {
        bounds = bounds_;
        setLatLngZoom(getLatLng(LatLng::Unwrapped), getZoom());
    }
}

LatLngBounds TransformState::getLatLngBounds() const {
    return bounds;
}

void TransformState::setMinZoom(const double minZoom) {
    if (minZoom <= getMaxZoom()) {
        min_scale = zoomScale(util::clamp(minZoom, util::MIN_ZOOM, util::MAX_ZOOM));
    }
}

double TransformState::getMinZoom() const {
    double test_scale = min_scale;
    double unused_x = x;
    double unused_y = y;
    constrain(test_scale, unused_x, unused_y);

    return scaleZoom(test_scale);
}

void TransformState::setMaxZoom(const double maxZoom) {
    if (maxZoom >= getMinZoom()) {
        max_scale = zoomScale(util::clamp(maxZoom, util::MIN_ZOOM, util::MAX_ZOOM));
    }
}

double TransformState::getMaxZoom() const {
    return scaleZoom(max_scale);
}

void TransformState::setMinPitch(const double pitch_) {
    if (pitch_ <= maxPitch) {
        minPitch = util::clamp(pitch_, util::PITCH_MIN, maxPitch);
    } else {
        Log::Warning(Event::General,
                     "Trying to set minimum pitch to larger than maximum pitch, no "
                     "changes made.");
    }
}

double TransformState::getMinPitch() const {
    return minPitch;
}

void TransformState::setMaxPitch(const double pitch_) {
    if (pitch_ >= minPitch) {
        maxPitch = util::clamp(pitch_, minPitch, util::PITCH_MAX);
    } else {
        Log::Warning(Event::General,
                     "Trying to set maximum pitch to smaller than minimum pitch, no "
                     "changes made.");
    }
}

double TransformState::getMaxPitch() const {
    return maxPitch;
}

double TransformState::getMinFieldOfView() const {
    return minFov;
}

double TransformState::getMaxFieldOfView() const {
    return maxFov;
}

// MARK: - Scale
double TransformState::getScale() const {
    return scale;
}

void TransformState::setScale(double val) {
    if (scale != val) {
        scale = val;
        requestMatricesUpdate = true;
    }
}

// MARK: - Positions

double TransformState::getX() const {
    return x;
}

void TransformState::setX(double val) {
    if (x != val) {
        x = val;
        requestMatricesUpdate = true;
    }
}

double TransformState::getY() const {
    return y;
}

void TransformState::setY(double val) {
    if (y != val) {
        y = val;
        requestMatricesUpdate = true;
    }
}

double TransformState::getZ() const {
    return z;
}

void TransformState::setZ(double val) {
    if (z != val) {
        z = val;
        requestMatricesUpdate = true;
    }
}

// MARK: - Rotation

double TransformState::getBearing() const {
    return bearing;
}

void TransformState::setBearing(double val) {
    if (bearing != val) {
        bearing = val;
        requestMatricesUpdate = true;
    }
}

float TransformState::getFieldOfView() const {
    return static_cast<float>(fov);
}

void TransformState::setFieldOfView(double val) {
    if (fov != val) {
        fov = val;
        requestMatricesUpdate = true;
    }
}

double TransformState::getRoll() const {
    return roll;
}

void TransformState::setRoll(double val) {
    if (roll != val) {
        roll = val;
        requestMatricesUpdate = true;
    }
}

float TransformState::getCameraToCenterDistance() const {
    return static_cast<float>(0.5 * size.height / std::tan(fov / 2.0));
}

double TransformState::getPitch() const {
    return pitch;
}

void TransformState::setPitch(double val) {
    if (pitch != val) {
        pitch = val;
        requestMatricesUpdate = true;
    }
}

double TransformState::getXSkew() const {
    return xSkew;
}

void TransformState::setXSkew(double val) {
    if (xSkew != val) {
        xSkew = val;
        requestMatricesUpdate = true;
    }
}
double TransformState::getYSkew() const {
    return ySkew;
}

void TransformState::setYSkew(double val) {
    if (ySkew != val) {
        ySkew = val;
        requestMatricesUpdate = true;
    }
}

bool TransformState::getAxonometric() const {
    return axonometric;
}

void TransformState::setAxonometric(bool val) {
    if (axonometric != val) {
        axonometric = val;
        requestMatricesUpdate = true;
    }
}

// MARK: - State

bool TransformState::isChanging() const {
    return rotating || scaling || panning || gestureInProgress;
}

bool TransformState::isRotating() const {
    return rotating;
}

bool TransformState::isScaling() const {
    return scaling;
}

bool TransformState::isPanning() const {
    return panning;
}

bool TransformState::isGestureInProgress() const {
    return gestureInProgress;
}

// MARK: - Projection

double TransformState::zoomScale(double zoom) const {
    return roundForAccuracy(std::pow(2.0, zoom));
}

double TransformState::scaleZoom(double s) const {
    return roundForAccuracy(util::log2(s));
}

ScreenCoordinate TransformState::latLngToScreenCoordinate(const LatLng& latLng) const {
    if (isGlobeRendering() && !size.isEmpty()) {
        bool occluded = false;
        return latLngToScreenCoordinateGlobe(latLng, occluded);
    }
    // Camara con altura real (ADR 0043 -- regla de oro), simetrico de screenCoordinateToLatLng: sin esto,
    // CUALQUIER punto reproyectado con la camara real activa caia por el camino mercator plano de abajo,
    // que no corresponde a lo que se ve en pantalla (igual causa que el bug original del picking, pero en
    // la direccion inversa).
    if (isRealAltitudeModeEnabled() && !size.isEmpty()) {
        return latLngToScreenCoordinateEcef(latLng);
    }
    vec4 p;
    return latLngToScreenCoordinate(latLng, p);
}

ScreenCoordinate TransformState::latLngToScreenCoordinate(const LatLng& latLng, vec4& p) const {
    if (size.isEmpty()) {
        return {};
    }

    Point<double> pt = Projection::project(latLng, scale) / util::tileSize_D;
    vec4 c = {{pt.x, pt.y, 0, 1}};
    matrix::transformMat4(p, c, getCoordMatrix());
    return {p[0] / p[3], size.height - p[1] / p[3]};
}

TileCoordinate TransformState::screenCoordinateToTileCoordinate(const ScreenCoordinate& point, uint8_t atZoom) const {
    if (size.isEmpty()) {
        return {.p = {}, .z = 0};
    }

    float targetZ = 0;

    double flippedY = size.height - point.y;

    // since we don't know the correct projected z value for the point,
    // unproject two points to get a line and then find the point on that
    // line with z=0

    vec4 coord0;
    vec4 coord1;
    vec4 point0 = {{point.x, flippedY, 0, 1}};
    vec4 point1 = {{point.x, flippedY, 1, 1}};
    matrix::transformMat4(coord0, point0, getInvertedMatrix());
    matrix::transformMat4(coord1, point1, getInvertedMatrix());

    double w0 = coord0[3];
    double w1 = coord1[3];

    Point<double> p0 = Point<double>(coord0[0], coord0[1]) / w0;
    Point<double> p1 = Point<double>(coord1[0], coord1[1]) / w1;

    double z0 = coord0[2] / w0;
    double z1 = coord1[2] / w1;
    double t = z0 <= z1 ? 0 : (targetZ - z0) / (z1 - z0);

    Point<double> p = util::interpolate(p0, p1, t) / scale * static_cast<double>(1 << atZoom);
    return {.p = {p.x, p.y}, .z = static_cast<double>(atZoom)};
}

LatLng TransformState::screenCoordinateToLatLng(const ScreenCoordinate& point, LatLng::WrapMode wrapMode) const {
    if (isGlobeRendering() && !size.isEmpty()) {
        if (const auto globeLatLng = screenCoordinateToLatLngGlobe(point)) {
            return {globeLatLng->latitude(), globeLatLng->longitude(), wrapMode};
        }
    }
    // Camara con altura real (ADR 0043 -- regla de oro): el camino de abajo (proyeccion mercator plana,
    // pensada para el mapa 2D clasico) IGNORA la camara orbital ECEF real -- son dos camaras distintas que
    // divergen fuerte cerca de los polos y con la camara muy alejada. Los gestos de un dedo (panTo de ATAK,
    // AtakNavigation.kt) rayaban un punto con la camara equivocada: la pantalla mostraba otra cosa que la
    // que el pan movia (field-test 04-10: "con un dedo hace zoom", "se queda pegado en el polo").
    if (isRealAltitudeModeEnabled() && !size.isEmpty()) {
        const LatLng ecef = screenCoordinateToLatLngEcef(point);
        return {ecef.latitude(), ecef.longitude(), wrapMode};
    }
    auto coord = screenCoordinateToTileCoordinate(point, 0);
    return Projection::unproject(coord.p, 1. / util::tileSize_D, wrapMode);
}

LatLng TransformState::screenCoordinateToLatLngEcef(const ScreenCoordinate& point) const {
    namespace globe = util::globe;
    namespace ecef = util::ecef;

    // Reconstruccion a mano (base de camara + offset por tangente de FOV) tenia un giro geometrico que se
    // acentuaba con la distancia camara-foco: la base sale del plano tangente en el FOCO (surfaceNormal del
    // foco), y para un ojo MUY lejos del foco (zoom alejado) eso ya no corresponde al "derecha de pantalla"
    // real -- el picking se invertia pasado cierto zoom (field-test 04-10: "a 1 km bien, mas lejos al reves,
    // como si hubiera dos niveles"). Confirmado con una sonda: la relacion este/oeste de un punto lateral
    // cruza signo en forma continua entre zoom 14 y 15, sin ningun salto de rama.
    //
    // Arreglo de raiz, sin reconstruir nada a mano: se arma la MISMA matriz view*proj que usa
    // getEcefTileMatrix para dibujar (con origen en el propio ojo, sin el corrimiento RTE por sub-tile, que
    // no hace falta aca), se invierte, y se desproyectan dos puntos NDC (cerca/lejos) para sacar el rayo --
    // igual tecnica que el picking del globo nativo de MapLibre, getRayDirectionFromPixel/invGlobeMatrix.
    const EcefCamera orbit = computeEcefCamera(0.0);
    const mat4 viewProj = getEcefTileMatrix(orbit.eyeEcef, orbit); // origen = ojo -> sin traslacion RTE

    // matrix::invert devuelve true SOLO en el caso degenerado (determinante 0, out sin tocar) y false cuando
    // SI invirtio -- al reves de lo que parece a primera vista (ver el mismo patron en getInvGlobeMatrix). Un
    // "if (!invert(...))" aca hacia que CADA llamada cayera al foco sin importar el pixel (massivo bug
    // encontrado mientras se perseguia, erroneamente, un problema de signo -- field-test 04-10).
    mat4 invViewProj;
    if (matrix::invert(invViewProj, viewProj)) {
        return orbit.focus; // matriz degenerada (tamano de pantalla nulo u otro borde): sin rayo, usar el foco
    }

    // El signo de ndcY esta invertido a proposito respecto de la convencion "de libro" (la misma que usa
    // getRayDirectionFromPixel/el picking del globo nativo); ndcX NO -- verificado con una sonda que compara,
    // punto a punto y en 11 niveles de zoom, contra el camino mercator plano ya probado en campo: arriba de
    // pantalla tiene que dar SUR, no norte (field-test 04-10, "norte y sur esta invertido, este/oeste bien").
    // Antes se habia invertido ndcX en cambio (un fix de una ronda previa, cuando TODAVIA estaban presentes el
    // bug de matrix::invert y el de la raiz negativa de mas abajo): compensaba esos bugs, no un signo real --
    // arreglados esos, la combinacion correcta es esta, re-derivada de cero con la sonda, no supuesta. No se
    // identifico la causa exacta dentro de la base de camara de getEcefTileMatrix que produce este signo (ver
    // ADR 0043 §6.5, la formula del OJO si coincide exacto con ATAK; esto es la proyeccion de un PIXEL a
    // traves de esa misma base, una cuenta distinta) -- verificado empiricamente, no derivado a mano. Si se
    // vuelve a tocar esta funcion, repetir la sonda (no asumir un signo por una ronda de fixes anterior).
    const double ndcX = (2.0 * point.x / size.width) - 1.0;
    const double ndcY = (2.0 * point.y / size.height) - 1.0;
    vec4 nearPoint;
    vec4 farPoint;
    matrix::transformMat4(nearPoint, vec4{ndcX, ndcY, -1.0, 1.0}, invViewProj);
    matrix::transformMat4(farPoint, vec4{ndcX, ndcY, 1.0, 1.0}, invViewProj);
    if (nearPoint[3] == 0.0 || farPoint[3] == 0.0) {
        return orbit.focus;
    }
    // Las dos desproyecciones estan relativas al ojo (sin traslacion en la matriz): la resta ya da la
    // direccion del rayo, sin necesidad de sumar orbit.eyeEcef para el origen.
    const vec3 a = {nearPoint[0] / nearPoint[3], nearPoint[1] / nearPoint[3], nearPoint[2] / nearPoint[3]};
    const vec3 b = {farPoint[0] / farPoint[3], farPoint[1] / farPoint[3], farPoint[2] / farPoint[3]};
    const vec3 dir = globe::normalize(globe::subtract(b, a));

    // Interseccion contra el ELIPSOIDE WGS84 real (no una esfera): la misma geometria que usa ATAK para
    // este picking (MapProjectionDisplayModel::earth = Ellipsoid2(semiMajor, semiMajor, semiMinor),
    // MapSceneModel2.cpp; CameraController_createFocusAltitudeModel infla ese elipsoide por la altitud del
    // foco para el modelo contra el que raycastea panTo/panBy -- solo lectura GPLv3, no copiado). Tecnica
    // estandar: escalar el rayo al espacio de la esfera unitaria por los 3 semiejes, resolver ahi, escalar
    // el resultado de vuelta. El achatamiento WGS84 es ~0,3% (semieje menor ~21 km mas corto) -- demasiado
    // chico para explicar los bugs de campo de esta sesion, pero es el modelo exacto de la referencia.
    const vec3 axes = {util::ecef::WGS84_SEMI_MAJOR_M, util::ecef::WGS84_SEMI_MAJOR_M, util::ecef::WGS84_SEMI_MINOR_M};
    const vec3 originScaled = {orbit.eyeEcef[0] / axes[0], orbit.eyeEcef[1] / axes[1], orbit.eyeEcef[2] / axes[2]};
    const vec3 dirScaled = {dir[0] / axes[0], dir[1] / axes[1], dir[2] / axes[2]}; // OJO: no es unitario

    // |origen' + t*dir'| = 1 (esfera unitaria): A*t^2 + 2*B*t + C = 0, con A=|dir'|^2 (no 1, dir' no es
    // unitario tras escalar por semiejes distintos).
    const double A = globe::dot(dirScaled, dirScaled);
    const double B = globe::dot(originScaled, dirScaled);
    const double C = globe::dot(originScaled, originScaled) - 1.0;
    const double disc = B * B - A * C;

    vec3 hit;
    bool useFallback = disc < 0.0;
    if (!useFallback) {
        const double sq = std::sqrt(disc);
        const double t0 = (-B - sq) / A;
        const double t1 = (-B + sq) / A;
        // La raiz matematicamente mas chica puede caer detras de la camara (t<0): el ojo esta siempre
        // afuera del elipsoide, asi que por Vieta las dos raices tienen el mismo signo -- si la mas chica da
        // negativa, tomar la otra; si ESA tambien es negativa, el rayo (la mitad de la recta hacia adelante)
        // no toca el elipsoide aunque la recta completa si (field-test 04-10: un "hit" detras de la camara
        // tomado como valido mandaba el punto al lado equivocado del horizonte).
        const double t = (t0 > 0.0) ? t0 : t1;
        if (t <= 0.0) {
            useFallback = true;
        } else {
            hit = globe::add(orbit.eyeEcef, globe::scale(dir, t));
            // Cara de atras del elipsoide (el rayo raspa el horizonte del lado cercano sin tocarlo y entra
            // por el opuesto, con el globo entero a la vista): la normal en el punto tiene que mirar hacia
            // la camara. La normal real del elipsoide en un punto P es P/ejes^2, pero el SIGNO de
            // dot(normal,dir) es el mismo que dot(P,dir) (ejes^2 > 0 siempre), asi que alcanza con P.
            if (globe::dot(hit, dir) >= 0.0) {
                useFallback = true;
            }
        }
    }
    if (useFallback) {
        // Punto del elipsoide mas cercano al rayo, en la metrica escalada (limite del horizonte, continuo
        // con el caso de arriba justo en la tangente): punto mas cercano de la RECTA escalada al centro
        // (formula general, dirScaled no es unitario), proyectado a la esfera unitaria y vuelto a escalar.
        const double tClosest = A != 0.0 ? -B / A : 0.0;
        const vec3 closestScaled = globe::add(originScaled, globe::scale(dirScaled, tClosest));
        const vec3 onUnitSphere = globe::normalize(closestScaled);
        hit = {onUnitSphere[0] * axes[0], onUnitSphere[1] * axes[1], onUnitSphere[2] * axes[2]};
    }
    return ecef::ecefToLatLng(hit);
}

ScreenCoordinate TransformState::latLngToScreenCoordinateEcef(const LatLng& latLng) const {
    namespace ecef = util::ecef;
    namespace globe = util::globe;

    const EcefCamera orbit = computeEcefCamera(0.0);
    const mat4 viewProj = getEcefTileMatrix(orbit.eyeEcef, orbit); // origen = ojo, igual que en la inversa

    const vec3 posEcef = ecef::llaToEcef(latLng, 0.0);
    // La matriz espera posiciones relativas al ojo (sin traslacion RTE) -- misma convencion que la inversa.
    const vec3 rel = globe::subtract(posEcef, orbit.eyeEcef);
    vec4 clip;
    matrix::transformMat4(clip, vec4{rel[0], rel[1], rel[2], 1.0}, viewProj);
    if (clip[3] == 0.0) {
        return {};
    }
    const double ndcX = clip[0] / clip[3];
    const double ndcY = clip[1] / clip[3];
    // Inversa exacta de las formulas de NDC de screenCoordinateToLatLngEcef (ndcX natural, ndcY ya en la
    // convencion de esta base de camara, ver el comentario alli -- no hace falta volver a derivar el signo,
    // es la misma cuenta despejada al reves).
    return {(ndcX + 1.0) * 0.5 * size.width, (ndcY + 1.0) * 0.5 * size.height};
}

mat4 TransformState::coordinatePointMatrix(const mat4& projMatrix) const {
    mat4 proj = projMatrix;
    matrix::scale(proj, proj, util::tileSize_D, util::tileSize_D, 1);
    matrix::multiply(proj, getPixelMatrix(), proj);
    return proj;
}

mat4 TransformState::getPixelMatrix() const {
    mat4 m;
    matrix::identity(m);
    matrix::scale(m, m, static_cast<double>(size.width) / 2, -static_cast<double>(size.height) / 2, 1);
    matrix::translate(m, m, 1, -1, 0);
    return m;
}

// MARK: - (private helper functions)

bool TransformState::rotatedNorth() const {
    using NO = NorthOrientation;
    return (orientation == NO::Leftwards || orientation == NO::Rightwards);
}

bool TransformState::constrainScreen(double& scale_, double& lat, double& lon) const {
    if (constrainMode == ConstrainMode::Screen) {
        double zoom = scaleZoom(scale_);
        CameraOptions options = CameraOptions();
        constrainCameraAndZoomToBounds(options, zoom);

        scale_ = zoomScale(zoom);

        if (options.center) {
            LatLng center = options.center.value();
            lat = center.latitude();
            lon = center.longitude();

            return true;
        }
    }
    return false;
}

void TransformState::constrain(double& scale_, double& x_, double& y_) const {
    if (constrainMode == ConstrainMode::None || constrainMode == ConstrainMode::Screen) {
        return;
    }

    // Camara orbital (ADR 0040, 0042): el globo no tiene que llenar la pantalla como el mapa mercator. Exigirlo
    // trababa el foco cerca del ecuador con zoom bajo (no se llegaba al hemisferio norte, field-test 01-10) e
    // imponia un zoom minimo. Solo se mantiene el foco dentro del mundo mercator (lat +-85).
    if (realAltitudeEnabled) {
        const double maxY = scale_ * util::tileSize_D / 2.0;
        y_ = std::clamp(y_, -maxY, maxY);
        return;
    }

    // Constrain scale to avoid zooming out far enough to show off-world areas on the Y axis.
    const double ratioY = (rotatedNorth() ? size.width : size.height) / util::tileSize_D;
    scale_ = util::max(scale_, ratioY);

    // Constrain min/max pan to avoid showing off-world areas on the Y axis.
    double max_y = (scale_ * util::tileSize_D - (rotatedNorth() ? size.width : size.height)) / 2;
    y_ = std::max(-max_y, std::min(y_, max_y));

    if (constrainMode == ConstrainMode::WidthAndHeight) {
        // Constrain min/max pan to avoid showing off-world areas on the X axis.
        double max_x = (scale_ * util::tileSize_D - (rotatedNorth() ? size.height : size.width)) / 2;
        x_ = std::max(-max_x, std::min(x_, max_x));
    }
}

void TransformState::constrainCameraAndZoomToBounds(CameraOptions& requestedCamera, double& requestedZoom) const {
    if (constrainMode != ConstrainMode::Screen || getLatLngBounds() == LatLngBounds()) {
        return;
    }

    LatLng centerLatLng = getLatLng();

    if (requestedCamera.center) {
        centerLatLng = requestedCamera.center.value();
    }

    Point<double> anchorOffset{0, 0};
    double requestedScale = zoomScale(requestedZoom);

    // Since the transition calculations will include any specified anchor in the result
    // we need to do the same when testing if the requested center and zoom is outside the bounds or not.
    if (requestedCamera.anchor) {
        ScreenCoordinate anchor = requestedCamera.anchor.value();
        anchor.y = getSize().height - anchor.y;
        LatLng anchorLatLng = screenCoordinateToLatLng(anchor);

        // The screenCoordinateToLatLng function requires the matrices inside the state to reflect
        // the requested scale. So we create a copy and set the requested zoom before the conversion.
        // This will give us the same result as the transition calculations.
        TransformState state{*this};
        state.setLatLngZoom(getLatLng(), scaleZoom(requestedScale));
        LatLng screenLatLng = state.screenCoordinateToLatLng(anchor);

        auto latLngCoord = Projection::project(anchorLatLng, requestedScale);
        auto anchorCoord = Projection::project(screenLatLng, requestedScale);
        anchorOffset = latLngCoord - anchorCoord;
    }

    mln::LatLngBounds currentBounds = getLatLngBounds();
    mln::ScreenCoordinate neBounds = Projection::project(currentBounds.northeast(), requestedScale);
    mln::ScreenCoordinate swBounds = Projection::project(currentBounds.southwest(), requestedScale);
    mln::ScreenCoordinate center = Projection::project(centerLatLng, requestedScale);
    mln::ScreenCoordinate currentCenter = Projection::project(getLatLng(), requestedScale);

    double minY = neBounds.y;
    double maxY = swBounds.y;
    double minX = swBounds.x;
    double maxX = neBounds.x;

    double startX = center.x;
    double startY = center.y;

    double resultX = startX;
    double resultY = startY;

    uint32_t screenWidth = getSize().width;
    uint32_t screenHeight = getSize().height;

    double h2 = screenHeight / 2.0;
    if (startY - h2 + anchorOffset.y < minY) {
        resultY = minY + h2;
    }
    if (startY + anchorOffset.y + h2 > maxY) {
        resultY = maxY - h2;
    }

    double w2 = screenWidth / 2.0;
    if (startX + anchorOffset.x - w2 < minX) {
        resultX = minX + w2;
    }
    if (startX + anchorOffset.x + w2 > maxX) {
        resultX = maxX - w2;
    }

    double scaleY = 0;
    if (maxY - minY < screenHeight) {
        scaleY = screenHeight / (maxY - minY);
        resultY = (maxY + minY) / 2.0;
    }

    double scaleX = 0;
    if (maxX - minX < screenWidth) {
        scaleX = screenWidth / (maxX - minX);
        resultX = (maxX + minX) / 2.0;
    }

    double maxScale = scaleX > scaleY ? scaleX : scaleY;

    // Max scale will be 1 when the screen is exactly the same size as the max bounds in either the X or Y direction.
    // To avoid numerical instabilities we add small amount to the check to make sure we don't try to scale when we
    // don't actually need it.
    if (maxScale > 1.000001) {
        requestedZoom += scaleZoom(maxScale);

        if (scaleY > scaleX) {
            // If we scaled the y direction we want the resulting x position to be the same as the current x position.
            resultX = currentCenter.x;
        } else {
            // If we scaled the x direction we want the resulting y position to be the same as the current y position.
            resultY = currentCenter.y;
        }

        // Since we changed the scale, we might display something outside the bounds.
        // When checking we need to take into consideration that we just changed the scale,
        // since the resultX and minX were calculated with the requested scale, and not the scale we
        // just calculated to make sure we stay inside the bounds.
        if (resultX * maxScale - w2 <= minX * maxScale) {
            resultX = minX * maxScale + w2;
            resultX /= maxScale;
        } else if (resultX * maxScale + w2 >= maxX * maxScale) {
            resultX = maxX * maxScale - w2;
            resultX /= maxScale;
        }

        if (resultY * maxScale - h2 <= minY * maxScale) {
            resultY = minY * maxScale + h2;
            resultY /= maxScale;
        } else if (resultY * maxScale + h2 >= maxY * maxScale) {
            resultY = maxY * maxScale - h2;
            resultY /= maxScale;
        }
    }

    if (resultX != startX || resultY != startY) {
        // If we made changes just drop any anchor point
        requestedCamera.anchor.reset();
        requestedCamera.center = std::optional(Projection::unproject({resultX, resultY}, requestedScale));
    }
}

ScreenCoordinate TransformState::getCenterOffset() const {
    return {0.5 * (edgeInsets.left() - edgeInsets.right()), 0.5 * (edgeInsets.top() - edgeInsets.bottom())};
}

void TransformState::moveLatLng(const LatLng& latLng, const ScreenCoordinate& anchor) {
    auto centerCoord = Projection::project(getLatLng(LatLng::Unwrapped), scale);
    auto latLngCoord = Projection::project(latLng, scale);
    auto anchorCoord = Projection::project(screenCoordinateToLatLng(anchor), scale);
    setLatLngZoom(Projection::unproject(centerCoord + latLngCoord - anchorCoord, scale), getZoom());
}

void TransformState::setLatLngZoom(const LatLng& latLng, double zoom) {
    LatLng constrained = latLng;
    constrained = bounds.constrain(latLng);

    double newScale = util::clamp(zoomScale(zoom), min_scale, max_scale);
    const double newWorldSize = newScale * util::tileSize_D;
    Bc = newWorldSize / util::DEGREES_MAX;
    Cc = newWorldSize / util::M2PI;

    const double m = 1 - 1e-15;
    const double f = util::clamp(std::sin(util::deg2rad(constrained.latitude())), -m, m);

    ScreenCoordinate point = {
        -constrained.longitude() * Bc,
        0.5 * Cc * std::log((1 + f) / (1 - f)),
    };
    setScalePoint(newScale, point);
}

void TransformState::setCenterAltitude(double alt_m) {
    z = alt_m / Projection::getMetersPerPixelAtLatitude(getLatLng().latitude(), getZoom());
    requestMatricesUpdate = true;
}

void TransformState::setScalePoint(const double newScale, const ScreenCoordinate& point) {
    double constrainedScale = newScale;
    ScreenCoordinate constrainedPoint = point;
    constrain(constrainedScale, constrainedPoint.x, constrainedPoint.y);

    scale = constrainedScale;
    x = constrainedPoint.x;
    y = constrainedPoint.y;
    Bc = Projection::worldSize(scale) / util::DEGREES_MAX;
    Cc = Projection::worldSize(scale) / util::M2PI;
    requestMatricesUpdate = true;
}

float TransformState::getCameraToTileDistance(const UnwrappedTileID& tileID) const {
    mat4 tileProjectionMatrix;
    matrixFor(tileProjectionMatrix, tileID);
    matrix::multiply(tileProjectionMatrix, getProjectionMatrix(), tileProjectionMatrix);
    vec4 tileCenter = {{util::tileSize_D / 2, util::tileSize_D / 2, 0, 1}};
    vec4 projectedCenter;
    matrix::transformMat4(projectedCenter, tileCenter, tileProjectionMatrix);
    return static_cast<float>(projectedCenter[3]);
}

float TransformState::maxPitchScaleFactor() const {
    if (size.isEmpty()) {
        return {};
    }
    auto latLng = screenCoordinateToLatLng({0, static_cast<float>(getSize().height)});

    Point<double> pt = Projection::project(latLng, scale) / util::tileSize_D;
    vec4 p = {{pt.x, pt.y, 0, 1}};
    vec4 topPoint;
    matrix::transformMat4(topPoint, p, getCoordMatrix());
    return static_cast<float>(topPoint[3]) / getCameraToCenterDistance();
}

} // namespace mln
