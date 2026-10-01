#pragma once

#include <mln/map/camera.hpp>
#include <mln/map/mode.hpp>
#include <mln/style/projection.hpp>
#include <mln/util/camera.hpp>
#include <mln/util/constants.hpp>
#include <mln/util/geo.hpp>
#include <mln/util/ecef.hpp>
#include <mln/util/geometry.hpp>
#include <mln/util/mat4.hpp>
#include <mln/util/projection.hpp>
#include <mln/util/size.hpp>

#include <cstdint>
#include <array>
#include <limits>
#include <functional>
#include <optional>

namespace mln {

class UnwrappedTileID;
class TileCoordinate;

struct TransformStateProperties {
    TransformStateProperties& withX(const std::optional<double>& val) {
        x = val;
        return *this;
    }
    TransformStateProperties& withY(const std::optional<double>& val) {
        y = val;
        return *this;
    }
    TransformStateProperties& withZ(const std::optional<double>& val) {
        z = val;
        return *this;
    }
    TransformStateProperties& withScale(const std::optional<double>& val) {
        scale = val;
        return *this;
    }
    TransformStateProperties& withFov(const std::optional<double>& val) {
        fov = val;
        return *this;
    }
    TransformStateProperties& withBearing(const std::optional<double>& val) {
        bearing = val;
        return *this;
    }
    TransformStateProperties& withPitch(const std::optional<double>& val) {
        pitch = val;
        return *this;
    }
    TransformStateProperties& withRoll(const std::optional<double>& val) {
        roll = val;
        return *this;
    }
    TransformStateProperties& withXSkew(const std::optional<double>& val) {
        xSkew = val;
        return *this;
    }
    TransformStateProperties& withYSkew(const std::optional<double>& val) {
        ySkew = val;
        return *this;
    }
    TransformStateProperties& withAxonometric(const std::optional<bool>& val) {
        axonometric = val;
        return *this;
    }
    TransformStateProperties& withPanningInProgress(const std::optional<bool>& val) {
        panning = val;
        return *this;
    }
    TransformStateProperties& withScalingInProgress(const std::optional<bool>& val) {
        scaling = val;
        return *this;
    }
    TransformStateProperties& withRotatingInProgress(const std::optional<bool>& val) {
        rotating = val;
        return *this;
    }
    TransformStateProperties& withEdgeInsets(const std::optional<EdgeInsets>& val) {
        edgeInsets = val;
        return *this;
    }
    TransformStateProperties& withSize(const std::optional<Size>& val) {
        size = val;
        return *this;
    }
    TransformStateProperties& withConstrainMode(const std::optional<ConstrainMode>& val) {
        constrain = val;
        return *this;
    }
    TransformStateProperties& withNorthOrientation(const std::optional<NorthOrientation>& val) {
        northOrientation = val;
        return *this;
    }
    TransformStateProperties& withViewportMode(const std::optional<ViewportMode>& val) {
        viewPortMode = val;
        return *this;
    }
    TransformStateProperties withFrustumOffset(const std::optional<EdgeInsets>& val) {
        frustumOffset = val;
        return *this;
    }

    std::optional<double> x;
    std::optional<double> y;
    std::optional<double> z;
    std::optional<double> fov;
    std::optional<double> scale;
    std::optional<double> bearing;
    std::optional<double> pitch;
    std::optional<double> roll;
    std::optional<double> xSkew;
    std::optional<double> ySkew;
    std::optional<bool> axonometric;
    std::optional<bool> panning;
    std::optional<bool> scaling;
    std::optional<bool> rotating;
    std::optional<EdgeInsets> edgeInsets;
    std::optional<Size> size;
    std::optional<ConstrainMode> constrain;
    std::optional<NorthOrientation> northOrientation;
    std::optional<ViewportMode> viewPortMode;
    std::optional<EdgeInsets> frustumOffset;
};

class TransformState {
public:
    TransformState(ConstrainMode = ConstrainMode::HeightOnly, ViewportMode = ViewportMode::Default);

    void setProperties(const TransformStateProperties& properties);

    // Matrix
    void matrixFor(mat4&, const UnwrappedTileID&) const;
    void getProjMatrix(mat4& matrix, uint16_t nearZ = 1, bool aligned = false) const;

    // Projection
    const style::ProjectionDefinition& getProjection() const { return projection; }
    void setProjection(const style::ProjectionDefinition&);

    double getGlobeness() const;
    bool isGlobeRendering() const;

    const mat4& getGlobeMatrix() const;
    const mat4& getInvGlobeMatrix() const;
    const vec4& getGlobeClippingPlane() const;
    const vec3& getGlobeCameraPosition() const;
    double getGlobeRadiusPixels() const;

    static vec4 getTileMercatorCoords(const UnwrappedTileID&);

    ScreenCoordinate latLngToScreenCoordinateGlobe(const LatLng&, bool& occluded) const;
    std::optional<LatLng> screenCoordinateToLatLngGlobe(const ScreenCoordinate&) const;
    vec3 getRayDirectionFromPixel(const ScreenCoordinate&) const;

    // Dimensions
    Size getSize() const;
    void setSize(const Size& size_);

    EdgeInsets getFrustumOffset() const;
    void setFrustumOffset(const EdgeInsets& frustumOffset_);

    // North Orientation
    NorthOrientation getNorthOrientation() const;
    double getNorthOrientationAngle() const;
    void setNorthOrientation(NorthOrientation);

    // Constrain mode
    ConstrainMode getConstrainMode() const;
    void setConstrainMode(ConstrainMode);

    // Viewport mode
    ViewportMode getViewportMode() const;
    void setViewportMode(ViewportMode val);

    CameraOptions getCameraOptions(const std::optional<EdgeInsets>&) const;

    // EdgeInsects
    EdgeInsets getEdgeInsets() const { return edgeInsets; }
    void setEdgeInsets(const EdgeInsets&);

    // Position
    LatLng getLatLng(LatLng::WrapMode = LatLng::Unwrapped) const;
    double getCenterAltitude() const;
    double pixel_x() const;
    double pixel_y() const;

    // Zoom
    double getZoom() const;
    uint8_t getIntegerZoom() const;
    double getZoomFraction() const;

    // Scale
    double getScale() const;
    void setScale(double);

    // Positions
    double getX() const;
    void setX(double);
    double getY() const;
    void setY(double);
    double getZ() const;
    void setZ(double);

    // Bounds
    void setLatLngBounds(LatLngBounds);
    LatLngBounds getLatLngBounds() const;
    void setMinZoom(double);
    double getMinZoom() const;
    void setMaxZoom(double);
    double getMaxZoom() const;
    void setMinPitch(double);
    double getMinPitch() const;
    void setMaxPitch(double);
    double getMaxPitch() const;
    double getMinFieldOfView() const;
    double getMaxFieldOfView() const;

    // Rotation
    double getBearing() const;
    void setBearing(double);
    float getFieldOfView() const;
    void setFieldOfView(double);
    float getCameraToCenterDistance() const;
    double getPitch() const;
    void setPitch(double);
    double getRoll() const;
    void setRoll(double);

    double getXSkew() const;
    void setXSkew(double);
    double getYSkew() const;
    void setYSkew(double);
    bool getAxonometric() const;
    void setAxonometric(bool);

    // State
    bool isChanging() const;
    bool isRotating() const;
    void setRotatingInProgress(bool val) { rotating = val; }
    bool isScaling() const;
    void setScalingInProgress(bool val) { scaling = val; }
    bool isPanning() const;
    void setPanningInProgress(bool val) { panning = val; }
    bool isGestureInProgress() const;
    void setGestureInProgress(bool val) { gestureInProgress = val; }

    // Conversion
    ScreenCoordinate latLngToScreenCoordinate(const LatLng&) const;
    ScreenCoordinate latLngToScreenCoordinate(const LatLng&, vec4&) const;
    LatLng screenCoordinateToLatLng(const ScreenCoordinate&, LatLng::WrapMode = LatLng::Unwrapped) const;
    // Implements mapbox-gl-js pointCoordinate() : MercatorCoordinate.
    TileCoordinate screenCoordinateToTileCoordinate(const ScreenCoordinate&, uint8_t atZoom) const;

    double zoomScale(double zoom) const;
    double scaleZoom(double scale) const;

    bool valid() const { return !size.isEmpty() && (scale >= min_scale && scale <= max_scale); }

    float getCameraToTileDistance(const UnwrappedTileID&) const;
    float maxPitchScaleFactor() const;

    /** Recenter the map so that the given coordinate is located at the given
        point on screen. */
    void moveLatLng(const LatLng&, const ScreenCoordinate&);
    void setLatLngZoom(const LatLng& latLng, double zoom);
    void setCenterAltitude(double alt_m);

    void constrain(double& scale, double& x, double& y) const;
    bool constrainScreen(double& scale_, double& x_, double& y_) const;
    void constrainCameraAndZoomToBounds(CameraOptions& camera, double& zoom) const;

    const mat4& getProjectionMatrix() const;
    const mat4& getInvProjectionMatrix() const;

    FreeCameraOptions getFreeCameraOptions() const;
    void setFreeCameraOptions(const FreeCameraOptions& options);

    // Camara ECEF real (ADR 0034) como camara ORBITAL alrededor del foco, igual que ATAK (ADR 0040):
    // getLatLng() es el FOCO (lo que se mira, sobre el terreno), el zoom da la DISTANCIA ojo-foco
    // (getCameraToCenterDistance x metros/px: el range(gsd) de ATAK) y pitch/bearing son el tilt y el
    // azimut (pitch 0 = nadir). Es la misma camara que la mercator de MapLibre -- en este modo el fov pasa a
    // 45 grados, el de la perspectiva ECEF --, asi los gestos, la colocacion de simbolos y el seguimiento
    // coinciden con lo dibujado. Apagada por defecto (invariante 9). `heightMetersAboveEllipsoid` queda para
    // la herramienta de primera persona (C3); la navegacion orbital no lo usa.
    void setRealAltitudeMode(bool enabled, double heightMetersAboveEllipsoid = 0.0);
    bool isRealAltitudeModeEnabled() const { return realAltitudeEnabled; }
    double getRealAltitudeMeters() const { return realAltitudeMeters; }

    // Camara orbital de un frame (ADR 0040): foco, ojo y planos, en ECEF real (WGS84).
    struct EcefCamera {
        LatLng focus;
        double focusElevationM = 0.0;
        double rangeM = 0.0;   // distancia ojo-foco
        vec3 focusEcef{};
        vec3 eyeEcef{};
        LatLng eyeLatLng;
        double eyeAglM = 0.0;  // altura del ojo sobre el terreno bajo el ojo
        vec3 forward{};        // direccion de vista (ojo -> foco), unitaria
        double nearM = 0.0;
        double farM = 0.0;
        bool collided = false; // se subio el ojo para no quedar a menos de 10 m del terreno (AdjustCamera de ATAK)
    };
    // `focusElevationM`: terreno en el foco. `groundAt`: terreno en un punto, para el suelo bajo el ojo; sin
    // el se toma la elevacion del foco (estimacion plana, suficiente para los coverings de tiles).
    EcefCamera computeEcefCamera(double focusElevationM,
                                 const std::function<double(const LatLng&)>& groundAt = {}) const;

    // Direccion de vista (unitaria, ECEF) segun pitch/bearing en la base ENU del foco.
    vec3 getCameraForwardEcef() const;

    // FOV vertical de la camara con altura real: 45 grados, el de ATAK (MapSceneModel2, HVFOV = 22,5 de
    // medio angulo). Independiente del fov del mapa mercator (36,87 por defecto en MapLibre).
    static constexpr double kEcefFieldOfViewRad = 45.0 * 3.14159265358979323846 / 180.0;

    // Drapes multi-resolucion del terreno 3D (ADR 0037, patron de GLMapView2.cpp de ATAK -- solo
    // lectura, GPLv3, no copiado): kEcefDrapeCount capturas nadir de la imagen, todas centradas
    // en el punto de mira, a 1x/4x/32x de una resolucion base. Base = la resolucion del view en
    // el punto de mira (drawMapResolution = scene.gsd en ATAK) con piso en la resolucion nativa
    // de la imagen (a 1,7 m sobre el suelo la del view es cientos de veces mas fina que z19: puro
    // overzoom), por scaleAdj = 1 + 1.1*sin(tilt). Proporciones de textura 2048/1024/1024 = las
    // regiones del atlas de ATAK (1/2, 1/4, 1/4 del ancho).
    static constexpr size_t kEcefDrapeCount = 3;
    static constexpr std::array<uint32_t, kEcefDrapeCount> kEcefDrapeTextureSizesPx = {2048, 1024, 1024};
    static constexpr std::array<double, kEcefDrapeCount> kEcefDrapeResolutionMultipliers = {1.0, 4.0, 32.0};

    struct EcefDrapeArea {
        LatLng center;
        double radiusMeters;
    };
    std::array<EcefDrapeArea, kEcefDrapeCount> computeEcefDrapeAreas() const;

    // Vista nadir sintetica (pitch=0, bearing=0) centrada en `area`, con el zoom mercator que le
    // da a `textureSizePx` la resolucion de area.radiusMeters. El covering de tiles normal
    // (TilePyramid::update) aplicado a este estado trae la imagen real de la zona/resolucion que
    // ese drape cubre, desacoplado del covering de la camara real.
    static TransformState makeSyntheticDrapeState(const TransformState& base,
                                                  const EcefDrapeArea& area,
                                                  uint32_t textureSizePx);

    // Matriz view-projection de la camara orbital `camera` para un sub-tile cuyo origen local es `originEcef`
    // (ECEF real, WGS84). Toda la resta ojo-origen se hace en double (RTE, ADR 0034) y solo el resultado
    // final (ya de magnitud chica, relativo al ojo) se convierte a float al armar el uniform buffer.
    mat4 getEcefTileMatrix(const vec3& originEcef, const EcefCamera& orbit) const;

private:
    bool rotatedNorth() const;

    // Viewport center offset, from [size.width / 2, size.height / 2], defined
    // by |edgeInsets| in screen coordinates, with top left origin.
    ScreenCoordinate getCenterOffset() const;

    mat4 coordinatePointMatrix(const mat4& projMatrix) const;
    mat4 getPixelMatrix() const;

    void setScalePoint(double scale, const ScreenCoordinate& point);

    void updateMatricesIfNeeded() const;
    void updateGlobeMatrices() const;
    bool needsMatricesUpdate() const { return requestMatricesUpdate; }

    bool setCameraPosition(const vec3& position);
    bool setCameraOrientation(const Quaternion& orientation);
    void updateCameraState() const;
    void updateStateFromCamera();

    const mat4& getCoordMatrix() const;
    const mat4& getInvertedMatrix() const;

private:
    // Limit the amount of zooming possible on the map.
    double min_scale = std::pow(2, 0);
    double max_scale = std::pow(2, util::DEFAULT_MAX_ZOOM);

    // Limit the amount of pitch
    double minPitch = util::PITCH_MIN;
    double maxPitch = util::DEFAULT_PITCH_MAX;
    double minFov = util::deg2rad(0.1);
    double maxFov = util::deg2rad(150.0);

    // map position
    double x = 0, y = 0, z = 0;
    double bearing = 0;
    double scale = 1;
    double fov = util::DEFAULT_FOV;
    double pitch = 0.0;
    double roll = 0.0;
    double xSkew = 0.0;
    double ySkew = 1.0;

    // cache values for spherical mercator math
    double Bc = Projection::worldSize(scale) / util::DEGREES_MAX;
    double Cc = Projection::worldSize(scale) / util::M2PI;

    mutable double globeRadiusPixels = 0;
    mutable vec3 globeCameraPosition;

    // Camara con altura real (ADR 0034, ver getEcefTileMatrix). Apagada por defecto.
    bool realAltitudeEnabled = false;
    double realAltitudeMeters = 0.0;

    EdgeInsets frustumOffset;
    EdgeInsets edgeInsets;

    mutable vec4 globeClippingPlane;

    LatLngBounds bounds;

    mutable mat4 projectionMatrix;
    mutable mat4 invProjectionMatrix;
    mutable mat4 coordMatrix;
    mutable mat4 invertedMatrix;
    mutable mat4 globeMatrix;
    mutable mat4 invGlobeMatrix;

    mutable util::Camera camera;

    ConstrainMode constrainMode;
    ViewportMode viewportMode;

    // logical dimensions
    Size size;

    NorthOrientation orientation = NorthOrientation::Upwards;

    // animation state
    bool rotating = false;
    bool scaling = false;
    bool panning = false;
    bool gestureInProgress = false;

    bool axonometric = false;

    style::ProjectionDefinition projection;

    mutable bool requestMatricesUpdate{true};
};

} // namespace mln
