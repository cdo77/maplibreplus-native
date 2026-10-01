#include <mln/util/ecef.hpp>

#include <mln/math/angles.hpp>

#include <cmath>

namespace mln {
namespace util {
namespace ecef {

namespace {
constexpr double kFlattening = (WGS84_SEMI_MAJOR_M - WGS84_SEMI_MINOR_M) / WGS84_SEMI_MAJOR_M;
constexpr double kEccentricitySquared = 2.0 * kFlattening - kFlattening * kFlattening;
} // namespace

vec3 llaToEcef(const LatLng& latLng, double heightMeters) {
    const double latRad = deg2rad(latLng.latitude());
    const double lonRad = deg2rad(latLng.longitude());
    const double sinLat = std::sin(latRad);
    const double cosLat = std::cos(latRad);
    const double sinLon = std::sin(lonRad);
    const double cosLon = std::cos(lonRad);

    // Radio de curvatura en el primer vertical.
    const double n = WGS84_SEMI_MAJOR_M / std::sqrt(1.0 - kEccentricitySquared * sinLat * sinLat);

    return {
        (n + heightMeters) * cosLat * cosLon,
        (n + heightMeters) * cosLat * sinLon,
        (n * (1.0 - kEccentricitySquared) + heightMeters) * sinLat,
    };
}

LatLng ecefToLatLng(const vec3& ecef, double* heightMetersOut) {
    const double x = ecef[0];
    const double y = ecef[1];
    const double z = ecef[2];
    const double lonRad = std::atan2(y, x);

    const double p = std::sqrt(x * x + y * y);
    // Sobre el eje polar (p = 0) la iteracion divide 0/0 (en el centro de la Tierra daba NaN y tumbaba el render
    // con la camara muy alejada, field-test 01-10): ahi la latitud es +-90 y la altura |z| - b.
    if (p < 1e-9) {
        if (heightMetersOut) {
            *heightMetersOut = std::abs(z) - WGS84_SEMI_MINOR_M;
        }
        return LatLng{z >= 0.0 ? 90.0 : -90.0, rad2deg(lonRad)};
    }

    // Bowring: arranca con una latitud geocentrica y refina hacia la geodesica.
    double latRad = std::atan2(z, p * (1.0 - kEccentricitySquared));
    for (int i = 0; i < 5; i++) {
        const double sinLat = std::sin(latRad);
        const double n = WGS84_SEMI_MAJOR_M / std::sqrt(1.0 - kEccentricitySquared * sinLat * sinLat);
        const double height = p / std::cos(latRad) - n;
        latRad = std::atan2(z, p * (1.0 - kEccentricitySquared * n / (n + height)));
    }

    if (heightMetersOut) {
        const double sinLat = std::sin(latRad);
        const double n = WGS84_SEMI_MAJOR_M / std::sqrt(1.0 - kEccentricitySquared * sinLat * sinLat);
        *heightMetersOut = p / std::cos(latRad) - n;
    }

    return LatLng{rad2deg(latRad), rad2deg(lonRad)};
}

vec3 surfaceNormal(const LatLng& latLng) {
    const double latRad = deg2rad(latLng.latitude());
    const double lonRad = deg2rad(latLng.longitude());
    const double sinLat = std::sin(latRad);
    const double cosLat = std::cos(latRad);
    const double sinLon = std::sin(lonRad);
    const double cosLon = std::cos(lonRad);

    // Normal geodesico: por definicion de latitud geodesica, la normal al elipsoide forma el angulo
    // lat con el ecuador -- es (cos lat cos lon, cos lat sin lon, sin lat), ya unitaria. (El gradiente
    // x/a^2, y/a^2, z/b^2 vale con las coordenadas del PUNTO, no con cos/sin de la latitud geodesica: esa
    // mezcla torcia el "arriba" ~0,19 grados a la latitud de Neuquen.)
    return {cosLat * cosLon, cosLat * sinLon, sinLat};
}

} // namespace ecef
} // namespace util
} // namespace mln
