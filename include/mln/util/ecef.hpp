#pragma once

#include <mln/util/geo.hpp>
#include <mln/util/vectors.hpp>

namespace mln {
namespace util {
namespace ecef {

// WGS84: semieje mayor (ecuatorial) y menor (polar), en metros.
constexpr double WGS84_SEMI_MAJOR_M = 6378137.0;
constexpr double WGS84_SEMI_MINOR_M = 6356752.3142;

// Convierte lat/lon/altura (grados, metros sobre el elipsoide) a ECEF real (metros,
// origen en el centro de la Tierra). Doble precision: a esta magnitud (~6.378.000 m)
// un float32 pierde ~0.5 m, por eso todo este calculo vive en CPU en double y solo el
// remanente chico (relativo a un origen local) se sube a la GPU como float (ADR 0034,
// patron RTE de ATAK, sin copiar su codigo).
vec3 llaToEcef(const LatLng& latLng, double heightMeters);

// Inversa: ECEF real -> lat/lon/altura. Iterativa (Bowring), converge en pocas vueltas.
LatLng ecefToLatLng(const vec3& ecef, double* heightMetersOut = nullptr);

// Normal unitario a la superficie del elipsoide en ese punto (direccion "arriba" real,
// no exactamente radial salvo en el ecuador/polos). Se usa para desplazar por elevacion
// en el shader sin perder precision.
vec3 surfaceNormal(const LatLng& latLng);

} // namespace ecef
} // namespace util
} // namespace mln
