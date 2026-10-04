#pragma once

#include <mln/util/mat4.hpp>
#include <mln/util/vectors.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <tuple>
#include <vector>

namespace mln {

// LOD de la malla del terreno con camara de altura real (ADR 0039). Mismo diseño que el servicio de
// terreno de ATAK (ElMgrTerrainRenderService + TiledGlobe), reimplementado sin copiar su codigo y
// trasladado de su grilla EPSG:4326 a tiles mercator:
//  - quadtree global desde z0, no atado a los tiles DEM cargados;
//  - se subdivide mientras el error de pantalla (SSE) supere 10 px, la celda intersecte el frustum y
//    no quede mas alla del far. El error geometrico es la resolucion de pixel de un tile de 256 px
//    del nivel (lado/128) y la distancia se mide al punto mas cercano de la celda;
//  - cada celda toma el relieve del tile DEM cargado mas fino que la contiene (derivado del
//    ancestro); sin ninguno queda plana;
//  - dos celdas vecinas difieren en a lo sumo 1 nivel (salvo las 7+ niveles por debajo de la mas
//    fina), asi los skirts solo tapan grietas minimas.
// Logica pura: no toca GPU ni estado del mapa, se testea sola (test/renderer/terrain_ecef_lod.test.cpp).

// Largo de los skirts de la malla ECEF (el de ATAK). Tambien entra en la caja envolvente del frustum.
constexpr double kEcefTerrainSkirtM = 500.0;
// Cuadros por lado de la malla ECEF de cada celda (ATAK: 32 postes por lado).
constexpr int32_t kEcefTerrainMeshSize = 32;

struct EcefLodTileKey {
    uint8_t z = 0;
    uint32_t x = 0;
    uint32_t y = 0;

    bool operator<(const EcefLodTileKey& o) const { return std::tie(z, x, y) < std::tie(o.z, o.x, o.y); }
    bool operator==(const EcefLodTileKey& o) const { return z == o.z && x == o.x && y == o.y; }
};

// Rango de elevacion (m) de un tile DEM cargado.
struct EcefDemRange {
    double minM = 0.0;
    double maxM = 0.0;
};
using EcefDemIndex = std::map<EcefLodTileKey, EcefDemRange>;

struct EcefLodCamera {
    vec3 originEcef{};      // posicion real del ojo: suelo + altura sobre el suelo
    double mercatorX = 0.0; // posicion de la camara en mercator normalizado [0,1)
    double mercatorY = 0.0;
    // Foco de la camara en mercator X (centro de los drapes). Cada celda se entrega en la copia del mundo
    // mas cercana a el, para que su UV caiga dentro de los drapes. NaN = usar mercatorX.
    double focusMercatorX = std::numeric_limits<double>::quiet_NaN();
    double farM = 0.0;      // far plane real (horizonte fisico)
    double lambda = 0.0;    // (alto de pantalla en px fisicos / 2) / tan(fov / 2)
    mat4 viewProjRte{};     // clip = viewProjRte * (p - originEcef, 1)
};

struct EcefLodCell {
    EcefLodTileKey tile;                // canonico
    int16_t wrap = 0;                   // copia del mundo en longitud: la mas cercana al foco
    std::optional<EcefLodTileKey> dem;  // tile DEM mas fino que la contiene; nullopt = celda plana
    float demScale = 1.0f;              // ventana de la celda dentro del tile DEM
    float demOffsetX = 0.0f;
    float demOffsetY = 0.0f;
    double distanceM = 0.0;             // al punto mas cercano de la celda
};

struct EcefLodResult {
    std::vector<EcefLodCell> cells;     // ordenadas de la mas cercana a la mas lejana
    bool fuseTripped = false;           // se llego al fusible de celdas (nunca deberia pasar)
};

EcefLodResult selectEcefTerrainCells(const EcefLodCamera& camera, const EcefDemIndex& dems);

} // namespace mln
