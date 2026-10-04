#include <mln/renderer/terrain_ecef_lod.hpp>

#include <mln/util/ecef.hpp>

#include <algorithm>
#include <cmath>
#include <set>

namespace mln {

namespace {

// Constantes del servicio de terreno de ATAK, por su significado fisico (ADR 0039).
constexpr double kMaxScreenSpaceError = 10.0;   // MAX_SSE de ATAK en Android
constexpr uint8_t kMaxLevel = 22;               // MAX_LEVEL 21 de ATAK en 4326 ~ z22 mercator (mismo tamaño)
constexpr double kGeometricErrorDivisor = 128.0; // resolucion de pixel de un tile de 256 px = lado / 128
constexpr int kNeighborExemptLevels = 7;        // ATAK no equilibra lo que esta 7+ niveles bajo el mas fino
// Por debajo de este nivel una celda mide miles de km: la curvatura no entra en la caja de 9 puntos,
// asi que no se descarta por frustum (solo por far).
constexpr uint8_t kAlwaysIntersectBelowLevel = 4;
// Fusible contra una explosion combinatoria; lo normal en ATAK son 150-300 celdas.
constexpr size_t kCellFuse = 2000;

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = util::ecef::WGS84_SEMI_MAJOR_M;
constexpr double kEarthCircumferenceM = 2.0 * kPi * kEarthRadiusM;

// Nodo del quadtree, siempre en el mundo canonico (x en [0, 2^z)). En ECEF hay un solo planeta: con
// varias copias del mundo (como el camino mercator) el globo se dibujaba hasta 3 veces en el mismo
// lugar, con la imagen corrida un mundo entero (globo rayado al alejarse, field-test 01-10).
struct Node {
    uint8_t z;
    int64_t x;
    int64_t y;

    bool operator<(const Node& o) const { return std::tie(z, x, y) < std::tie(o.z, o.x, o.y); }
};

// Orden de la lista de trabajo de vecinos: de la mas fina a la mas gruesa.
struct FinerFirst {
    bool operator()(const Node& a, const Node& b) const {
        return std::tie(b.z, a.x, a.y) < std::tie(a.z, b.x, b.y);
    }
};

LatLng mercatorToLatLng(double mx, double my) {
    const double lon = mx * 360.0 - 180.0;
    const double lat = std::atan(std::sinh(kPi * (1.0 - 2.0 * my))) * 180.0 / kPi;
    return LatLng(lat, lon);
}

class Selector {
public:
    Selector(const EcefLodCamera& camera_, const EcefDemIndex& dems_)
        : camera(camera_),
          dems(dems_) {
        // Ancestros estrictos de cada tile DEM, con la union de sus rangos: sirven para saber que
        // una celda sin DEM propio tiene relieve mas fino adentro (hay que subdividirla) y para
        // acotar su caja de elevacion.
        for (const auto& [key, range] : dems) {
            maxDemZ = std::max(maxDemZ, key.z);
            for (uint8_t z = 0; z < key.z; ++z) {
                const int shift = key.z - z;
                const EcefLodTileKey ancestor{z, key.x >> shift, key.y >> shift};
                auto [it, inserted] = subtreeRanges.emplace(ancestor, range);
                if (!inserted) {
                    it->second.minM = std::min(it->second.minM, range.minM);
                    it->second.maxM = std::max(it->second.maxM, range.maxM);
                }
            }
        }
    }

    EcefLodResult run() {
        std::set<Node> leaves;
        refine(Node{0, 0, 0}, leaves);
        enforceNeighborLevels(leaves);

        EcefLodResult result;
        result.fuseTripped = fuseTripped;
        result.cells.reserve(leaves.size());
        for (const Node& n : leaves) {
            result.cells.push_back(makeCell(n));
        }
        std::sort(result.cells.begin(), result.cells.end(),
                  [](const EcefLodCell& a, const EcefLodCell& b) { return a.distanceM < b.distanceM; });
        return result;
    }

private:
    static int64_t tileCount(uint8_t z) { return int64_t{1} << z; }

    // Desplazamiento entero que lleva `fromX` a la copia del mundo mas cercana a `toX` (mercator [0,1)).
    static double nearestCopyShift(double fromX, double toX) { return std::round(toX - fromX); }

    static EcefLodTileKey canonicalOf(const Node& n) {
        return {n.z, static_cast<uint32_t>(n.x), static_cast<uint32_t>(n.y)};
    }

    // Copia del mundo en la que se entrega la celda: la mas cercana al foco (centro de los drapes).
    int16_t wrapOf(const Node& n) const {
        const double centerX = (static_cast<double>(n.x) + 0.5) / static_cast<double>(tileCount(n.z));
        const double focusX = std::isnan(camera.focusMercatorX) ? camera.mercatorX : camera.focusMercatorX;
        return static_cast<int16_t>(nearestCopyShift(centerX, focusX));
    }

    // Tile DEM cargado mas fino que contiene la celda (el ancestro del que ATAK deriva el relieve).
    std::optional<EcefLodTileKey> finestDem(const Node& n) const {
        const EcefLodTileKey c = canonicalOf(n);
        for (int z = std::min<int>(n.z, maxDemZ); z >= 0; --z) {
            const int shift = n.z - z;
            const EcefLodTileKey key{static_cast<uint8_t>(z), c.x >> shift, c.y >> shift};
            if (dems.contains(key)) {
                return key;
            }
        }
        return std::nullopt;
    }

    EcefDemRange rangeOf(const Node& n) const {
        if (const auto dem = finestDem(n)) {
            return dems.at(*dem);
        }
        if (const auto it = subtreeRanges.find(canonicalOf(n)); it != subtreeRanges.end()) {
            return it->second;
        }
        return {};
    }

    // Sin DEM ancestro pero con DEM cargado adentro: se subdivide hasta encontrarlo (ATAK solo
    // recursa hacia hijos con dato; aca el dato fino esta mas abajo).
    bool needsFinerDem(const Node& n) const {
        return !finestDem(n) && subtreeRanges.contains(canonicalOf(n));
    }

    static double sideM(const Node& n) {
        const double centerY = (static_cast<double>(n.y) + 0.5) / static_cast<double>(tileCount(n.z));
        const double lat = mercatorToLatLng(0.5, centerY).latitude() * kPi / 180.0;
        return kEarthCircumferenceM * std::cos(lat) / static_cast<double>(tileCount(n.z));
    }

    // Distancia al punto mas cercano de la celda, a su elevacion media (computeDistanceSquared de ATAK).
    double closestDistanceM(const Node& n, const EcefDemRange& range) const {
        const double count = static_cast<double>(tileCount(n.z));
        // La camara en su copia mas cercana a la celda: del otro lado del antimeridiano tambien es vecina.
        const double cameraX = camera.mercatorX - nearestCopyShift((n.x + 0.5) / count, camera.mercatorX);
        const double mx = std::clamp(cameraX, n.x / count, (n.x + 1) / count);
        const double my = std::clamp(camera.mercatorY, n.y / count, (n.y + 1) / count);
        const vec3 p = util::ecef::llaToEcef(mercatorToLatLng(mx, my), (range.minM + range.maxM) * 0.5);
        const double dx = p[0] - camera.originEcef[0];
        const double dy = p[1] - camera.originEcef[1];
        const double dz = p[2] - camera.originEcef[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Caja de la celda contra el frustum: 9 puntos (esquinas, medios de lado y centro) a la elevacion
    // minima (menos el skirt, como la caja de ATAK) y maxima (mas la flecha de la curvatura). Queda
    // afuera si todos caen del lado exterior del mismo plano de recorte.
    bool intersects(const Node& n, const EcefDemRange& range, double distanceM) const {
        if (distanceM > camera.farM) {
            return false;
        }
        if (n.z < kAlwaysIntersectBelowLevel) {
            return true;
        }
        const double side = sideM(n);
        const double sagittaM = side * side / (8.0 * kEarthRadiusM);
        const double heights[2] = {range.minM - kEcefTerrainSkirtM, range.maxM + sagittaM};
        const double count = static_cast<double>(tileCount(n.z));
        unsigned outsideAll = 0x3F;
        for (int iy = 0; iy <= 2 && outsideAll; ++iy) {
            for (int ix = 0; ix <= 2 && outsideAll; ++ix) {
                const LatLng ll = mercatorToLatLng((n.x + ix * 0.5) / count, (n.y + iy * 0.5) / count);
                for (const double h : heights) {
                    const vec3 p = util::ecef::llaToEcef(ll, h);
                    const double rx = p[0] - camera.originEcef[0];
                    const double ry = p[1] - camera.originEcef[1];
                    const double rz = p[2] - camera.originEcef[2];
                    const mat4& m = camera.viewProjRte;
                    const double cx = m[0] * rx + m[4] * ry + m[8] * rz + m[12];
                    const double cy = m[1] * rx + m[5] * ry + m[9] * rz + m[13];
                    const double cz = m[2] * rx + m[6] * ry + m[10] * rz + m[14];
                    const double cw = m[3] * rx + m[7] * ry + m[11] * rz + m[15];
                    unsigned outside = 0;
                    outside |= (cx < -cw) ? 0x01u : 0u;
                    outside |= (cx > cw) ? 0x02u : 0u;
                    outside |= (cy < -cw) ? 0x04u : 0u;
                    outside |= (cy > cw) ? 0x08u : 0u;
                    outside |= (cz < -cw) ? 0x10u : 0u;
                    outside |= (cz > cw) ? 0x20u : 0u;
                    outsideAll &= outside;
                }
            }
        }
        return outsideAll == 0;
    }

    // Subdivision por SSE (shouldRecurse de ATAK): sse > 10 px y la celda a la vista.
    void refine(const Node& n, std::set<Node>& leaves) {
        if (n.y < 0 || n.y >= tileCount(n.z)) {
            return;
        }
        if (leaves.size() >= kCellFuse) {
            fuseTripped = true;
            return;
        }
        const EcefDemRange range = rangeOf(n);
        const double distanceM = closestDistanceM(n, range);
        if (!intersects(n, range, distanceM)) {
            return;
        }
        const double sse = camera.lambda * (sideM(n) / kGeometricErrorDivisor) / std::max(distanceM, 1e-3);
        if (n.z < kMaxLevel && (sse > kMaxScreenSpaceError || needsFinerDem(n))) {
            for (int i = 0; i < 4; ++i) {
                refine(Node{static_cast<uint8_t>(n.z + 1), (n.x << 1) + (i & 1), (n.y << 1) + (i >> 1)}, leaves);
            }
            return;
        }
        leaves.insert(n);
    }

    // Vecinos con a lo sumo 1 nivel de diferencia (post-proceso de ATAK): una celda con un vecino 2+
    // niveles mas fino se subdivide una vez, y se repite de la mas fina a la mas gruesa. "Tiene un
    // vecino 2+ niveles mas fino" = alguno de los 8 cuadrados del nivel siguiente pegados a sus
    // lados es ancestro estricto de una hoja.
    void enforceNeighborLevels(std::set<Node>& leaves) {
        if (leaves.empty()) {
            return;
        }
        int maxZ = 0;
        std::set<Node> strictAncestors;
        for (const Node& n : leaves) {
            maxZ = std::max<int>(maxZ, n.z);
            for (int z = 0; z < n.z; ++z) {
                const int shift = n.z - z;
                strictAncestors.insert(Node{static_cast<uint8_t>(z), n.x >> shift, n.y >> shift});
            }
        }
        const auto exempt = [maxZ](const Node& n) {
            return maxZ - n.z >= kNeighborExemptLevels || n.z + 1 >= maxZ;
        };
        std::set<Node, FinerFirst> work;
        for (const Node& n : leaves) {
            if (!exempt(n)) {
                work.insert(n);
            }
        }
        while (!work.empty()) {
            const Node n = *work.begin();
            work.erase(work.begin());
            if (!leaves.contains(n)) {
                continue;
            }
            const auto z1 = static_cast<uint8_t>(n.z + 1);
            const int64_t x2 = n.x << 1;
            const int64_t y2 = n.y << 1;
            // En longitud el mundo se cierra: el vecino a traves del antimeridiano es la columna del otro borde.
            const auto wrapX = [count = tileCount(z1)](int64_t x) { return ((x % count) + count) % count; };
            const Node adjacent[8] = {{z1, wrapX(x2 - 1), y2}, {z1, wrapX(x2 - 1), y2 + 1}, {z1, wrapX(x2 + 2), y2},
                                      {z1, wrapX(x2 + 2), y2 + 1}, {z1, x2, y2 - 1}, {z1, x2 + 1, y2 - 1},
                                      {z1, x2, y2 + 2}, {z1, x2 + 1, y2 + 2}};
            const bool finerNeighbor = std::any_of(std::begin(adjacent), std::end(adjacent),
                                                   [&](const Node& a) { return strictAncestors.contains(a); });
            if (!finerNeighbor) {
                continue;
            }
            if (leaves.size() >= kCellFuse) {
                fuseTripped = true;
                return;
            }
            leaves.erase(n);
            strictAncestors.insert(n);
            for (int i = 0; i < 4; ++i) {
                const Node c{z1, x2 + (i & 1), y2 + (i >> 1)};
                const EcefDemRange range = rangeOf(c);
                if (!intersects(c, range, closestDistanceM(c, range))) {
                    continue;
                }
                leaves.insert(c);
                if (!exempt(c)) {
                    work.insert(c);
                }
            }
        }
    }

    EcefLodCell makeCell(const Node& n) const {
        EcefLodCell cell;
        cell.tile = canonicalOf(n);
        cell.wrap = wrapOf(n);
        cell.dem = finestDem(n);
        if (cell.dem) {
            const int shift = n.z - cell.dem->z;
            const float scale = 1.0f / static_cast<float>(int64_t{1} << shift);
            cell.demScale = scale;
            cell.demOffsetX = static_cast<float>(cell.tile.x - (cell.dem->x << shift)) * scale;
            cell.demOffsetY = static_cast<float>(cell.tile.y - (cell.dem->y << shift)) * scale;
        }
        cell.distanceM = closestDistanceM(n, rangeOf(n));
        return cell;
    }

    const EcefLodCamera& camera;
    const EcefDemIndex& dems;
    std::map<EcefLodTileKey, EcefDemRange> subtreeRanges;
    uint8_t maxDemZ = 0;
    bool fuseTripped = false;
};

} // namespace

EcefLodResult selectEcefTerrainCells(const EcefLodCamera& camera, const EcefDemIndex& dems) {
    return Selector(camera, dems).run();
}

} // namespace mln
