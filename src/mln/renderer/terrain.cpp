#include <mln/renderer/terrain.hpp>

#include <mln/geometry/dem_data.hpp>
#include <mln/gfx/context.hpp>
#include <mln/gfx/color_mode.hpp>
#include <mln/gfx/cull_face_mode.hpp>
#include <mln/gfx/depth_mode.hpp>
#include <mln/gfx/drawable.hpp>
#include <mln/gfx/drawable_builder.hpp>
#include <mln/gfx/shader_registry.hpp>
#include <mln/gfx/terrain_drawable_data.hpp>
#include <mln/gfx/texture2d.hpp>
#include <mln/gfx/upload_pass.hpp>
#include <mln/map/transform_state.hpp>
#include <mln/renderer/buckets/hillshade_bucket.hpp>
#include <mln/renderer/layer_tweaker.hpp>
#include <mln/renderer/paint_parameters.hpp>
#include <mln/renderer/render_source.hpp>
#include <mln/renderer/render_tile.hpp>
#include <mln/renderer/terrain_ecef_lod.hpp>
#include <mln/shaders/shader_program_base.hpp>
#include <mln/util/logging.hpp>
#include <mln/util/image.hpp>
#include <mln/math/angles.hpp>
#include <mln/shaders/shader_defines.hpp>
#include <mln/shaders/terrain_layer_ubo.hpp>
#include <mln/style/types.hpp>
#include <mln/tile/raster_dem_tile.hpp>
#include <mln/tile/tile.hpp>
#include <mln/util/constants.hpp>
#include <mln/util/convert.hpp>
#include <mln/util/geo.hpp>
#include <mln/util/globe.hpp>
#include <mln/util/projection.hpp>
#include <mln/util/tile_coordinate.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <optional>
#include <utility>
#include <vector>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>

namespace mln {

namespace {
// Elevacion cruda (metros reales) de un tile DEM en (tileX, tileY) en [0,1), bilineal. La usan
// getElevation (camara) y el mapa de alturas de los iconos 3D: cada llamador aplica la exageracion
// con la que dibuja.
double sampleDemBilinear(const DEMData& demData, double tileX, double tileY) {
    const double px = tileX * demData.dim - 0.5;
    const double py = tileY * demData.dim - 0.5;
    const auto x0 = static_cast<int32_t>(std::floor(px));
    const auto y0 = static_cast<int32_t>(std::floor(py));
    const double fx = px - x0;
    const double fy = py - y0;

    const auto sample = [&](int32_t x, int32_t y) {
        return static_cast<double>(demData.get(std::clamp(x, -1, demData.dim), std::clamp(y, -1, demData.dim)));
    };

    const double top = sample(x0, y0) * (1.0 - fx) + sample(x0 + 1, y0) * fx;
    const double bottom = sample(x0, y0 + 1) * (1.0 - fx) + sample(x0 + 1, y0 + 1) * fx;
    return top * (1.0 - fy) + bottom * fy;
}
} // namespace


using namespace shaders;

namespace {

constexpr int32_t terrainMeshSize = 128;
// El tamano del "drape" (mapa 2D proyectado sobre la malla del terreno) se calcula
// por tile en update(), segun el DPR del dispositivo y el overzoom del tile (ADR 0034).
constexpr auto terrainShaderGroupName = "TerrainShader";

int16_t clampToShort(double value) {
    return static_cast<int16_t>(std::clamp(value, -32768.0, 32767.0));
}

TerrainLayoutVertex terrainVertex(double x, double y, int16_t skirt) {
    // Malla compartida (modo planar mercator, de siempre): ecef_pos/ecef_normal en cero, el
    // shader las ignora fuera del modo camara con altura real (ADR 0034).
    return TerrainLayoutVertex{{{clampToShort(x), clampToShort(y), skirt}}, {{0.f, 0.f, 0.f}}, {{0.f, 0.f, 0.f}}};
}

// Recorre los vertices de una malla de meshSize x meshSize cuadros en el orden que esperan los
// indices de buildMeshIndices: grilla, skirt superior, skirt inferior y skirts izquierdo/derecho.
// El tercer argumento es 1 en el borde bajado del skirt.
template <typename Emit>
void forEachMeshVertex(int32_t meshSize, Emit&& emit) {
    const double delta = static_cast<double>(util::EXTENT) / meshSize;
    for (int32_t y = 0; y <= meshSize; y++) {
        for (int32_t x = 0; x <= meshSize; x++) {
            emit(x * delta, y * delta, int16_t{0});
        }
    }
    for (int32_t x = 0; x <= meshSize; x++) {
        emit(x * delta, 0.0, int16_t{1});
    }
    for (int32_t x = 0; x <= meshSize; x++) {
        emit(x * delta, static_cast<double>(util::EXTENT), int16_t{1});
    }
    for (int32_t x = 0; x < 2; x++) {
        for (int32_t y = 0; y <= meshSize; y++) {
            for (int16_t z = 0; z < 2; z++) {
                emit(static_cast<double>(x * util::EXTENT), y * delta, z);
            }
        }
    }
}

int32_t meshVertexCount(int32_t meshSize) {
    return (meshSize + 1) * (meshSize + 1) + 2 * (meshSize + 1) + 4 * (meshSize + 1);
}

std::shared_ptr<TerrainIndexVector> buildMeshIndices(int32_t meshSize) {
    auto indices = std::make_shared<TerrainIndexVector>();
    const int32_t meshSize2 = meshSize * meshSize;
    for (int32_t y = 0; y < meshSize2; y += meshSize + 1) {
        for (int32_t x = 0; x < meshSize; x++) {
            indices->emplace_back(static_cast<uint16_t>(x + y),
                                  static_cast<uint16_t>(meshSize + x + y + 1),
                                  static_cast<uint16_t>(meshSize + x + y + 2));
            indices->emplace_back(static_cast<uint16_t>(x + y),
                                  static_cast<uint16_t>(meshSize + x + y + 2),
                                  static_cast<uint16_t>(x + y + 1));
        }
    }

    const int32_t offsetTop = (meshSize + 1) * (meshSize + 1);
    const int32_t offsetTopEdge = 0;
    const int32_t offsetBottom = offsetTop + (meshSize + 1);
    const int32_t offsetBottomEdge = (meshSize + 1) * meshSize;
    for (int32_t x = 0; x < meshSize; x++) {
        indices->emplace_back(static_cast<uint16_t>(offsetBottomEdge + x),
                              static_cast<uint16_t>(offsetBottom + x),
                              static_cast<uint16_t>(offsetBottom + x + 1));
        indices->emplace_back(static_cast<uint16_t>(offsetBottomEdge + x),
                              static_cast<uint16_t>(offsetBottom + x + 1),
                              static_cast<uint16_t>(offsetBottomEdge + x + 1));
        indices->emplace_back(static_cast<uint16_t>(offsetTopEdge + x),
                              static_cast<uint16_t>(offsetTop + x + 1),
                              static_cast<uint16_t>(offsetTop + x));
        indices->emplace_back(static_cast<uint16_t>(offsetTopEdge + x),
                              static_cast<uint16_t>(offsetTopEdge + x + 1),
                              static_cast<uint16_t>(offsetTop + x + 1));
    }

    const int32_t offsetLeft = offsetTop + 2 * (meshSize + 1);
    const int32_t offsetRight = offsetLeft + (meshSize + 1) * 2;
    for (int32_t y = 0; y < meshSize * 2; y += 2) {
        indices->emplace_back(static_cast<uint16_t>(offsetLeft + y),
                              static_cast<uint16_t>(offsetLeft + y + 1),
                              static_cast<uint16_t>(offsetLeft + y + 3));
        indices->emplace_back(static_cast<uint16_t>(offsetLeft + y),
                              static_cast<uint16_t>(offsetLeft + y + 3),
                              static_cast<uint16_t>(offsetLeft + y + 2));
        indices->emplace_back(static_cast<uint16_t>(offsetRight + y),
                              static_cast<uint16_t>(offsetRight + y + 3),
                              static_cast<uint16_t>(offsetRight + y + 1));
        indices->emplace_back(static_cast<uint16_t>(offsetRight + y),
                              static_cast<uint16_t>(offsetRight + y + 2),
                              static_cast<uint16_t>(offsetRight + y + 3));
    }
    return indices;
}

// Rango de elevacion de un tile DEM (caja del LOD ECEF, ADR 0039): todos los texels, como la
// caja de ATAK sobre sus postes.
EcefDemRange demRangeOf(const DEMData& dem) {
    EcefDemRange range{std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
    for (int32_t y = 0; y < dem.dim; y++) {
        for (int32_t x = 0; x < dem.dim; x++) {
            const double elevation = dem.get(x, y);
            range.minM = std::min(range.minM, elevation);
            range.maxM = std::max(range.maxM, elevation);
        }
    }
    return range;
}

} // namespace

RenderTerrain::RenderTerrain() = default;

RenderTerrain::~RenderTerrain() = default;

void RenderTerrain::setOptions(const style::Terrain& options_) {
    options = options_;
}

bool RenderTerrain::isRenderable() const {
    return options.valid() && layerGroup && !layerGroup->empty();
}

double RenderTerrain::getSkirtLength(double zoom) {
    return 2.0 * M_PI * util::EARTH_RADIUS_M / std::pow(2.0, std::max(zoom, 0.0)) / 5.0;
}

void RenderTerrain::buildMesh() {
    if (sharedVertices) {
        return;
    }

    sharedVertices = std::make_shared<TerrainVertexVector>();
    forEachMeshVertex(terrainMeshSize, [&](double x, double y, int16_t skirt) {
        sharedVertices->emplace_back(terrainVertex(x, y, skirt));
    });
    sharedIndices = buildMeshIndices(terrainMeshSize);
    segments.clear();
    segments.emplace_back(0, 0, sharedVertices->elements(), sharedIndices->elements());

    // Malla de cada celda del terreno con camara de altura real (ADR 0039): 32x32 cuadros, como ATAK.
    ecefIndices = buildMeshIndices(kEcefTerrainMeshSize);
    ecefSegments.clear();
    ecefSegments.emplace_back(0, 0, static_cast<std::size_t>(meshVertexCount(kEcefTerrainMeshSize)),
                              ecefIndices->elements());
}

std::shared_ptr<TerrainVertexVector> RenderTerrain::buildEcefMesh(const OverscaledTileID& id, vec3& originOut) const {
    // Misma topologia y mismo orden de emision que buildMesh() (grid + skirts top/bottom +
    // skirts left/right) para poder reusar sharedIndices sin cambios -- solo cambia la posicion
    // de cada vertice: ECEF real (WGS84), relativa al centro geografico de ESTE sub-tile
    // (RTE por-tile, ADR 0034), en vez de coordenadas tile-local mercator.
    const auto& canonical = id.canonical;
    const double tileScale = static_cast<double>(1ull << canonical.z);

    const auto toLatLng = [&](double lx, double ly) {
        const double mercX = (canonical.x + lx / static_cast<double>(util::EXTENT)) * util::tileSize_D;
        const double mercY = (canonical.y + ly / static_cast<double>(util::EXTENT)) * util::tileSize_D;
        return Projection::unproject(Point<double>{mercX, mercY}, tileScale);
    };

    const LatLng centerLatLng = toLatLng(util::EXTENT * 0.5, util::EXTENT * 0.5);
    const vec3 origin = util::ecef::llaToEcef(centerLatLng, 0.0);
    originOut = origin;

    const auto vertexAt = [&](double lx, double ly, int16_t skirt) {
        const LatLng ll = toLatLng(lx, ly);
        const vec3 ecefAbs = util::ecef::llaToEcef(ll, 0.0);
        const vec3 normal = util::ecef::surfaceNormal(ll);
        return TerrainLayoutVertex{
            {{clampToShort(lx), clampToShort(ly), skirt}},
            {{static_cast<float>(ecefAbs[0] - origin[0]), static_cast<float>(ecefAbs[1] - origin[1]),
              static_cast<float>(ecefAbs[2] - origin[2])}},
            {{static_cast<float>(normal[0]), static_cast<float>(normal[1]), static_cast<float>(normal[2])}}};
    };

    auto vertices = std::make_shared<TerrainVertexVector>();
    forEachMeshVertex(kEcefTerrainMeshSize, [&](double lx, double ly, int16_t skirt) {
        vertices->emplace_back(vertexAt(lx, ly, skirt));
    });
    return vertices;
}

void RenderTerrain::update(gfx::ShaderRegistry& shaders,
                           gfx::Context& context,
                           const TransformState& state,
                           RenderSource* demSource,
                           float pixelRatio,
                           UniqueChangeRequestVec& changes) {
    if (!options.valid() || !demSource) {
        teardown(changes);
        return;
    }

    const auto renderTiles = demSource->getRenderTiles();
    if (!renderTiles || renderTiles->empty()) {
        teardown(changes);
        return;
    }

    if (!shader) {
        shader = context.getGenericShader(shaders, terrainShaderGroupName);
    }
    if (!shader) {
        teardown(changes);
        return;
    }

    buildMesh();

    if (!layerGroup) {
        auto layerGroup_ = context.createTileLayerGroup(
            std::numeric_limits<int32_t>::max(), /*initialCapacity=*/64, "terrain");
        if (!layerGroup_) {
            return;
        }
        layerGroup = std::move(layerGroup_);
    }

    auto* tileLayerGroup = static_cast<TileLayerGroup*>(layerGroup.get());

    // El DEM (terrarium z15) no tiene relieve util a nivel de calle; a zoom profundo su "relieve" es
    // ruido que descoloca la camara y la profundidad. Desvanecemos la exageracion al acercar:
    // relieve pleno a escala urbana, casi plano a nivel de calle (queda como 2D, sin negro).
    {
        constexpr double kFadeStart = 19.0, kFadeEnd = 23.0;
        exaggerationFade = static_cast<float>(
            std::clamp((kFadeEnd - state.getZoom()) / (kFadeEnd - kFadeStart), 0.2, 1.0));
    }
    // Camara con altura real: relieve REAL (1x), sin la exageracion de estilo ni el
    // desvanecimiento por zoom mercator -- la camara esta parada sobre el suelo a una altura real
    // y el relieve tiene que medir lo que mide (ATAK no exagera a nivel de suelo). En campo, con
    // la exageracion del estilo, las bardas se veian ~1.7x mas altas de lo real.
    const bool ecefMode = state.isRealAltitudeModeEnabled();
    const float exaggeration = ecefMode ? 1.0f : options.getExaggeration() * exaggerationFade;
    // Skirt: 500 m en modo camara real (el de ATAK, ADR 0039); en planar, el de MapLibre por zoom.
    const auto eleDelta = static_cast<float>(ecefMode ? kEcefTerrainSkirtM : getSkirtLength(state.getZoom()));

    // --- MegaTexture con LOD por cercania (ADR 0034), modo planar UNICAMENTE ---
    // Presupuesto de VRAM (paginas). El terreno se parte en sub-tiles y se les da prioridad por
    // cercania: las paginas van primero a lo grande/cercano; si un encuadre muy inclinado abarca
    // mas superficie que el presupuesto, lo mas lejano (horizonte, chico en pantalla) no se
    // dibuja, pero el campo cercano NUNCA queda sin pagina -> sin negro abajo ni freeze. Guiado
    // por GLMegaTexture de ATAK. El modo camara con altura real (ECEF) ya NO usa este esquema --
    // ver `drapes` mas abajo (ADR 0037).
    constexpr size_t kPageBudgetPlanar = 24;
    constexpr uint32_t kPageSize = 1024;
    constexpr int kMaxSubdiv = 5;

    if (!pagesRegistered) {
        pages.reserve(kPageBudgetPlanar);
        for (size_t i = 0; i < kPageBudgetPlanar; ++i) {
            auto page = std::make_shared<TileRenderTarget>(
                context, Size{kPageSize, kPageSize}, gfx::TextureChannelDataType::UnsignedByte,
                UnwrappedTileID{0, 0, 0});
            page->setActive(false);
            changes.emplace_back(std::make_unique<AddRenderTargetRequest>(page));
            pages.push_back(std::move(page));
        }
        pagesRegistered = true;
    }

    // --- Drapes multi-resolucion (ADR 0037), modo camara con altura real UNICAMENTE ---
    // Los 3 render passes offscreen de ATAK (1x/4x/32x), tamanos fijos
    // TransformState::kEcefDrapeTextureSizesPx -- los mismos que usa el TransformState sintetico
    // de cada covering independiente (TilePyramid::update). El terreno -- cuantas celdas
    // geometricas tenga -- proyecta su malla sobre estas texturas compartidas en vez de pedir
    // una pagina propia por celda (ver el loop de drawables y updateUniforms).
    if (!drapesRegistered) {
        for (size_t i = 0; i < drapes.size(); ++i) {
            const uint32_t sizePx = TransformState::kEcefDrapeTextureSizesPx[i];
            drapes[i] = std::make_shared<TileRenderTarget>(
                context, Size{sizePx, sizePx}, gfx::TextureChannelDataType::UnsignedByte, UnwrappedTileID{0, 0, 0});
            drapes[i]->setActive(false);
            changes.emplace_back(std::make_unique<AddRenderTargetRequest>(drapes[i]));
        }
        drapesRegistered = true;
    }

    mat4 projMatrix;
    state.getProjMatrix(projMatrix);
    const Size viewSize = state.getSize();

    // Drapes multi-resolucion (ADR 0037): se activan/reposicionan solo en modo ECEF, con las
    // mismas areas que TransformState::computeEcefDrapeAreas() -- la MISMA cuenta que usa
    // RenderOrchestrator::createRenderTree para el covering de tiles de cada drape, asi el area
    // que captura cada render target coincide con el area para la que su covering trajo imagen.
    // En modo planar quedan inactivos -- ese camino sigue con `pages` sin cambios.
    for (const auto& drape : drapes) {
        drape->setActive(ecefMode);
    }
    if (ecefMode) {
        const auto drapeAreas = state.computeEcefDrapeAreas();
        for (size_t i = 0; i < drapes.size(); ++i) {
            drapes[i]->setGeographicArea({.center = drapeAreas[i].center, .radiusMeters = drapeAreas[i].radiusMeters});
        }
    }

    // Proyecta un punto tile-local [0,EXTENT] (dentro del tile PADRE) a NDC -- solo para el modo
    // planar de siempre (mercator).
    using ProjectCornerFn = std::function<bool(double, double, double&, double&)>;

    // Prioridad de una sub-celda (modo planar, mercator de siempre): <=0 si no se ve; si se ve,
    // cuanto mas grande en pantalla (mas cerca) mayor prioridad. Una celda que cruza el plano
    // cercano es campo cercano -> prioridad maxima.
    const auto cellPriority = [](const ProjectCornerFn& project, double lx0, double ly0, double lx1,
                                 double ly1) -> double {
        constexpr double kMargin = 1.25;
        const double corners[4][2] = {{lx0, ly0}, {lx1, ly0}, {lx0, ly1}, {lx1, ly1}};
        double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
        int behind = 0;
        for (const auto& c : corners) {
            double nx, ny;
            if (!project(c[0], c[1], nx, ny)) {
                ++behind;
                continue;
            }
            minx = std::min(minx, nx);
            maxx = std::max(maxx, nx);
            miny = std::min(miny, ny);
            maxy = std::max(maxy, ny);
        }
        if (behind == 4) {
            return -1.0;
        }
        if (behind > 0) {
            return 1e12;  // cruza el plano cercano: campo cercano, maxima prioridad
        }
        if (!(maxx >= -kMargin && minx <= kMargin && maxy >= -kMargin && miny <= kMargin)) {
            return -1.0;  // fuera de la pantalla
        }
        return std::max(maxx - minx, maxy - miny);  // extension en pantalla ~ cercania
    };

    // Nivel de subdivision de un tile padre (modo planar) segun su tamano en pantalla.
    const auto parentSubdiv = [&](const ProjectCornerFn& project) -> int {
        const double e = static_cast<double>(util::EXTENT);
        const double corners[4][2] = {{0.0, 0.0}, {e, 0.0}, {0.0, e}, {e, e}};
        double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
        int behind = 0;
        for (const auto& c : corners) {
            double nx, ny;
            if (!project(c[0], c[1], nx, ny)) {
                ++behind;
                continue;
            }
            minx = std::min(minx, nx);
            maxx = std::max(maxx, nx);
            miny = std::min(miny, ny);
            maxy = std::max(maxy, ny);
        }
        if (behind > 0) {
            return kMaxSubdiv;
        }
        const double px = std::max((maxx - minx) * 0.5 * viewSize.width, (maxy - miny) * 0.5 * viewSize.height);
        constexpr double kTargetTilePx = 512.0;
        if (px <= kTargetTilePx) {
            return 0;
        }
        return std::clamp(static_cast<int>(std::lround(std::log2(px / kTargetTilePx))), 0, kMaxSubdiv);
    };

    struct SubTile {
        OverscaledTileID id;
        const RenderTile* parent;
        float demScale;
        float demOffsetX;
        float demOffsetY;
        double prio;
        bool flat = false;  // modo ECEF: celda sin DEM, se dibuja plana (ADR 0039)
    };
    struct ParentInfo {
        const RenderTile* tile;
        ProjectCornerFn project;
        int cz;
        int natS;
    };

    std::vector<ParentInfo> parents;
    std::set<OverscaledTileID> visibleParents;
    // Modo ECEF (ADR 0039): tiles DEM cargados con su rango de elevacion, para el LOD.
    EcefDemIndex ecefDemIndex;
    std::map<EcefLodTileKey, const RenderTile*> ecefDemTiles;
    for (const RenderTile& tile : *renderTiles) {
        const auto& pid = tile.getOverscaledTileID();
        const Tile& tileData = tile.getTile();
        if (!tileData.isRenderable()) {
            continue;
        }
        auto* bucket = static_cast<const RasterDEMTile&>(tileData).getBucket();
        if (!bucket || !bucket->hasData()) {
            continue;
        }
        visibleParents.insert(pid);
        const DEMData& dem = bucket->getDEMData();
        if (!demByTile.contains(pid)) {
            demByTile.emplace(pid, std::make_shared<const DEMData>(dem));
        }

        if (ecefMode) {
            auto rangeIt = demRangeByTile.find(pid);
            if (rangeIt == demRangeByTile.end()) {
                rangeIt = demRangeByTile.emplace(pid, demRangeOf(dem)).first;
            }
            const EcefLodTileKey key{pid.canonical.z, pid.canonical.x, pid.canonical.y};
            ecefDemIndex.emplace(key, rangeIt->second);
            ecefDemTiles.emplace(key, &tile);
            continue;
        }

        ParentInfo info;
        info.tile = &tile;
        info.cz = static_cast<int>(pid.canonical.z);
        mat4 model, clip;
        state.matrixFor(model, pid.toUnwrapped());
        matrix::multiply(clip, projMatrix, model);
        info.project = [clip](double lx, double ly, double& nx, double& ny) -> bool {
            vec4 out;
            matrix::transformMat4(out, vec4{{lx, ly, 0.0, 1.0}}, clip);
            if (out[3] <= 1e-6) {
                return false;
            }
            nx = out[0] / out[3];
            ny = out[1] / out[3];
            return true;
        };
        info.natS = parentSubdiv(info.project);
        parents.push_back(std::move(info));
    }

    // Arma los sub-tiles visibles.
    std::vector<SubTile> subTiles;
    if (ecefMode) {
        // LOD de la malla como el servicio de terreno de ATAK (ADR 0039, terrain_ecef_lod.hpp): quadtree
        // global por error de pantalla, frustum y far; relieve derivado del DEM cargado mas fino.
        // Camara orbital (ADR 0040): foco sobre el relieve y colision del ojo contra el terreno bajo el ojo.
        ecefCamera = state.computeEcefCamera(getElevation(state.getLatLng(), state.getZoom()),
                                             [this, &state](const LatLng& at) { return getElevation(at, state.getZoom()); });
        EcefLodCamera camera;
        camera.originEcef = ecefCamera->eyeEcef;
        const Point<double> cameraMercator = Projection::project(ecefCamera->eyeLatLng, 1.0) / util::tileSize_D;
        camera.mercatorX = cameraMercator.x;
        camera.mercatorY = cameraMercator.y;
        camera.farM = ecefCamera->farM;
        camera.lambda = (viewSize.height * static_cast<double>(pixelRatio) / 2.0) /
                        std::tan(TransformState::kEcefFieldOfViewRad / 2.0);
        camera.viewProjRte = state.getEcefTileMatrix(camera.originEcef, *ecefCamera);

        const EcefLodResult lod = selectEcefTerrainCells(camera, ecefDemIndex);
        const RenderTile* anyDemTile = ecefDemTiles.empty() ? nullptr : ecefDemTiles.begin()->second;
        size_t flatCells = 0;
        for (const EcefLodCell& cell : lod.cells) {
            const auto demIt = cell.dem ? ecefDemTiles.find(*cell.dem) : ecefDemTiles.end();
            const bool flat = demIt == ecefDemTiles.end();
            const RenderTile* demTile = flat ? anyDemTile : demIt->second;
            if (!demTile) {
                break;  // sin ningun DEM cargado no hay textura que enlazar; update() ya lo evita arriba
            }
            flatCells += flat ? 1 : 0;
            subTiles.push_back(SubTile{
                OverscaledTileID(cell.tile.z, cell.wrap, cell.tile.z, cell.tile.x, cell.tile.y),
                demTile, cell.demScale, cell.demOffsetX, cell.demOffsetY, -cell.distanceM, flat});
        }

        static auto lastLodLog = std::chrono::steady_clock::time_point{};
        const auto lodNow = std::chrono::steady_clock::now();
        if (lod.fuseTripped || lodNow - lastLodLog > std::chrono::seconds(1)) {
            lastLodLog = lodNow;
            // DIAGNOSTICO TEMPORAL (ADR 0039, quitar tras el field-test) + aviso del fusible (ese queda).
            int minZ = 99, maxZ = 0;
            for (const EcefLodCell& cell : lod.cells) {
                minZ = std::min<int>(minZ, cell.tile.z);
                maxZ = std::max<int>(maxZ, cell.tile.z);
            }
            const std::string nearest = lod.cells.empty()
                                            ? std::string("-")
                                            : std::to_string(lod.cells.front().distanceM) + "m z" +
                                                  std::to_string(lod.cells.front().tile.z);
            Log::Warning(Event::General,
                std::string("ECEF-LOD cells=") + std::to_string(lod.cells.size()) + " z=" + std::to_string(minZ) +
                    ".." + std::to_string(maxZ) + " nearest=" + nearest + " flat=" + std::to_string(flatCells) +
                    " dems=" + std::to_string(ecefDemIndex.size()) + " focusElevM=" +
                    std::to_string(ecefCamera->focusElevationM) + " rangeM=" + std::to_string(ecefCamera->rangeM) +
                    " eyeAglM=" + std::to_string(ecefCamera->eyeAglM) + " collided=" +
                    std::to_string(ecefCamera->collided) + " farM=" + std::to_string(ecefCamera->farM) +
                    (lod.fuseTripped ? " FUSIBLE: se corto el LOD" : ""));
        }
    } else {
        ecefCamera.reset();
        // Camino planar de siempre: baja un tope global de nivel hasta caber en el presupuesto.
        for (int sCap = kMaxSubdiv; sCap >= 0; --sCap) {
            subTiles.clear();
            for (const ParentInfo& p : parents) {
                const int s = std::min(p.natS, sCap);
                const uint32_t n = 1u << s;
                const auto& pid = p.tile->getOverscaledTileID();
                const auto childZ = static_cast<uint8_t>(p.cz + s);
                const int64_t baseX = static_cast<int64_t>(pid.canonical.x) * n;
                const int64_t baseY = static_cast<int64_t>(pid.canonical.y) * n;
                const float invN = 1.0f / static_cast<float>(n);
                const double cell = static_cast<double>(util::EXTENT) / n;
                for (uint32_t sy = 0; sy < n; ++sy) {
                    for (uint32_t sx = 0; sx < n; ++sx) {
                        const double prio = cellPriority(p.project, sx * cell, sy * cell, (sx + 1) * cell,
                                                         (sy + 1) * cell);
                        if (prio <= 0.0) {
                            continue;
                        }
                        subTiles.push_back(SubTile{
                            OverscaledTileID(childZ, pid.wrap, childZ,
                                             static_cast<uint32_t>(baseX + sx), static_cast<uint32_t>(baseY + sy)),
                            p.tile, invN, static_cast<float>(sx) * invN, static_cast<float>(sy) * invN, prio});
                    }
                }
            }
            if (subTiles.size() <= kPageBudgetPlanar) {
                break;
            }
        }
    }

    // Ordena de lo cercano a lo lejano. En modo planar, si sobrepasa el tope de paginas
    // (kPageBudgetPlanar) se descarta lo mas lejano; en modo ECEF no hay tope: el LOD ya acota las
    // celdas como ATAK (ADR 0039).
    std::sort(subTiles.begin(), subTiles.end(),
              [](const SubTile& a, const SubTile& b) { return a.prio > b.prio; });
    if (!ecefMode && subTiles.size() > kPageBudgetPlanar) {
        subTiles.erase(subTiles.begin() + static_cast<std::ptrdiff_t>(kPageBudgetPlanar), subTiles.end());
    }

    // Libera cachés de padres que ya no se ven.
    for (auto it = demTextures.begin(); it != demTextures.end();) {
        it = visibleParents.contains(it->first) ? std::next(it) : demTextures.erase(it);
    }
    for (auto it = demByTile.begin(); it != demByTile.end();) {
        it = visibleParents.contains(it->first) ? std::next(it) : demByTile.erase(it);
    }
    for (auto it = demRangeByTile.begin(); it != demRangeByTile.end();) {
        it = visibleParents.contains(it->first) ? std::next(it) : demRangeByTile.erase(it);
    }

    // Camara con altura real (ADR 0034): el terreno pasa a vivir en ECEF real (elipsoide WGS84,
    // RTE por sub-tile) en vez de la proyeccion mercator plana de siempre. Interruptor apagado
    // por defecto (invariante 9) -- con el modo apagado nada de este bloque cambia el
    // comportamiento existente. `ecefMode` ya se calculo mas arriba, junto al armado de `project`.
    if (ecefMode) {
        const std::set<OverscaledTileID> visibleSubTiles = [&] {
            std::set<OverscaledTileID> s;
            for (const SubTile& st : subTiles) s.insert(st.id);
            return s;
        }();
        for (auto it = ecefVertexCache.begin(); it != ecefVertexCache.end();) {
            it = visibleSubTiles.contains(it->first) ? std::next(it) : ecefVertexCache.erase(it);
        }
        for (auto it = ecefOriginCache.begin(); it != ecefOriginCache.end();) {
            it = visibleSubTiles.contains(it->first) ? std::next(it) : ecefOriginCache.erase(it);
        }
    } else if (!ecefVertexCache.empty()) {
        ecefVertexCache.clear();
        ecefOriginCache.clear();
    }

    minElevation = 0;
    maxElevation = 0;

    // Reconstruye los drawables del terreno (pocos, <= presupuesto) en orden de prioridad. Reusa
    // texturas DEM cacheadas (no re-sube el DEM cada frame). Modo planar: cada sub-tile se lleva
    // su propia pagina (MegaTexture, ADR 0034). Modo ECEF: todos los sub-tiles comparten los
    // mismos drapes (ADR 0037) -- no hay "pagina por indice" que asignar.
    tileLayerGroup->clearDrawables();
    for (size_t i = 0; i < subTiles.size(); ++i) {
        const SubTile& st = subTiles[i];
        TileRenderTargetPtr page;
        if (!ecefMode) {
            page = pages[i];
            page->setTileID(st.id.toUnwrapped());
            page->setActive(true);
        }

        const auto& parentId = st.parent->getOverscaledTileID();
        auto* bucket = static_cast<const RasterDEMTile&>(st.parent->getTile()).getBucket();
        const DEMData& dem = bucket->getDEMData();

        auto demIt = demTextures.find(parentId);
        if (demIt == demTextures.end()) {
            auto demTexture = context.createTexture2D();
            demTexture->setImage(dem.getImagePtr());
            demTexture->setSamplerConfiguration({.filter = gfx::TextureFilterType::Nearest,
                                                 .wrapU = gfx::TextureWrapType::Clamp,
                                                 .wrapV = gfx::TextureWrapType::Clamp});
            demIt = demTextures.emplace(parentId, std::move(demTexture)).first;
        }

        // Malla: compartida (planar, de siempre) o propia del sub-tile (ECEF real, cacheada por
        // OverscaledTileID -- se recalcula solo la primera vez que aparece ese sub-tile).
        std::shared_ptr<TerrainVertexVector> meshVertices = sharedVertices;
        std::optional<vec3> ecefOrigin;
        if (ecefMode) {
            auto meshIt = ecefVertexCache.find(st.id);
            if (meshIt == ecefVertexCache.end()) {
                vec3 origin;
                auto built = buildEcefMesh(st.id, origin);
                meshIt = ecefVertexCache.emplace(st.id, std::move(built)).first;
                ecefOriginCache.emplace(st.id, origin);
            }
            meshVertices = meshIt->second;
            ecefOrigin = ecefOriginCache.at(st.id);
        }

        auto vertexAttrs = context.createVertexAttributeArray();
        if (const auto& attr = vertexAttrs->set(idTerrainPosVertexAttribute)) {
            attr->setSharedRawData(meshVertices,
                                   offsetof(TerrainLayoutVertex, a1),
                                   0,
                                   sizeof(TerrainLayoutVertex),
                                   gfx::AttributeDataType::Short3);
        }
        if (const auto& attr = vertexAttrs->set(idTerrainEcefPosVertexAttribute)) {
            attr->setSharedRawData(meshVertices,
                                   offsetof(TerrainLayoutVertex, a2),
                                   0,
                                   sizeof(TerrainLayoutVertex),
                                   gfx::AttributeDataType::Float3);
        }
        if (const auto& attr = vertexAttrs->set(idTerrainEcefNormalVertexAttribute)) {
            attr->setSharedRawData(meshVertices,
                                   offsetof(TerrainLayoutVertex, a3),
                                   0,
                                   sizeof(TerrainLayoutVertex),
                                   gfx::AttributeDataType::Float3);
        }

        auto builder = context.createDrawableBuilder("terrain");
        builder->setShader(shader);
        builder->setIs3D(true);
        builder->setEnableDepth(true);
        builder->setDepthType(gfx::DepthMaskType::ReadWrite);
        builder->setColorMode(gfx::ColorMode::unblended());
        builder->setCullFaceMode(gfx::CullFaceMode::disabled());
        builder->setRenderPass(RenderPass::Opaque);
        builder->setVertexAttributes(std::move(vertexAttrs));
        builder->setRawVertices({}, meshVertices->elements(), gfx::AttributeDataType::Short3);
        if (ecefMode) {
            builder->setSegments(gfx::Triangles(), ecefIndices, ecefSegments.data(), ecefSegments.size());
        } else {
            builder->setSegments(gfx::Triangles(), sharedIndices, segments.data(), segments.size());
        }

        // Drapes multi-resolucion (ADR 0037): en modo ECEF, TODOS los sub-tiles samplean los
        // mismos render targets (drapes) en vez de una pagina propia -- los slots que un camino
        // no usa igual se bindean (a la propia DEM, valida y ya subida) para no dejar unidades de
        // textura sin asignar.
        builder->setTexture(ecefMode ? demIt->second : page->getTexture(), idTerrainImageTexture);
        builder->setTexture(demIt->second, idTerrainDemTexture);
        for (size_t d = 0; d < drapes.size(); ++d) {
            builder->setTexture(ecefMode ? drapes[d]->getTexture() : demIt->second,
                                static_cast<size_t>(idTerrainDrape0Texture) + d);
        }

        builder->flush(context);

        for (auto& drawable : builder->clearDrawables()) {
            drawable->setTileID(st.id);
            drawable->setData(std::make_unique<gfx::TerrainDrawableData>(
                dem.dim, dem.getUnpackVector(), st.flat ? 0.0f : exaggeration, eleDelta,
                st.demScale, st.demOffsetX, st.demOffsetY, ecefOrigin));
            tileLayerGroup->addDrawable(RenderPass::Opaque, st.id, std::move(drawable));
        }
    }
    // En modo ECEF ninguna pagina se usa este frame (drapes compartidos, ADR 0035) -- desactivar
    // todas, no solo desde subTiles.size(), para no dejar paginas encendidas de un frame anterior
    // en modo planar.
    for (size_t i = ecefMode ? 0 : subTiles.size(); i < pages.size(); ++i) {
        pages[i]->setActive(false);
    }

    for (const auto& [tileID, demData] : demByTile) {
        for (int32_t y = 0; y < demData->dim; y += 8) {
            for (int32_t x = 0; x < demData->dim; x += 8) {
                const double elevation = demData->get(x, y) * exaggeration;
                minElevation = std::min(minElevation, elevation);
                maxElevation = std::max(maxElevation, elevation);
            }
        }
    }

    if (ecefMode) {
        updateEcefHeightmap(context, state.computeEcefDrapeAreas().back());
    }
}

void RenderTerrain::updateEcefHeightmap(gfx::Context& context, const TransformState::EcefDrapeArea& area) {
    std::vector<OverscaledTileID> demKeys;
    demKeys.reserve(demByTile.size());
    for (const auto& entry : demByTile) {
        demKeys.push_back(entry.first);
    }

    const double latRad = util::deg2rad(area.center.latitude());
    const Point<double> center = Projection::project(area.center, 1.0) / util::tileSize_D;
    const double span = 2.0 * area.radiusMeters / (util::M2PI * util::EARTH_RADIUS_M * std::cos(latRad));
    const std::array<double, 3> bounds = {center.x - span / 2.0, center.y - span / 2.0, span};
    if (ecefHeightmap && bounds == ecefHeightmapMercator && demKeys == ecefHeightmapDemKeys) {
        return;
    }

    // ~17 m por texel sobre el area 32x (~17 km). Filas de norte a sur, igual que el mercator: el
    // shader de simbolos lee la fila (y - y0) / lado sin invertir.
    constexpr uint32_t kSize = 1024;
    auto image = std::make_shared<PremultipliedImage>(Size{kSize, kSize});
    const auto encode = [&](uint32_t i, uint32_t j, double elevation) {
        const double v = std::clamp(elevation + 32768.0, 0.0, 65535.99);
        uint8_t* texel = image->data.get() + ((static_cast<size_t>(j) * kSize) + i) * 4;
        texel[0] = static_cast<uint8_t>(v / 256.0);
        texel[1] = static_cast<uint8_t>(std::fmod(v, 256.0));
        texel[2] = static_cast<uint8_t>((v - std::floor(v)) * 256.0);
        texel[3] = 255;
    };
    for (uint32_t j = 0; j < kSize; ++j) {
        for (uint32_t i = 0; i < kSize; ++i) {
            encode(i, j, 0.0);
        }
    }

    // demByTile recorre de menor a mayor zoom: los tiles finos pisan a los gruesos donde se solapan.
    for (const auto& [tileID, dem] : demByTile) {
        const double tiles = std::pow(2.0, tileID.canonical.z);
        const double tx0 = (tileID.canonical.x + (tileID.wrap * tiles)) / tiles;
        const double ty0 = tileID.canonical.y / tiles;
        const double tileSpan = 1.0 / tiles;
        const auto texelRange = [&](double tileStart, double boundsStart, int64_t& first, int64_t& last) {
            first = static_cast<int64_t>(std::max(0.0, std::floor((tileStart - boundsStart) / span * kSize)));
            last = static_cast<int64_t>(
                std::min<double>(kSize, std::ceil((tileStart + tileSpan - boundsStart) / span * kSize)));
        };
        int64_t i0 = 0;
        int64_t i1 = 0;
        int64_t j0 = 0;
        int64_t j1 = 0;
        texelRange(tx0, bounds[0], i0, i1);
        texelRange(ty0, bounds[1], j0, j1);
        for (int64_t j = j0; j < j1; ++j) {
            const double ty = (bounds[1] + ((static_cast<double>(j) + 0.5) / kSize * span) - ty0) / tileSpan;
            if (ty < 0.0 || ty >= 1.0) {
                continue;
            }
            for (int64_t i = i0; i < i1; ++i) {
                const double tx = (bounds[0] + ((static_cast<double>(i) + 0.5) / kSize * span) - tx0) / tileSpan;
                if (tx < 0.0 || tx >= 1.0) {
                    continue;
                }
                encode(static_cast<uint32_t>(i), static_cast<uint32_t>(j), sampleDemBilinear(*dem, tx, ty));
            }
        }
    }

    if (!ecefHeightmap) {
        ecefHeightmap = context.createTexture2D();
        ecefHeightmap->setSamplerConfiguration({.filter = gfx::TextureFilterType::Nearest,
                                                .wrapU = gfx::TextureWrapType::Clamp,
                                                .wrapV = gfx::TextureWrapType::Clamp});
    }
    ecefHeightmap->setImage(std::move(image));
    ecefHeightmapMercator = bounds;
    ecefHeightmapDemKeys = std::move(demKeys);
}

void RenderTerrain::teardown(UniqueChangeRequestVec& changes) {
    for (const auto& page : pages) {
        changes.emplace_back(std::make_unique<RemoveRenderTargetRequest>(page));
    }
    pages.clear();
    for (auto& drape : drapes) {
        if (drape) {
            changes.emplace_back(std::make_unique<RemoveRenderTargetRequest>(drape));
            drape.reset();
        }
    }
    drapesRegistered = false;
    ecefHeightmap.reset();
    ecefHeightmapDemKeys.clear();
    demTextures.clear();
    pagesRegistered = false;
    demByTile.clear();
    ecefVertexCache.clear();
    ecefOriginCache.clear();

    if (layerGroup) {
        layerGroup->clearDrawables();
    }
}

void RenderTerrain::upload(gfx::UploadPass& uploadPass) {
    if (layerGroup) {
        layerGroup->upload(uploadPass);
    }
}

void RenderTerrain::updateUniforms(PaintParameters& parameters) {
    if (!layerGroup) {
        return;
    }

    // Elevacion del centro de la vista: el terreno se dibuja relativo a ella para que la camara
    // nunca quede por debajo del relieve al acercar (si no, a zoom profundo se veia todo negro).
    const double demElevationAtCenter = getElevation(parameters.state.getLatLng(), parameters.state.getZoom());
    const auto centerElevation =
        static_cast<float>(demElevationAtCenter * options.getExaggeration()) * exaggerationFade;

    static_cast<TileLayerGroup*>(layerGroup.get())->visitDrawables([&](gfx::Drawable& drawable) {
        if (!drawable.getTileID() || !drawable.getData()) {
            return;
        }
        const auto& data = static_cast<const gfx::TerrainDrawableData&>(*drawable.getData());
        const UnwrappedTileID tileID = drawable.getTileID()->toUnwrapped();
        // Camara con altura real (ADR 0034): matriz RTE en ECEF real en vez de la mercator de
        // siempre -- se recalcula cada frame (la camara se mueve), pero sin tocar la malla. La camara
        // es UNA para todas las celdas: el suelo bajo ella es el relieve real (1x), aun en celdas planas.
        const auto matrix = (data.ecefOrigin && ecefCamera)
                                ? parameters.state.getEcefTileMatrix(*data.ecefOrigin, *ecefCamera)
                                : LayerTweaker::getTileMatrix(tileID,
                                                              parameters,
                                                              {0.f, 0.f},
                                                              style::TranslateAnchorType::Map,
                                                              false,
                                                              false,
                                                              drawable);

        // Mapea la posicion local del sub-tile [0,EXTENT] a la ventana del DEM del padre.
        mat4 terrainMatrix = matrix::identity4();
        const double demA = static_cast<double>(data.demScale) / util::EXTENT;
        matrix::scale(terrainMatrix, terrainMatrix, demA, demA, 0.0);
        terrainMatrix[12] = data.demOffsetX;
        terrainMatrix[13] = data.demOffsetY;

        // Drapes multi-resolucion (ADR 0037, solo en modo ECEF): tile-local [0,EXTENT] ->
        // world-tile -> UV de cada drape. tile-local a world-tile es la MISMA transformacion que ya
        // usa el motor para posicionar cualquier tile mercator (matrixFor).
        std::array<mat4, TransformState::kEcefDrapeCount> drapeMatrices;
        drapeMatrices.fill(matrix::identity4());
        if (data.ecefOrigin) {
            mat4 tileToWorld;
            parameters.state.matrixFor(tileToWorld, tileID);
            for (size_t i = 0; i < drapes.size(); ++i) {
                matrix::multiply(drapeMatrices[i], drapes[i]->worldToUVMatrix(parameters), tileToWorld);
            }
        }

        const TerrainDrawableUBO drawableUBO = {.matrix = util::cast<float>(matrix),
                                                .terrain_matrix = util::cast<float>(terrainMatrix),
                                                .drape_matrix0 = util::cast<float>(drapeMatrices[0]),
                                                .drape_matrix1 = util::cast<float>(drapeMatrices[1]),
                                                .drape_matrix2 = util::cast<float>(drapeMatrices[2]),
                                                .terrain_unpack = data.unpack,
                                                .terrain_dim = static_cast<float>(data.dim),
                                                .terrain_exaggeration = data.exaggeration,
                                                .ele_delta = data.eleDelta,
                                                .center_elevation = centerElevation,
                                                .ecef_mode = data.ecefOrigin ? 1.0f : 0.0f,
                                                .pad0 = 0.0f,
                                                .pad1 = 0.0f,
                                                .pad2 = 0.0f};

        drawable.mutableUniformBuffers().createOrUpdate(idTerrainDrawableUBO, &drawableUBO, parameters.context);
    });
}

void RenderTerrain::render(RenderOrchestrator& orchestrator, PaintParameters& parameters) {
    if (layerGroup) {
        layerGroup->render(orchestrator, parameters);
    }
}

double RenderTerrain::getElevation(const LatLng& latLng, double zoom) const {
    if (demByTile.empty()) {
        return 0.0;
    }

    const Point<double> mercator = Projection::project(latLng, 1.0) / util::tileSize_D;

    // El tile DEM MAS FINO que contiene el punto -- el mismo relieve que dibuja la malla ahi
    // (getTerrainMeshElevation en ATAK). demByTile recorre de menor a mayor zoom: tomar el primero
    // daba el mas grueso (z10, ~150 m por texel mezclando barda y ciudad) y la camara quedaba
    // decenas de metros arriba o debajo del suelo dibujado.
    const DEMData* bestDem = nullptr;
    double bestTileX = 0.0;
    double bestTileY = 0.0;
    uint8_t bestZ = 0;
    for (const auto& [tileID, dem] : demByTile) {
        const double tileScale = static_cast<double>(1ull << tileID.canonical.z);
        const double tx = mercator.x * tileScale - tileID.canonical.x - tileID.wrap * tileScale;
        const double ty = mercator.y * tileScale - tileID.canonical.y;
        if (tx < 0.0 || tx >= 1.0 || ty < 0.0 || ty >= 1.0) {
            continue;
        }
        if (!bestDem || tileID.canonical.z > bestZ) {
            bestDem = dem.get();
            bestTileX = tx;
            bestTileY = ty;
            bestZ = tileID.canonical.z;
        }
    }

    if (bestDem) {
        return sampleDemBilinear(*bestDem, bestTileX, bestTileY);
    }

    (void)zoom;
    return 0.0;
}

} // namespace mln
