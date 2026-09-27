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
#include <mln/shaders/shader_program_base.hpp>
#include <mln/shaders/shader_defines.hpp>
#include <mln/shaders/terrain_layer_ubo.hpp>
#include <mln/style/types.hpp>
#include <mln/tile/raster_dem_tile.hpp>
#include <mln/tile/tile.hpp>
#include <mln/util/constants.hpp>
#include <mln/util/convert.hpp>
#include <mln/util/geo.hpp>
#include <mln/util/projection.hpp>
#include <mln/util/tile_coordinate.hpp>

#include <algorithm>
#include <optional>
#include <utility>
#include <vector>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>

namespace mln {

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
    return TerrainLayoutVertex{{{clampToShort(x), clampToShort(y), skirt}}};
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
    sharedIndices = std::make_shared<TerrainIndexVector>();

    const double delta = static_cast<double>(util::EXTENT) / terrainMeshSize;

    for (int32_t y = 0; y <= terrainMeshSize; y++) {
        for (int32_t x = 0; x <= terrainMeshSize; x++) {
            sharedVertices->emplace_back(terrainVertex(x * delta, y * delta, 0));
        }
    }

    const int32_t meshSize2 = terrainMeshSize * terrainMeshSize;
    for (int32_t y = 0; y < meshSize2; y += terrainMeshSize + 1) {
        for (int32_t x = 0; x < terrainMeshSize; x++) {
            sharedIndices->emplace_back(static_cast<uint16_t>(x + y),
                                        static_cast<uint16_t>(terrainMeshSize + x + y + 1),
                                        static_cast<uint16_t>(terrainMeshSize + x + y + 2));
            sharedIndices->emplace_back(static_cast<uint16_t>(x + y),
                                        static_cast<uint16_t>(terrainMeshSize + x + y + 2),
                                        static_cast<uint16_t>(x + y + 1));
        }
    }

    const auto offsetTop = static_cast<int32_t>(sharedVertices->elements());
    const int32_t offsetTopEdge = 0;
    const int32_t offsetBottom = offsetTop + (terrainMeshSize + 1);
    const int32_t offsetBottomEdge = (terrainMeshSize + 1) * terrainMeshSize;

    for (int32_t x = 0; x <= terrainMeshSize; x++) {
        sharedVertices->emplace_back(terrainVertex(x * delta, 0.0, 1));
    }
    for (int32_t x = 0; x <= terrainMeshSize; x++) {
        sharedVertices->emplace_back(terrainVertex(x * delta, util::EXTENT, 1));
    }
    for (int32_t x = 0; x < terrainMeshSize; x++) {
        sharedIndices->emplace_back(static_cast<uint16_t>(offsetBottomEdge + x),
                                    static_cast<uint16_t>(offsetBottom + x),
                                    static_cast<uint16_t>(offsetBottom + x + 1));
        sharedIndices->emplace_back(static_cast<uint16_t>(offsetBottomEdge + x),
                                    static_cast<uint16_t>(offsetBottom + x + 1),
                                    static_cast<uint16_t>(offsetBottomEdge + x + 1));
        sharedIndices->emplace_back(static_cast<uint16_t>(offsetTopEdge + x),
                                    static_cast<uint16_t>(offsetTop + x + 1),
                                    static_cast<uint16_t>(offsetTop + x));
        sharedIndices->emplace_back(static_cast<uint16_t>(offsetTopEdge + x),
                                    static_cast<uint16_t>(offsetTopEdge + x + 1),
                                    static_cast<uint16_t>(offsetTop + x + 1));
    }

    const auto offsetLeft = static_cast<int32_t>(sharedVertices->elements());
    const int32_t offsetRight = offsetLeft + (terrainMeshSize + 1) * 2;
    for (int32_t x = 0; x < 2; x++) {
        for (int32_t y = 0; y <= terrainMeshSize; y++) {
            for (int16_t z = 0; z < 2; z++) {
                sharedVertices->emplace_back(terrainVertex(x * util::EXTENT, y * delta, z));
            }
        }
    }
    for (int32_t y = 0; y < terrainMeshSize * 2; y += 2) {
        sharedIndices->emplace_back(static_cast<uint16_t>(offsetLeft + y),
                                    static_cast<uint16_t>(offsetLeft + y + 1),
                                    static_cast<uint16_t>(offsetLeft + y + 3));
        sharedIndices->emplace_back(static_cast<uint16_t>(offsetLeft + y),
                                    static_cast<uint16_t>(offsetLeft + y + 3),
                                    static_cast<uint16_t>(offsetLeft + y + 2));
        sharedIndices->emplace_back(static_cast<uint16_t>(offsetRight + y),
                                    static_cast<uint16_t>(offsetRight + y + 3),
                                    static_cast<uint16_t>(offsetRight + y + 1));
        sharedIndices->emplace_back(static_cast<uint16_t>(offsetRight + y),
                                    static_cast<uint16_t>(offsetRight + y + 2),
                                    static_cast<uint16_t>(offsetRight + y + 3));
    }

    segments.clear();
    segments.emplace_back(0, 0, sharedVertices->elements(), sharedIndices->elements());
}

void RenderTerrain::update(gfx::ShaderRegistry& shaders,
                           gfx::Context& context,
                           const TransformState& state,
                           RenderSource* demSource,
                           [[maybe_unused]] float pixelRatio,
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
    const float exaggeration = options.getExaggeration() * exaggerationFade;
    const auto eleDelta = static_cast<float>(getSkirtLength(state.getZoom()));

    // --- MegaTexture con LOD por cercania (ADR 0034) ---
    // Presupuesto de VRAM constante (kPageBudget paginas). El terreno se parte en sub-tiles y se
    // les da prioridad por tamano en pantalla: las paginas van primero a lo grande/cercano; si un
    // encuadre muy inclinado abarca mas superficie que el presupuesto, lo mas lejano (horizonte,
    // chico en pantalla) no se dibuja, pero el campo cercano NUNCA queda sin pagina -> sin negro
    // abajo ni freeze. Guiado por GLMegaTexture de ATAK.
    constexpr size_t kPageBudget = 24;
    constexpr uint32_t kPageSize = 1024;
    constexpr int kMaxSubdiv = 5;

    if (!pagesRegistered) {
        pages.reserve(kPageBudget);
        for (size_t i = 0; i < kPageBudget; ++i) {
            auto page = std::make_shared<TileRenderTarget>(
                context, Size{kPageSize, kPageSize}, gfx::TextureChannelDataType::UnsignedByte,
                UnwrappedTileID{0, 0, 0});
            page->setActive(false);
            changes.emplace_back(std::make_unique<AddRenderTargetRequest>(page));
            pages.push_back(std::move(page));
        }
        pagesRegistered = true;
    }

    mat4 projMatrix;
    state.getProjMatrix(projMatrix);
    const Size viewSize = state.getSize();

    // Prioridad de una sub-celda: <=0 si no se ve; si se ve, cuanto mas grande en pantalla (mas
    // cerca) mayor prioridad. Proyecta las 4 esquinas a nivel del mar con la matriz clip del
    // padre; una celda que cruza el plano cercano es campo cercano -> prioridad maxima.
    const auto cellPriority = [](const mat4& clip, double lx0, double ly0, double lx1, double ly1) -> double {
        constexpr double kMargin = 1.25;
        const double corners[4][2] = {{lx0, ly0}, {lx1, ly0}, {lx0, ly1}, {lx1, ly1}};
        double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
        int behind = 0;
        for (const auto& c : corners) {
            vec4 out;
            matrix::transformMat4(out, vec4{{c[0], c[1], 0.0, 1.0}}, clip);
            if (out[3] <= 1e-6) {
                ++behind;
                continue;
            }
            const double nx = out[0] / out[3];
            const double ny = out[1] / out[3];
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

    // Nivel de subdivision de un tile padre segun su tamano en pantalla (screen-space error).
    const auto parentSubdiv = [&](const mat4& clip) -> int {
        const double e = static_cast<double>(util::EXTENT);
        const double corners[4][2] = {{0.0, 0.0}, {e, 0.0}, {0.0, e}, {e, e}};
        double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
        int behind = 0;
        for (const auto& c : corners) {
            vec4 out;
            matrix::transformMat4(out, vec4{{c[0], c[1], 0.0, 1.0}}, clip);
            if (out[3] <= 1e-6) {
                ++behind;
                continue;
            }
            minx = std::min(minx, out[0] / out[3]);
            maxx = std::max(maxx, out[0] / out[3]);
            miny = std::min(miny, out[1] / out[3]);
            maxy = std::max(maxy, out[1] / out[3]);
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
    };
    struct ParentInfo {
        const RenderTile* tile;
        mat4 clip;
        int cz;
        int natS;
    };

    std::vector<ParentInfo> parents;
    std::set<OverscaledTileID> visibleParents;
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
        mat4 model, clip;
        state.matrixFor(model, pid.toUnwrapped());
        matrix::multiply(clip, projMatrix, model);
        parents.push_back(ParentInfo{&tile, clip, static_cast<int>(pid.canonical.z), parentSubdiv(clip)});
    }

    // Arma los sub-tiles visibles; baja un tope global de nivel hasta caber en el presupuesto.
    std::vector<SubTile> subTiles;
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
                    const double prio = cellPriority(p.clip, sx * cell, sy * cell, (sx + 1) * cell, (sy + 1) * cell);
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
        if (subTiles.size() <= kPageBudget) {
            break;
        }
    }

    // Prioriza lo cercano: si sobrepasa el presupuesto, se descarta lo mas lejano (horizonte).
    std::sort(subTiles.begin(), subTiles.end(),
              [](const SubTile& a, const SubTile& b) { return a.prio > b.prio; });
    if (subTiles.size() > kPageBudget) {
        subTiles.erase(subTiles.begin() + static_cast<std::ptrdiff_t>(kPageBudget), subTiles.end());
    }

    // Libera cachés de padres que ya no se ven.
    for (auto it = demTextures.begin(); it != demTextures.end();) {
        it = visibleParents.contains(it->first) ? std::next(it) : demTextures.erase(it);
    }
    for (auto it = demByTile.begin(); it != demByTile.end();) {
        it = visibleParents.contains(it->first) ? std::next(it) : demByTile.erase(it);
    }

    minElevation = 0;
    maxElevation = 0;

    // Reconstruye los drawables del terreno (pocos, <= presupuesto) asignando una pagina a cada
    // sub-tile en orden de prioridad. Reusa texturas DEM cacheadas (no re-sube el DEM cada frame).
    tileLayerGroup->clearDrawables();
    for (size_t i = 0; i < subTiles.size(); ++i) {
        const SubTile& st = subTiles[i];
        auto& page = pages[i];
        page->setTileID(st.id.toUnwrapped());
        page->setActive(true);

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

        auto vertexAttrs = context.createVertexAttributeArray();
        if (const auto& attr = vertexAttrs->set(idTerrainPosVertexAttribute)) {
            attr->setSharedRawData(sharedVertices,
                                   offsetof(TerrainLayoutVertex, a1),
                                   0,
                                   sizeof(TerrainLayoutVertex),
                                   gfx::AttributeDataType::Short3);
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
        builder->setRawVertices({}, sharedVertices->elements(), gfx::AttributeDataType::Short3);
        builder->setSegments(gfx::Triangles(), sharedIndices, segments.data(), segments.size());

        builder->setTexture(page->getTexture(), idTerrainImageTexture);
        builder->setTexture(demIt->second, idTerrainDemTexture);

        builder->flush(context);

        for (auto& drawable : builder->clearDrawables()) {
            drawable->setTileID(st.id);
            drawable->setData(std::make_unique<gfx::TerrainDrawableData>(
                dem.dim, dem.getUnpackVector(), exaggeration, eleDelta,
                st.demScale, st.demOffsetX, st.demOffsetY));
            tileLayerGroup->addDrawable(RenderPass::Opaque, st.id, std::move(drawable));
        }
    }
    for (size_t i = subTiles.size(); i < pages.size(); ++i) {
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
}

void RenderTerrain::teardown(UniqueChangeRequestVec& changes) {
    for (const auto& page : pages) {
        changes.emplace_back(std::make_unique<RemoveRenderTargetRequest>(page));
    }
    pages.clear();
    demTextures.clear();
    pagesRegistered = false;
    demByTile.clear();

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
    const auto centerElevation = static_cast<float>(
        getElevation(parameters.state.getLatLng(), parameters.state.getZoom())) * exaggerationFade;

    static_cast<TileLayerGroup*>(layerGroup.get())->visitDrawables([&](gfx::Drawable& drawable) {
        if (!drawable.getTileID() || !drawable.getData()) {
            return;
        }
        const auto& data = static_cast<const gfx::TerrainDrawableData&>(*drawable.getData());
        const UnwrappedTileID tileID = drawable.getTileID()->toUnwrapped();
        const auto matrix = LayerTweaker::getTileMatrix(
            tileID, parameters, {0.f, 0.f}, style::TranslateAnchorType::Map, false, false, drawable);

        // Mapea la posicion local del sub-tile [0,EXTENT] a la ventana del DEM del padre.
        mat4 terrainMatrix = matrix::identity4();
        const double demA = static_cast<double>(data.demScale) / util::EXTENT;
        matrix::scale(terrainMatrix, terrainMatrix, demA, demA, 0.0);
        terrainMatrix[12] = data.demOffsetX;
        terrainMatrix[13] = data.demOffsetY;

        const TerrainDrawableUBO drawableUBO = {.matrix = util::cast<float>(matrix),
                                                .terrain_matrix = util::cast<float>(terrainMatrix),
                                                .terrain_unpack = data.unpack,
                                                .terrain_dim = static_cast<float>(data.dim),
                                                .terrain_exaggeration = data.exaggeration,
                                                .ele_delta = data.eleDelta,
                                                .center_elevation = centerElevation};

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

    for (const auto& [tileID, demData] : demByTile) {
        const double tileScale = static_cast<double>(1ull << tileID.canonical.z);
        const double tileX = mercator.x * tileScale - tileID.canonical.x - tileID.wrap * tileScale;
        const double tileY = mercator.y * tileScale - tileID.canonical.y;
        if (tileX < 0.0 || tileX >= 1.0 || tileY < 0.0 || tileY >= 1.0) {
            continue;
        }

        const double px = tileX * demData->dim - 0.5;
        const double py = tileY * demData->dim - 0.5;
        const auto x0 = static_cast<int32_t>(std::floor(px));
        const auto y0 = static_cast<int32_t>(std::floor(py));
        const double fx = px - x0;
        const double fy = py - y0;

        const auto sample = [&](int32_t x, int32_t y) {
            return static_cast<double>(
                demData->get(std::clamp(x, -1, demData->dim), std::clamp(y, -1, demData->dim)));
        };

        const double top = sample(x0, y0) * (1.0 - fx) + sample(x0 + 1, y0) * fx;
        const double bottom = sample(x0, y0 + 1) * (1.0 - fx) + sample(x0 + 1, y0 + 1) * fx;
        return (top * (1.0 - fy) + bottom * fy) * options.getExaggeration();
    }

    (void)zoom;
    return 0.0;
}

} // namespace mln
