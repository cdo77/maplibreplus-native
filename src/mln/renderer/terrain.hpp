#pragma once

#include <mln/map/transform_state.hpp>
#include <mln/renderer/change_request.hpp>
#include <mln/renderer/layer_group.hpp>
#include <mln/renderer/terrain_ecef_lod.hpp>
#include <mln/renderer/tile_render_target.hpp>
#include <mln/gfx/index_vector.hpp>
#include <mln/gfx/vertex_vector.hpp>
#include <mln/shaders/attributes.hpp>
#include <mln/shaders/segment.hpp>
#include <mln/util/geo.hpp>
#include <mln/style/terrain.hpp>
#include <mln/tile/tile_id.hpp>
#include <mln/util/ecef.hpp>
#include <mln/util/vectors.hpp>

#include <array>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace mln {

class DEMData;
class PaintParameters;
class RenderOrchestrator;
class RenderSource;
class TransformState;

namespace gfx {
class Context;
class ShaderRegistry;
class ShaderProgramBase;
class UploadPass;
class Texture2D;
using ShaderProgramBasePtr = std::shared_ptr<ShaderProgramBase>;
using Texture2DPtr = std::shared_ptr<Texture2D>;
} // namespace gfx

// pos3d: posicion tile-local [0,EXTENT] (muestreo del DEM y UV de los drapes). ecef_pos/ecef_normal:
// posicion ECEF real (metros) relativa al origen del sub-tile + normal elipsoidal (ADR 0034).
using TerrainLayoutVertex = gfx::Vertex<TypeList<attributes::pos3d, attributes::ecef_pos, attributes::ecef_normal>>;
using TerrainVertexVector = gfx::VertexVector<TerrainLayoutVertex>;
using TerrainIndexVector = gfx::IndexVector<gfx::Triangles>;

class RenderTerrain {
public:
    RenderTerrain();
    ~RenderTerrain();

    void setOptions(const style::Terrain&);
    const style::Terrain& getOptions() const { return options; }

    bool isRenderable() const;

    void update(gfx::ShaderRegistry&,
                gfx::Context&,
                const TransformState&,
                RenderSource* demSource,
                float pixelRatio,
                UniqueChangeRequestVec& changes);

    void teardown(UniqueChangeRequestVec& changes);

    void upload(gfx::UploadPass&);
    void updateUniforms(PaintParameters&);
    void render(RenderOrchestrator&, PaintParameters&);

    double getElevation(const LatLng&, double zoom) const;
    double getMinElevation() const { return minElevation; }
    double getMaxElevation() const { return maxElevation; }

    // Mapa de alturas del area del drape 32x (ADR 0038): ancla los iconos 3D al relieve, uno por
    // punto (el getTerrainMeshElevation de ATAK). RGBA8 codificado terrarium, como la DEM.
    const gfx::Texture2DPtr& getEcefHeightmap() const { return ecefHeightmap; }
    // Camara orbital del frame (ADR 0040), armada en update() con el relieve cargado (foco y suelo bajo el
    // ojo); nullopt sin terreno (el terreno existe solo con la camara real, ADR 0041).
    const std::optional<TransformState::EcefCamera>& getEcefCamera() const { return ecefCamera; }
    // x0, y0 y lado del area del mapa de alturas, en mercator normalizado [0,1].
    const std::array<double, 3>& getEcefHeightmapMercator() const { return ecefHeightmapMercator; }

private:
    void buildMesh();

    // Malla ECEF real de un sub-tile (RTE por-tile, ADR 0034): posicion+normal de la superficie
    // del elipsoide WGS84 para cada vertice de la grilla, relativas al centro geografico del
    // propio sub-tile (originOut, ECEF real en double). Se recomputa solo la primera vez que
    // aparece ese OverscaledTileID (cache); la elevacion se aplica despues, en el shader.
    std::shared_ptr<TerrainVertexVector> buildEcefMesh(const OverscaledTileID& id, vec3& originOut) const;

    // Casquetes polares: el mercator termina en +-85,0511 y el quadtree no tiene celdas mas alla, asi que el
    // globo quedaba hueco en los polos. Malla propia (abanico de anillos desde el polo hasta el borde del
    // mercator), fuera del sistema de celdas/DEM/drapes; se arma una vez y se dibuja de color plano.
    struct PolarCap {
        std::shared_ptr<TerrainVertexVector> vertices;
        vec3 origin{};
        std::array<float, 3> color{};
        OverscaledTileID id{0, 0, 0};
    };
    void buildPolarCaps();
    std::array<PolarCap, 2> polarCaps; // [0] sur, [1] norte
    std::shared_ptr<TerrainIndexVector> polarCapIndices;
    SegmentVector polarCapSegments;

    style::Terrain options;

    gfx::ShaderProgramBasePtr shader;
    LayerGroupBasePtr layerGroup;

    // Indices de la malla ECEF de cada celda (32x32, ADR 0039); los vertices son propios de cada celda.
    std::shared_ptr<TerrainIndexVector> ecefIndices;
    SegmentVector ecefSegments;

    std::map<OverscaledTileID, gfx::Texture2DPtr> demTextures; // cache de texturas DEM por tile padre
    gfx::Texture2DPtr flatDemTexture; // DEM de 1 texel en cero para las celdas planas (sin ningun tile DEM)

    // Drapes multi-resolucion (ADR 0037): los 3 render
    // passes offscreen de ATAK (GLMapView2.cpp, 1x/4x/32x de la resolucion base, todos centrados
    // en el punto de mira -- ver TransformState::computeEcefDrapeAreas). Cada uno captura el mapa
    // base una vez por frame como vista nadir sintetica, con su propio covering de tiles
    // (TilePyramid::update). El terreno los proyecta sobre su malla y el shader toma el mas fino
    // que tenga dato en cada punto -- mismo resultado que el dibujo de grueso a fino con corte
    // por alfa de ATAK (GLTerrainTile.cpp), en una sola pasada.
    std::array<TileRenderTargetPtr, TransformState::kEcefDrapeCount> drapes;
    bool drapesRegistered = false;
    // Cuantos drapes son el planisferio entero (vista de globo), contando desde el mas grueso: 0 a 3.
    float globalDrapes = 0.0f;

    // Mapa de alturas de los iconos 3D (ADR 0038): se rehace solo cuando cambia el area del drape
    // 32x (ya cuantizada) o el conjunto de tiles DEM cargados.
    void updateEcefHeightmap(gfx::Context&, const TransformState::EcefDrapeArea&);
    gfx::Texture2DPtr ecefHeightmap;
    std::array<double, 3> ecefHeightmapMercator{};
    std::vector<OverscaledTileID> ecefHeightmapDemKeys;

    std::map<OverscaledTileID, std::shared_ptr<const DEMData>> demByTile;
    // Rango de elevacion por tile DEM (caja del LOD ECEF, ADR 0039); se calcula una vez por tile.
    std::map<OverscaledTileID, EcefDemRange> demRangeByTile;
    std::optional<TransformState::EcefCamera> ecefCamera;

    // Cache de mallas ECEF por sub-tile (ADR 0034). Indexado por
    // OverscaledTileID igual que demByTile/demTextures: se libera cuando el sub-tile deja de
    // verse, se recomputa solo si aparece uno nuevo (evita trigonometria en double cada frame).
    std::map<OverscaledTileID, std::shared_ptr<TerrainVertexVector>> ecefVertexCache;
    std::map<OverscaledTileID, vec3> ecefOriginCache;

    double minElevation = 0;
    double maxElevation = 0;
};

} // namespace mln
