#pragma once

#include <mln/renderer/change_request.hpp>
#include <mln/renderer/layer_group.hpp>
#include <mln/renderer/tile_render_target.hpp>
#include <mln/gfx/index_vector.hpp>
#include <mln/gfx/vertex_vector.hpp>
#include <mln/shaders/attributes.hpp>
#include <mln/shaders/segment.hpp>
#include <mln/util/geo.hpp>
#include <mln/style/terrain.hpp>
#include <mln/tile/tile_id.hpp>

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

using TerrainLayoutVertex = gfx::Vertex<TypeList<attributes::pos3d>>;
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

    static double getSkirtLength(double zoom);

private:
    void buildMesh();

    style::Terrain options;

    gfx::ShaderProgramBasePtr shader;
    LayerGroupBasePtr layerGroup;

    std::shared_ptr<TerrainVertexVector> sharedVertices;
    std::shared_ptr<TerrainIndexVector> sharedIndices;
    SegmentVector segments;

    // MegaTexture (ADR 0034): pool fijo de paginas (render targets reusables) con presupuesto
    // de VRAM constante y reuso, guiado por GLMegaTexture de ATAK. Reemplaza el esquema anterior
    // de un render target por sub-tile, que crecia sin techo y colgaba el GPU al acercar.
    std::vector<TileRenderTargetPtr> pages;                    // pool fijo de paginas (drape)
    std::map<OverscaledTileID, gfx::Texture2DPtr> demTextures; // cache de texturas DEM por tile padre
    bool pagesRegistered = false;
    float exaggerationFade = 1.0f;  // desvanece el relieve a zoom profundo (ruido del DEM)
    std::map<OverscaledTileID, std::shared_ptr<const DEMData>> demByTile;

    double minElevation = 0;
    double maxElevation = 0;
};

} // namespace mln
