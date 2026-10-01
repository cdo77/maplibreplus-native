#include <mln/renderer/tile_render_target.hpp>

#include <mln/gfx/context.hpp>
#include <mln/gfx/offscreen_texture.hpp>
#include <mln/gfx/render_pass.hpp>
#include <mln/map/transform_state.hpp>
#include <mln/renderer/layer_group.hpp>
#include <mln/renderer/paint_parameters.hpp>
#include <mln/renderer/render_orchestrator.hpp>
#include <mln/renderer/render_tree.hpp>
#include <mln/util/constants.hpp>
#include <mln/util/projection.hpp>

#include <cmath>

namespace mln {

namespace {
constexpr double tileDepthRange = 1.0e6;
} // namespace

TileRenderTarget::TileRenderTarget(gfx::Context& context_,
                                   Size size,
                                   gfx::TextureChannelDataType type,
                                   UnwrappedTileID tileID_)
    : RenderTarget(context_, size, type),
      tileID(tileID_) {}

TileRenderTarget::~TileRenderTarget() = default;

void TileRenderTarget::upload(gfx::UploadPass&) {}

mat4 TileRenderTarget::tileProjMatrix(const PaintParameters& parameters) const {
    const auto& state = parameters.state;
    if (geographicArea) {
        // Drape compartido (ADR 0035): el area capturada es un circulo real (centro+radio en
        // metros) alrededor de una posicion geografica, no el area exacta de un tile del quadtree
        // mercator -- mismas world-tile-units que usa el resto del pipeline de capas
        // (Projection::project con la escala actual), para que renderTree/orchestrator dibujen
        // ahi sin cambios. Aproximacion ecuatorial de metros->world-units (igual que
        // omnidirectionalTileCover en tile_cover.cpp): suficiente para dimensionar un recorte
        // cuadrado, no para precision geodesica.
        const double worldSize = Projection::worldSize(state.getScale());
        const double worldUnitsPerMeter = worldSize / (2.0 * M_PI * util::EARTH_RADIUS_M);
        const Point<double> center = Projection::project(geographicArea->center, state.getScale());
        const double half = geographicArea->radiusMeters * worldUnitsPerMeter;

        mat4 matrix;
        matrix::ortho(
            matrix, center.x - half, center.x + half, center.y + half, center.y - half, -tileDepthRange, tileDepthRange);
        return matrix;
    }

    const double tileScale = static_cast<double>(1ull << tileID.canonical.z);
    const double tileSize = Projection::worldSize(state.getScale()) / tileScale;

    const double x0 = (static_cast<double>(tileID.canonical.x) + tileID.wrap * tileScale) * tileSize;
    const double y0 = static_cast<double>(tileID.canonical.y) * tileSize;

    mat4 matrix;
    matrix::ortho(matrix, x0, x0 + tileSize, y0 + tileSize, y0, -tileDepthRange, tileDepthRange);
    return matrix;
}

mat4 TileRenderTarget::worldToUVMatrix(const PaintParameters& parameters) const {
    // NDC [-1,1] -> UV [0,1]: mat[col*4+row], column-major (igual que el resto del motor).
    mat4 biasScale = matrix::identity4();
    biasScale[0] = 0.5;
    biasScale[5] = 0.5;
    biasScale[12] = 0.5;
    biasScale[13] = 0.5;

    mat4 result;
    matrix::multiply(result, biasScale, tileProjMatrix(parameters));
    return result;
}

void TileRenderTarget::render(RenderOrchestrator& orchestrator,
                              const RenderTree& renderTree,
                              PaintParameters& parameters) {
    if (!active) {
        return;
    }
    const mat4 projMatrix = tileProjMatrix(parameters);

    parameters.renderPass = parameters.encoder->createRenderPass("tile render target",
                                                                 {.renderable = *offscreenTexture,
                                                                  .clearColor = Color{0.0f, 0.0f, 0.0f, 0.0f},
                                                                  .clearDepth = {},
                                                                  .clearStencil = {}});
#if MLN_RENDER_BACKEND_OPENGL
    parameters.updateStencilBufferAvailability();
#endif

    const gfx::ScissorRect prevScissorRect = parameters.scissorRect;
    const auto& size = getTexture()->getSize();
    parameters.scissorRect = {.x = 0, .y = 0, .width = size.width, .height = size.height};

    const mat4* prevOverride = parameters.projMatrixOverride;
    parameters.projMatrixOverride = &projMatrix;

    parameters.context.bindGlobalUniformBuffers(*parameters.renderPass);

    const auto layerGroupCount = orchestrator.numLayerGroups();

    // Drape del terreno (area geografica): solo lo que va pegado a la superficie, como el pass
    // Surface de ATAK.
    //  - Sin `background` (ADR 0037): la captura queda TRANSPARENTE donde la imagen todavia no
    //    cargo, para que el shader del terreno muestre ahi el drape mas grueso.
    //  - Sin `symbol` (ADR 0038): los iconos van en el pase de sprites despues del terreno
    //    (renderer_impl), anclados al relieve y de tamano fijo; drapeados eran calcomanias gigantes.
    //    Tampoco se corren sus tweakers aca: pisarian los UBO que usa el pase de sprites.
    const bool isDrape = geographicArea.has_value();
    const auto skipLayerGroup = [&](const LayerGroupBase& layerGroup) {
        return isDrape && (orchestrator.layerGroupIsType(layerGroup, "background") ||
                           orchestrator.layerGroupIsType(layerGroup, "symbol"));
    };

    parameters.currentLayer = 0;
    orchestrator.visitLayerGroups([&](LayerGroupBase& layerGroup) {
        if (!skipLayerGroup(layerGroup)) {
            layerGroup.runTweakers(renderTree, parameters);
        }
        parameters.currentLayer++;
    });

    parameters.pass = RenderPass::Opaque;
    parameters.depthRangeSize = 1 - (layerGroupCount + 2) * PaintParameters::numSublayers *
                                        PaintParameters::depthEpsilon;
    parameters.currentLayer = 0;
    orchestrator.visitLayerGroupsReversed([&](LayerGroupBase& layerGroup) {
        if (!skipLayerGroup(layerGroup)) {
            layerGroup.render(orchestrator, parameters);
        }
        parameters.currentLayer++;
    });

    parameters.pass = RenderPass::Translucent;
    parameters.depthRangeSize = 1 - (layerGroupCount + 2) * PaintParameters::numSublayers *
                                        PaintParameters::depthEpsilon;
    parameters.currentLayer = layerGroupCount > 0 ? static_cast<uint32_t>(layerGroupCount) - 1 : 0;
    orchestrator.visitLayerGroups([&](LayerGroupBase& layerGroup) {
        if (!skipLayerGroup(layerGroup)) {
            layerGroup.render(orchestrator, parameters);
        }
        if (parameters.currentLayer > 0) {
            parameters.currentLayer--;
        }
    });

    parameters.context.unbindGlobalUniformBuffers(*parameters.renderPass);

    parameters.projMatrixOverride = prevOverride;

    parameters.renderPass.reset();
    parameters.encoder->present(*offscreenTexture);

    parameters.scissorRect = prevScissorRect;
}

} // namespace mln
