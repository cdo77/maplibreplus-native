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
    const double tileScale = static_cast<double>(1ull << tileID.canonical.z);
    const double tileSize = Projection::worldSize(state.getScale()) / tileScale;

    const double x0 = (static_cast<double>(tileID.canonical.x) + tileID.wrap * tileScale) * tileSize;
    const double y0 = static_cast<double>(tileID.canonical.y) * tileSize;

    mat4 matrix;
    matrix::ortho(matrix, x0, x0 + tileSize, y0 + tileSize, y0, -tileDepthRange, tileDepthRange);
    return matrix;
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

    parameters.currentLayer = 0;
    orchestrator.visitLayerGroups([&](LayerGroupBase& layerGroup) {
        layerGroup.runTweakers(renderTree, parameters);
        parameters.currentLayer++;
    });

    parameters.pass = RenderPass::Opaque;
    parameters.depthRangeSize = 1 - (layerGroupCount + 2) * PaintParameters::numSublayers *
                                        PaintParameters::depthEpsilon;
    parameters.currentLayer = 0;
    orchestrator.visitLayerGroupsReversed([&](LayerGroupBase& layerGroup) {
        layerGroup.render(orchestrator, parameters);
        parameters.currentLayer++;
    });

    parameters.pass = RenderPass::Translucent;
    parameters.depthRangeSize = 1 - (layerGroupCount + 2) * PaintParameters::numSublayers *
                                        PaintParameters::depthEpsilon;
    parameters.currentLayer = layerGroupCount > 0 ? static_cast<uint32_t>(layerGroupCount) - 1 : 0;
    orchestrator.visitLayerGroups([&](LayerGroupBase& layerGroup) {
        layerGroup.render(orchestrator, parameters);
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
