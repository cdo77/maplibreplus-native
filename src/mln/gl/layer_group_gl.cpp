#include <mln/util/render_stage.hpp>
#include <mln/gl/layer_group_gl.hpp>

#include <mln/gfx/drawable_tweaker.hpp>
#include <mln/gfx/render_pass.hpp>
#include <mln/gfx/renderable.hpp>
#include <mln/gfx/renderer_backend.hpp>
#include <mln/gfx/upload_pass.hpp>
#include <mln/gl/drawable_gl.hpp>
#include <mln/renderer/paint_parameters.hpp>
#include <mln/shaders/gl/shader_program_gl.hpp>
#include <mln/util/convert.hpp>
#include <mln/util/instrumentation.hpp>
#include <mln/util/surface_cull.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace mln {
namespace gl {

using namespace platform;

TileLayerGroupGL::TileLayerGroupGL(int32_t layerIndex_, std::size_t initialCapacity, std::string name_)
    : TileLayerGroup(layerIndex_, initialCapacity, std::move(name_)) {}

void TileLayerGroupGL::upload(gfx::UploadPass& uploadPass) {
    MLN_TRACE_FUNC();
    MLN_ZONE_STR(name);

    if (!enabled) {
        return;
    }

    visitDrawables([&](gfx::Drawable& drawable) {
        if (!drawable.getEnabled()) {
            return;
        }

        auto& drawableGL = static_cast<gl::DrawableGL&>(drawable);

#if !defined(NDEBUG)
        std::string label;
        if (const auto& tileID = drawable.getTileID()) {
            label = drawable.getName() + "/" + util::toString(*tileID);
        }
        const auto labelPtr = (label.empty() ? drawable.getName() : label).c_str();
        const auto debugGroup = uploadPass.createDebugGroup(labelPtr);
#endif

        drawableGL.upload(uploadPass);
    });
}

namespace {

// Huella de una tesela dentro de la celda que se esta renderizando, como rectangulo de scissor en pixeles (origen abajo a la izquierda; el norte es arriba). Equivale al recorte por
// stencil de cada tesela, sin el buffer de stencil: evita que el relleno translucido de teselas vecinas se pise en el margen del buffer de cada una (los "cuadrados").
gfx::ScissorRect tileScissorInCell(const util::SurfaceCull& cell, const OverscaledTileID& id, const gfx::ScissorRect& full) {
    if (id.wrap != cell.wrap) return {0, 0, 0, 0};
    const double s = std::ldexp(1.0, static_cast<int>(id.canonical.z) - static_cast<int>(cell.z));
    const double u0 = static_cast<double>(id.canonical.x) / s - static_cast<double>(cell.x);
    const double v0 = static_cast<double>(id.canonical.y) / s - static_cast<double>(cell.y);
    const double w = 1.0 / s;
    const double fw = static_cast<double>(full.width);
    const double fh = static_cast<double>(full.height);
    const auto clampPx = [](double v, double hi) { return std::min(std::max(v, 0.0), hi); };
    const double x0 = clampPx(std::floor(u0 * fw), fw);
    const double x1 = clampPx(std::ceil((u0 + w) * fw), fw);
    const double yTop = clampPx(std::floor(v0 * fh), fh);
    const double yBottom = clampPx(std::ceil((v0 + w) * fh), fh);
    if (x1 <= x0 || yBottom <= yTop) return {0, 0, 0, 0};
    return {full.x + static_cast<int32_t>(x0),
            full.y + static_cast<int32_t>(fh - yBottom),
            static_cast<uint32_t>(x1 - x0),
            static_cast<uint32_t>(yBottom - yTop)};
}

} // namespace

void TileLayerGroupGL::render(RenderOrchestrator&, PaintParameters& parameters) {
    if (!enabled) {
        return;
    }

    MLN_TRACE_FUNC();

    auto& context = static_cast<gl::Context&>(parameters.context);

    // `stencilModeFor3D` uses a different stencil mask value each time its called, so if the
    // drawables in this layer use 3D stencil mode, we need to set it up here so that all the
    // drawables end up using the same mode value.
    // 2D and 3D features in the same layer group is not supported.
    bool features3d = false;
    bool stencil3d = false;
    gfx::StencilMode stencilMode3d;

    parameters.stencilClippingAvailable = parameters.renderTargetHasStencilBuffer;

    if (getDrawableCount() && parameters.stencilClippingAvailable) {
        MLN_TRACE_ZONE(clip masks);
#if !defined(NDEBUG)
        const auto label_clip = getName() + (getName().empty() ? "" : "-") + "tile-clip-masks";
        const auto debugGroupClip = parameters.encoder->createDebugGroup(label_clip.c_str());
#endif

        // If we're using stencil clipping, we need to handle 3D features separately
        if (stencilTiles && !stencilTiles->empty()) {
            visitDrawables([&](const gfx::Drawable& drawable) {
                if (drawable.getEnabled() && drawable.getIs3D() && drawable.hasRenderPass(parameters.pass)) {
                    features3d = true;
                    if (drawable.getEnableStencil()) {
                        stencil3d = true;
                    }
                }
            });
        }

        // If we're doing 3D stenciling and have any features
        // to draw, set up the single-value stencil mask.
        // If we're doing 2D stenciling and have any drawables with tile IDs,
        // render each tile into the stencil buffer with a different value.
        if (features3d) {
            stencilMode3d = stencil3d ? parameters.stencilModeFor3D() : gfx::StencilMode::disabled();
        } else if (stencilTiles && !stencilTiles->empty()) {
            if (!parameters.renderTileClippingMasks(stencilTiles)) {
                parameters.stencilClippingAvailable = false;
            }
        }
    }

    if (!parameters.stencilClippingAvailable) {
        context.setStencilMode(gfx::StencilMode::disabled());
    }

#if !defined(NDEBUG)
    const auto label_render = getName() + (getName().empty() ? "" : "-") + "render";
    const auto debugGroupRender = parameters.encoder->createDebugGroup(label_render.c_str());
#endif

    // Dentro de una celda (fase P2), los grupos de vectores no se recortan por celda y retienen las teselas de TODOS los niveles que cubren la zona; dibujar el mismo poligono desde
    // dos o tres niveles suma la opacidad del relleno translucido (los "cuadrados" del circulo de precision, medido: el mismo relleno entraba desde z10, z16 y z18). Se dibuja
    // solo la tesela mas fina que cubre cada zona: se salta una tesela si otra del mismo grupo que se solapa con la celda es descendiente suya.
    util::RenderStage::set("capa: teselas en la celda");
    std::vector<OverscaledTileID> tilesInCell;
    const util::SurfaceCull* const cellClip = util::SurfaceCull::clip();
    if (cellClip && !util::SurfaceCull::current()) {
        visitDrawables([&](gfx::Drawable& drawable) {
            const auto& id = drawable.getTileID();
            if (id && drawable.getEnabled() && drawable.hasRenderPass(parameters.pass) && cellClip->intersects(*id)) tilesInCell.push_back(*id);
        });
    }
    const auto hasFinerTileInCell = [&](const OverscaledTileID& t) {
        for (const OverscaledTileID& u : tilesInCell) {
            if (u.wrap != t.wrap || u.canonical.z <= t.canonical.z) continue;
            const unsigned dz = static_cast<unsigned>(u.canonical.z - t.canonical.z);
            if ((static_cast<uint64_t>(u.canonical.x) >> dz) == t.canonical.x && (static_cast<uint64_t>(u.canonical.y) >> dz) == t.canonical.y) return true;
        }
        return false;
    };

    bool bindUBOs = false;
    visitDrawables([&](gfx::Drawable& drawable) {
        if (!drawable.getEnabled() || !drawable.hasRenderPass(parameters.pass)) {
            return;
        }
        if (!tilesInCell.empty() && drawable.getTileID() && hasFinerTileInCell(*drawable.getTileID())) return;

#if !defined(NDEBUG)
        std::string label_tile;
        if (const auto& tileID = drawable.getTileID()) {
            label_tile = util::toString(drawable.getID().id()) + "/" + drawable.getName() + "/" +
                         util::toString(*tileID);
        }
        const auto labelPtr = (label_tile.empty() ? drawable.getName() : label_tile).c_str();
        const auto debugGroupTile = parameters.encoder->createDebugGroup(labelPtr);
#endif

        if (!bindUBOs) {
            uniformBuffers.bind();
            bindUBOs = true;
        }

        for (const auto& tweaker : drawable.getTweakers()) {
            tweaker->execute(drawable, parameters);
        }

        // For layer groups with 3D features, enable either the single-value
        // stencil mode for features with stencil enabled or disable stenciling.
        // 2D drawables will set their own stencil mode within `draw`.
        if (features3d) {
            context.setStencilMode(drawable.getEnableStencil() ? stencilMode3d : gfx::StencilMode::disabled());
        }

        const gfx::ScissorRect fullScissor = parameters.scissorRect;
        if (const util::SurfaceCull* clip = util::SurfaceCull::clip(); clip && drawable.getTileID()) {
            parameters.scissorRect = tileScissorInCell(*clip, *drawable.getTileID(), fullScissor);
        }
        drawable.draw(parameters);
        parameters.scissorRect = fullScissor;
    });

    if (bindUBOs) {
        uniformBuffers.unbind();
    }
}

LayerGroupGL::LayerGroupGL(int32_t layerIndex_, std::size_t initialCapacity, std::string name_)
    : LayerGroup(layerIndex_, initialCapacity, std::move(name_)) {}

void LayerGroupGL::upload(gfx::UploadPass& uploadPass) {
    if (!enabled) {
        return;
    }

    visitDrawables([&](gfx::Drawable& drawable) {
        if (!drawable.getEnabled()) {
            return;
        }

        auto& drawableGL = static_cast<gl::DrawableGL&>(drawable);

#if !defined(NDEBUG)
        const auto debugGroup = uploadPass.createDebugGroup(drawable.getName().c_str());
#endif

        drawableGL.upload(uploadPass);
    });
}

void LayerGroupGL::render(RenderOrchestrator&, PaintParameters& parameters) {
    if (!enabled) {
        return;
    }

    auto& context = static_cast<gl::Context&>(parameters.context);
    parameters.stencilClippingAvailable = parameters.renderTargetHasStencilBuffer;
    if (!parameters.stencilClippingAvailable) {
        context.setStencilMode(gfx::StencilMode::disabled());
    }

    bool bindUBOs = false;
    visitDrawables([&](gfx::Drawable& drawable) {
        if (!drawable.getEnabled() || !drawable.hasRenderPass(parameters.pass)) {
            return;
        }

#if !defined(NDEBUG)
        const auto debugGroup = parameters.encoder->createDebugGroup(drawable.getName().c_str());
#endif
        if (!bindUBOs) {
            uniformBuffers.bind();
            bindUBOs = true;
        }

        for (const auto& tweaker : drawable.getTweakers()) {
            tweaker->execute(drawable, parameters);
        }

        drawable.draw(parameters);
    });

    if (bindUBOs) {
        uniformBuffers.unbind();
    }
}

} // namespace gl
} // namespace mln
