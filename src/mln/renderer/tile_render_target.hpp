#pragma once

#include <mln/renderer/render_target.hpp>
#include <mln/tile/tile_id.hpp>
#include <mln/util/mat4.hpp>

namespace mln {

class TileRenderTarget final : public RenderTarget {
public:
    TileRenderTarget(gfx::Context& context, Size size, gfx::TextureChannelDataType type, UnwrappedTileID tileID);
    ~TileRenderTarget() override;

    const UnwrappedTileID& getTileID() const { return tileID; }
    // La pagina se reutiliza para distintos tiles (MegaTexture); se puede desactivar para no
    // gastar un pase de render cuando no esta asignada a ningun tile visible.
    void setTileID(const UnwrappedTileID& id) { tileID = id; }
    void setActive(bool a) { active = a; }

    void upload(gfx::UploadPass&) override;
    void render(RenderOrchestrator&, const RenderTree&, PaintParameters&) override;

private:
    mat4 tileProjMatrix(const PaintParameters&) const;

    UnwrappedTileID tileID;
    bool active = true;
};

using TileRenderTargetPtr = std::shared_ptr<TileRenderTarget>;

} // namespace mln
