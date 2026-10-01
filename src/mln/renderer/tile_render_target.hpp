#pragma once

#include <mln/renderer/render_target.hpp>
#include <mln/tile/tile_id.hpp>
#include <mln/util/geo.hpp>
#include <mln/util/mat4.hpp>

#include <optional>

namespace mln {

// Area geografica real (centro + radio en metros) que un TileRenderTarget puede capturar en vez
// de un tile del quadtree mercator (ADR 0035): el terreno con camara real necesita "drapes"
// compartidos por proyeccion (varias celdas geometricas muestreando la MISMA textura), no una
// textura nueva por cada tile/celda -- ver RenderTerrain::updateDrapes.
struct GeographicArea {
    LatLng center;
    double radiusMeters;
};

class TileRenderTarget final : public RenderTarget {
public:
    TileRenderTarget(gfx::Context& context, Size size, gfx::TextureChannelDataType type, UnwrappedTileID tileID);
    ~TileRenderTarget() override;

    const UnwrappedTileID& getTileID() const { return tileID; }
    // La pagina se reutiliza para distintos tiles (MegaTexture); se puede desactivar para no
    // gastar un pase de render cuando no esta asignada a ningun tile visible.
    void setTileID(const UnwrappedTileID& id) {
        tileID = id;
        geographicArea.reset();
    }
    // Alternativa a setTileID (ADR 0035): captura un area geografica real (centro+radio en
    // metros) en vez del area exacta de un tile del quadtree mercator. Mientras este set, la
    // proyeccion de render() y worldToUVMatrix() se basan en el area, no en tileID.
    void setGeographicArea(const GeographicArea& area) { geographicArea = area; }
    void setActive(bool a) { active = a; }

    // Matriz que lleva coordenadas world-tile (mismas unidades que Projection::project con la
    // escala actual) a UV [0,1] de este render target -- para que otros renderables (el terreno)
    // proyecten sobre un drape compartido en vez de asumir "esta textura ES mi area exacta"
    // (ADR 0035).
    mat4 worldToUVMatrix(const PaintParameters&) const;

    void upload(gfx::UploadPass&) override;
    void render(RenderOrchestrator&, const RenderTree&, PaintParameters&) override;

private:
    mat4 tileProjMatrix(const PaintParameters&) const;

    UnwrappedTileID tileID;
    std::optional<GeographicArea> geographicArea;
    bool active = true;
};

using TileRenderTargetPtr = std::shared_ptr<TileRenderTarget>;

} // namespace mln
