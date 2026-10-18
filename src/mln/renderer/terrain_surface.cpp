#include <mln/renderer/terrain.hpp>

#include <mln/gfx/context.hpp>
#include <mln/util/logging.hpp>
#include <mln/util/surface_cull.hpp>
#include <mln/util/surface_mode.hpp>
#include <mln/util/surface_stats.hpp>

#include <mln/util/constants.hpp>

#include <algorithm>
#include <chrono>
#include <iterator>
#include <map>
#include <string>

namespace mln {

using namespace util::surface;

void RenderTerrain::updateSurfaceCells(gfx::Context& context,
                                       const std::vector<VisibleCell>& visible,
                                       const std::vector<OverscaledTileID>& surfaceTiles,
                                       UniqueChangeRequestVec& changes) {
    if (util::SurfaceMode::instance().value() <= 0) {
        if (!cellTargets.empty() || surfacePool.count() > 0) teardownSurfaceCells(changes);
        return;
    }

    // Las celdas se renderizan solo en el frame en que se planifican; el resto del tiempo su target queda inactivo.
    for (auto& [slot, target] : cellTargets) target->setActive(false);

    const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

    // Sucio por tesela (F4): solo las celdas que se solapan con una tesela de imagen que llego o salio desde el cuadro anterior. Lo que no es imagen (capas del usuario,
    // geocercas) se actualiza por el refresco periodico del planificador.
    if (surfaceTiles != surfaceTilesPrev) {
        std::vector<OverscaledTileID> changed;
        std::set_symmetric_difference(
            surfaceTilesPrev.begin(), surfaceTilesPrev.end(), surfaceTiles.begin(), surfaceTiles.end(), std::back_inserter(changed));
        surfacePool.markDirtyIf([&](const CellKey& k) {
            for (const OverscaledTileID& t : changed) {
                if (cellOverlapsTile(k.z, k.x, k.y, k.wrap, t.canonical.z, t.canonical.x, t.canonical.y, t.wrap)) return true;
            }
            return false;
        });
        surfaceTilesPrev = surfaceTiles;
    }

    surfacePool.beginFrame(++surfaceFrame);
    const std::vector<PlanItem> plan = surfaceScheduler.plan(visible, surfacePool, nowMs);

    for (const auto& creation : surfacePool.takeCreations()) {
        auto target = std::make_shared<TileRenderTarget>(context,
                                                         Size{creation.sizePx, creation.sizePx},
                                                         gfx::TextureChannelDataType::UnsignedByte,
                                                         UnwrappedTileID{0, 0, 0});
        target->setActive(false);
        changes.emplace_back(std::make_unique<AddRenderTargetRequest>(target));
        cellTargets[creation.slotId] = std::move(target);
    }

    for (const PlanItem& item : plan) {
        const Entry* entry = surfacePool.find(item.key);
        const auto it = entry ? cellTargets.find(entry->slotId) : cellTargets.end();
        if (it == cellTargets.end()) continue;
        const OverscaledTileID id(item.key.z, item.key.wrap, item.key.z, item.key.x, item.key.y);
        it->second->setTileID(id.toUnwrapped());
        it->second->setCull(util::SurfaceCull{item.key.z, item.key.x, item.key.y, item.key.wrap});
        it->second->setActive(true);
        surfacePool.markRendered(item.key, nowMs);
    }
    surfacePlannedWindow += plan.size();

    for (const uint32_t slot : surfacePool.endFrame()) {
        const auto it = cellTargets.find(slot);
        if (it != cellTargets.end()) {
            changes.emplace_back(std::make_unique<RemoveRenderTargetRequest>(it->second));
            cellTargets.erase(it);
        }
    }

    // Estadisticas de la ventana (cada ~2 s a 60 fps): lo que la F2 tiene que medir.
    if (++surfaceWindowFrames >= 120) {
        const auto& st = surfacePool.stats();
        const auto passes = util::SurfaceStats::instance().takeWindow();
        // Reconocimiento de memoria (plan P4): teselas de imagen retenidas por nivel de zoom y su peso estimado como RGBA8 de 256x256 (256 KB c/u, como las sube el motor).
        std::string tilesPerZ;
        {
            std::map<int, size_t> byZ;
            for (const OverscaledTileID& t : surfaceTilesPrev) ++byZ[t.canonical.z];
            for (const auto& [z, n] : byZ) tilesPerZ += "z" + std::to_string(z) + ":" + std::to_string(n) + " ";
        }
        Log::Warning(Event::General,
                     "E2-CELDAS modo=" + std::to_string(util::SurfaceMode::instance().value()) + " visibles=" + std::to_string(visible.size()) +
                         " planificadas_por_frame=" + std::to_string(static_cast<double>(surfacePlannedWindow) / static_cast<double>(surfaceWindowFrames)) +
                         " texturas=" + std::to_string(surfacePool.count()) + " pool_mb=" + std::to_string(surfacePool.bytesInUse() / (1024.0 * 1024.0)) +
                         " reuso_mb=" + std::to_string(surfacePool.bytesInReusePool() / (1024.0 * 1024.0)) + " pico_mb=" +
                         std::to_string(st.peakBytes / (1024.0 * 1024.0)) + " creaciones=" + std::to_string(st.creations) + " reusos=" +
                         std::to_string(st.reuses) + " expulsiones=" + std::to_string(st.evictions) + " destruidas=" + std::to_string(st.destroyed) +
                         " fuera_de_tope=" + std::to_string(st.overBudgetFrames) + " targets=" + std::to_string(cellTargets.size()) +
                         " teselas_imagen=" + std::to_string(surfaceTilesPrev.size()) + " (~" + std::to_string(surfaceTilesPrev.size() / 4) + " MB) por_z=[" + tilesPerZ + "]" +
                         " usa_propia=" + std::to_string(surfaceUsedOwn) + " usa_ancestro=" + std::to_string(surfaceUsedAncestor) + " usa_ninguna=" +
                         std::to_string(surfaceUsedNone) + " max_niveles_arriba=" + std::to_string(surfaceMaxLevelsUp) +
                         " pasadas=" + std::to_string(passes.passes) + " ms_por_pasada=" +
                         std::to_string(passes.passes ? passes.totalMicros / 1000.0 / static_cast<double>(passes.passes) : 0.0) + " ms_max_pasada=" +
                         std::to_string(passes.maxMicros / 1000.0));
        surfacePlannedWindow = 0;
        surfaceWindowFrames = 0;
        surfaceUsedOwn = surfaceUsedAncestor = surfaceUsedNone = 0;
        surfaceMaxLevelsUp = 0;
    }
}

std::optional<RenderTerrain::SurfaceCellChoice> RenderTerrain::chooseSurfaceCell(const CellKey& key) const {
    if (util::SurfaceMode::instance().value() < 2) return std::nullopt;

    // Usable = ya se renderizo al menos una vez (renderedAtMs > 0): una celda sucia conserva su imagen vieja y se muestra hasta que llegue la nueva.
    const auto usable = [&](const CellKey& k) -> const Entry* {
        const Entry* e = surfacePool.find(k);
        return e && e->renderedAtMs > 0 && cellTargets.count(e->slotId) ? e : nullptr;
    };
    CellKey source = key;
    const Entry* entry = usable(source);
    while (!entry && source.z > 0) {
        source = source.parent();
        entry = usable(source);
    }
    if (!entry) {
        ++surfaceUsedNone;
        return std::nullopt;
    }
    if (source.z == key.z) {
        ++surfaceUsedOwn;
    } else {
        ++surfaceUsedAncestor;
        surfaceMaxLevelsUp = std::max<unsigned>(surfaceMaxLevelsUp, key.z - source.z);
    }

    // tile-local del sub-tile -> UV de la celda fuente. El render target proyecta el tile con ortho(x0, x1, y1, y0): norte = v 1 (igual que los drapes).
    // Con f sub-tiles por lado de la fuente: u = (x - ax*f + lx/EXTENT)/f ; v = 1 - (y - ay*f + ly/EXTENT)/f.
    const double f = static_cast<double>(1u << (key.z - source.z));
    const double a = 1.0 / (util::EXTENT * f);
    const double bx = (static_cast<double>(key.x) - static_cast<double>(source.x) * f) / f;
    const double by = (static_cast<double>(key.y) - static_cast<double>(source.y) * f) / f;
    std::array<float, 16> m{};
    m[0] = static_cast<float>(a);
    m[5] = static_cast<float>(-a);
    m[10] = 1.0f;
    m[12] = static_cast<float>(bx);
    m[13] = static_cast<float>(1.0 - by);
    m[15] = 1.0f;
    return SurfaceCellChoice{cellTargets.at(entry->slotId)->getTexture(), m, static_cast<unsigned>(key.z - source.z)};
}

void RenderTerrain::teardownSurfaceCells(UniqueChangeRequestVec& changes) {
    for (const auto& [slot, target] : cellTargets) {
        changes.emplace_back(std::make_unique<RemoveRenderTargetRequest>(target));
    }
    cellTargets.clear();
    surfacePool = SurfacePool(PoolConfig{256u * 1024u * 1024u, 4u});
    surfaceFrame = 0;
    surfaceTilesPrev.clear();
    surfacePlannedWindow = 0;
    surfaceWindowFrames = 0;
}

} // namespace mln
