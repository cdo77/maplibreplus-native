#include <mln/renderer/terrain.hpp>

#include <mln/gfx/context.hpp>
#include <mln/util/logging.hpp>
#include <mln/util/surface_cull.hpp>
#include <mln/util/surface_mode.hpp>

#include <chrono>
#include <string>

namespace mln {

using namespace util::surface;

void RenderTerrain::updateSurfaceCells(gfx::Context& context,
                                       const std::vector<VisibleCell>& visible,
                                       uint64_t generation,
                                       UniqueChangeRequestVec& changes) {
    if (util::SurfaceMode::instance().value() <= 0) {
        if (!cellTargets.empty() || surfacePool.count() > 0) teardownSurfaceCells(changes);
        return;
    }

    // Las celdas se renderizan solo en el frame en que se planifican; el resto del tiempo su target queda inactivo.
    for (auto& [slot, target] : cellTargets) target->setActive(false);

    const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

    // Sucio por llegada o cambio de teselas de imagen. En F2 es grueso (cualquier cambio ensucia todo): sirve para medir el peor caso; la F4 lo afina por tesela.
    if (generation != surfaceGeneration) {
        surfaceGeneration = generation;
        surfacePool.markAllDirty();
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
        Log::Warning(Event::General,
                     "E2-CELDAS modo=" + std::to_string(util::SurfaceMode::instance().value()) + " visibles=" + std::to_string(visible.size()) +
                         " planificadas_por_frame=" + std::to_string(static_cast<double>(surfacePlannedWindow) / static_cast<double>(surfaceWindowFrames)) +
                         " texturas=" + std::to_string(surfacePool.count()) + " pool_mb=" + std::to_string(surfacePool.bytesInUse() / (1024.0 * 1024.0)) +
                         " reuso_mb=" + std::to_string(surfacePool.bytesInReusePool() / (1024.0 * 1024.0)) + " pico_mb=" +
                         std::to_string(st.peakBytes / (1024.0 * 1024.0)) + " creaciones=" + std::to_string(st.creations) + " reusos=" +
                         std::to_string(st.reuses) + " expulsiones=" + std::to_string(st.evictions) + " destruidas=" + std::to_string(st.destroyed) +
                         " fuera_de_tope=" + std::to_string(st.overBudgetFrames) + " targets=" + std::to_string(cellTargets.size()));
        surfacePlannedWindow = 0;
        surfaceWindowFrames = 0;
    }
}

void RenderTerrain::teardownSurfaceCells(UniqueChangeRequestVec& changes) {
    for (const auto& [slot, target] : cellTargets) {
        changes.emplace_back(std::make_unique<RemoveRenderTargetRequest>(target));
    }
    cellTargets.clear();
    surfacePool = SurfacePool(PoolConfig{256u * 1024u * 1024u, 4u});
    surfaceFrame = 0;
    surfaceGeneration = 0;
    surfacePlannedWindow = 0;
    surfaceWindowFrames = 0;
}

} // namespace mln
