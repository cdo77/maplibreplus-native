#include <gtest/gtest.h>

#include <mln/util/surface_cells.hpp>

#include <set>

using namespace mln::util::surface;

namespace {

constexpr size_t kCell512 = 512u * 512u * 2u; // una celda de 512 px en RGB565 = 0,5 MiB

CellKey key(unsigned z, uint32_t x, uint32_t y, int16_t wrap = 0) {
    return CellKey{static_cast<uint8_t>(z), x, y, wrap};
}

// Una celda visible a partir de su posición en un anillo alrededor de la cámara: los anillos lejanos son de un nivel más grueso.
std::vector<VisibleCell> visibleRings(unsigned level0, uint32_t shift, int rings = 8, int perRing = 8) {
    std::vector<VisibleCell> out;
    for (int r = 0; r < rings; ++r) {
        const unsigned level = level0 > static_cast<unsigned>(r) ? level0 - r : 0;
        for (int i = 0; i < perRing; ++i) {
            out.push_back({key(level, 1000 + shift + i, 2000 + r), 20.0 * (1 << r) + i});
        }
    }
    return out;
}

} // namespace

TEST(SurfaceCells, MipPolicyLikeAtak) {
    EXPECT_EQ(mipFor(18, 18), 0u);
    EXPECT_EQ(mipFor(17, 18), 0u);     // (18-17)/2 = 0
    EXPECT_EQ(mipFor(16, 18), 1u);
    EXPECT_EQ(mipFor(10, 18), 4u);
    EXPECT_EQ(mipFor(0, 18), 4u);      // tope
    EXPECT_EQ(mipFor(19, 18), 0u);     // más fina que level0: sin compresión
    EXPECT_EQ(sizeForMip(0), 512u);
    EXPECT_EQ(sizeForMip(1), 256u);
    EXPECT_EQ(sizeForMip(4), 32u);
    EXPECT_EQ(sizeForMip(9), 32u);     // piso
}

TEST(SurfaceCells, ParentAndUvWindowInAncestor) {
    const CellKey c = key(5, 13, 9);
    EXPECT_EQ(c.parent(), key(4, 6, 4));
    EXPECT_EQ(key(0, 0, 0).parent(), key(0, 0, 0));

    const auto one = uvWindowInAncestor(c, key(4, 6, 4));
    ASSERT_TRUE(one.has_value());
    EXPECT_FLOAT_EQ(one->scale, 0.5f);
    EXPECT_FLOAT_EQ(one->offsetX, 0.5f);   // 13 - 6*2 = 1 de 2
    EXPECT_FLOAT_EQ(one->offsetY, 0.5f);   // 9 - 4*2 = 1 de 2

    const auto two = uvWindowInAncestor(c, key(3, 3, 2));
    ASSERT_TRUE(two.has_value());
    EXPECT_FLOAT_EQ(two->scale, 0.25f);
    EXPECT_FLOAT_EQ(two->offsetX, 0.25f);  // 13 - 3*4 = 1 de 4
    EXPECT_FLOAT_EQ(two->offsetY, 0.25f);  // 9 - 2*4 = 1 de 4

    const auto self = uvWindowInAncestor(c, c);
    ASSERT_TRUE(self.has_value());
    EXPECT_FLOAT_EQ(self->scale, 1.0f);

    EXPECT_FALSE(uvWindowInAncestor(c, key(4, 7, 4)).has_value());       // no lo contiene
    EXPECT_FALSE(uvWindowInAncestor(c, key(4, 6, 4, 1)).has_value());    // otra copia del mundo
    EXPECT_FALSE(uvWindowInAncestor(key(4, 6, 4), c).has_value());       // un descendiente no es un ancestro
}

TEST(SurfacePool, NewCellIsDirtyAndAccountsBytes) {
    SurfacePool pool;
    pool.beginFrame(1);
    Entry& e = pool.request(key(10, 1, 1), 512);
    EXPECT_EQ(e.state, CellState::Dirty);
    EXPECT_EQ(pool.bytesInUse(), kCell512);
    EXPECT_EQ(pool.takeCreations().size(), 1u);
    EXPECT_TRUE(pool.takeCreations().empty());   // ya se entregaron

    pool.markRendered(key(10, 1, 1), 100);
    EXPECT_EQ(pool.find(key(10, 1, 1))->state, CellState::Clean);
    EXPECT_EQ(pool.find(key(10, 1, 1))->renderedAtMs, 100);
}

TEST(SurfacePool, ResizeChangesTextureAndMarksDirty) {
    SurfacePool pool;
    pool.beginFrame(1);
    pool.request(key(10, 1, 1), 512);
    pool.markRendered(key(10, 1, 1), 1);
    pool.takeCreations();

    pool.beginFrame(2);
    Entry& e = pool.request(key(10, 1, 1), 256);
    EXPECT_EQ(e.state, CellState::Dirty);
    EXPECT_EQ(e.sizePx, 256u);
    EXPECT_EQ(pool.bytesInUse(), 256u * 256u * 2u);
    EXPECT_EQ(pool.takeCreations().size(), 1u);   // una textura nueva del tamaño nuevo
}

TEST(SurfacePool, EvictsLeastRecentlyUsedAndNeverWhatWasRequestedThisFrame) {
    PoolConfig cfg;
    cfg.capBytes = 6 * kCell512;
    cfg.reuseLimitBytes = 0;
    SurfacePool pool(cfg);

    pool.beginFrame(1);
    for (uint32_t i = 0; i < 6; ++i) pool.request(key(12, i, 0), 512);
    EXPECT_TRUE(pool.endFrame().empty());
    EXPECT_EQ(pool.count(), 6u);

    pool.beginFrame(2);
    pool.request(key(12, 0, 0), 512);   // se vuelven a usar A y B
    pool.request(key(12, 1, 0), 512);
    pool.request(key(12, 6, 0), 512);   // y dos celdas nuevas
    pool.request(key(12, 7, 0), 512);
    pool.endFrame();

    EXPECT_LE(pool.bytesInUse(), cfg.capBytes);
    EXPECT_EQ(pool.count(), 6u);
    for (uint32_t i : {0u, 1u, 6u, 7u}) EXPECT_NE(pool.find(key(12, i, 0)), nullptr) << "pedida en este frame: " << i;
    EXPECT_EQ(pool.find(key(12, 2, 0)), nullptr);   // las más viejas (empate por clave) salieron
    EXPECT_EQ(pool.find(key(12, 3, 0)), nullptr);
    EXPECT_NE(pool.find(key(12, 4, 0)), nullptr);
    EXPECT_NE(pool.find(key(12, 5, 0)), nullptr);
    EXPECT_EQ(pool.stats().evictions, 2u);
}

TEST(SurfacePool, VisibleCellsAreNeverEvictedEvenOverBudget) {
    PoolConfig cfg;
    cfg.capBytes = 2 * kCell512;
    cfg.reuseLimitBytes = 0;
    SurfacePool pool(cfg);
    pool.beginFrame(1);
    for (uint32_t i = 0; i < 4; ++i) pool.request(key(12, i, 0), 512);
    pool.endFrame();
    EXPECT_EQ(pool.count(), 4u);                    // todo lo visible se conserva
    EXPECT_EQ(pool.stats().overBudgetFrames, 1u);   // y se avisa que se pasó del tope
    EXPECT_EQ(pool.stats().evictions, 0u);
}

TEST(SurfacePool, RollingSceneReusesTexturesWithoutCreatingOrDestroyingAny) {
    PoolConfig cfg;
    cfg.capBytes = 3 * kCell512;
    cfg.reuseLimitBytes = 1024 * 1024;
    SurfacePool pool(cfg);

    pool.beginFrame(1);
    pool.request(key(12, 0, 0), 512);
    pool.request(key(12, 1, 0), 512);
    EXPECT_TRUE(pool.endFrame().empty());

    pool.beginFrame(2);
    pool.request(key(12, 2, 0), 512);   // 3 celdas: entra en el tope
    pool.request(key(12, 3, 0), 512);   // la 4ª no entra: se expulsa A (la más vieja sin pedir) y se reusa SU textura
    EXPECT_TRUE(pool.endFrame().empty());
    EXPECT_EQ(pool.stats().creations, 3u);   // A, B y C; D reusó la de A
    EXPECT_EQ(pool.stats().reuses, 1u);
    EXPECT_EQ(pool.stats().evictions, 1u);
    EXPECT_EQ(pool.count(), 3u);
    EXPECT_EQ(pool.find(key(12, 0, 0)), nullptr);
    EXPECT_LE(pool.bytesInUse() + pool.bytesInReusePool(), cfg.capBytes);

    pool.beginFrame(3);
    pool.request(key(12, 4, 0), 512);   // la escena sigue rodando: reusa la de B
    EXPECT_TRUE(pool.endFrame().empty());
    EXPECT_EQ(pool.stats().creations, 3u);
    EXPECT_EQ(pool.stats().reuses, 2u);
    EXPECT_EQ(pool.stats().destroyed, 0u);
}

TEST(SurfacePool, DestroysFreeTexturesOfADifferentSizeWhenMemoryIsTight) {
    PoolConfig cfg;
    cfg.capBytes = kCell512;            // una celda de 512 px y nada más
    cfg.reuseLimitBytes = 1024 * 1024;
    SurfacePool pool(cfg);

    pool.beginFrame(1);
    pool.request(key(12, 0, 0), 512);
    pool.endFrame();

    pool.beginFrame(2);
    pool.request(key(12, 1, 0), 256);   // otro tamaño: no puede reusar la de 512; hay que expulsar A y destruir su textura para hacer lugar
    const auto destroyed = pool.endFrame();
    EXPECT_EQ(destroyed.size(), 1u);
    EXPECT_EQ(pool.count(), 1u);
    EXPECT_LE(pool.bytesInUse() + pool.bytesInReusePool(), cfg.capBytes);
}

TEST(SurfacePool, CellsNotRequestedForRetainFramesAreEvictedEvenUnderTheCap) {
    PoolConfig cfg;
    cfg.retainFrames = 10;
    SurfacePool pool(cfg);   // tope de 256 MB: sobra lugar, pero no se guardan celdas viejas

    pool.beginFrame(1);
    pool.request(key(12, 0, 0), 512);
    pool.request(key(12, 1, 0), 512);
    pool.endFrame();
    for (uint64_t f = 2; f <= 11; ++f) {      // 10 frames sin pedir: todavía se conservan (se puede volver a la celda gratis)
        pool.beginFrame(f);
        pool.request(key(12, 1, 0), 512);     // B se sigue pidiendo; A no
        pool.endFrame();
    }
    EXPECT_NE(pool.find(key(12, 0, 0)), nullptr);

    pool.beginFrame(12);
    pool.request(key(12, 1, 0), 512);
    pool.endFrame();
    EXPECT_EQ(pool.find(key(12, 0, 0)), nullptr);   // pasaron más de 10 frames sin pedirla
    EXPECT_NE(pool.find(key(12, 1, 0)), nullptr);
    EXPECT_EQ(pool.count(), 1u);
    EXPECT_EQ(pool.bytesInUse(), kCell512);
    EXPECT_LE(pool.bytesInReusePool(), cfg.reuseLimitBytes);   // lo liberado solo queda en el pool de reuso chico, no como caché
}

TEST(SurfacePool, WithoutRetentionTheCacheFillsUpToTheCapAsBefore) {
    PoolConfig cfg;
    cfg.retainFrames = 0;
    SurfacePool pool(cfg);
    pool.beginFrame(1);
    pool.request(key(12, 0, 0), 512);
    pool.endFrame();
    pool.beginFrame(1000);
    pool.endFrame();
    EXPECT_NE(pool.find(key(12, 0, 0)), nullptr);
}

TEST(SurfacePool, NearestCleanAncestorSkipsDirtyAndMissing) {
    SurfacePool pool;
    pool.beginFrame(1);
    pool.request(key(14, 100, 50), 512);
    pool.request(key(12, 25, 12), 128);
    pool.request(key(10, 6, 3), 64);
    pool.markRendered(key(10, 6, 3), 1);             // solo el abuelo-abuelo está limpio; el de nivel 12 está sucio

    const auto a = pool.nearestCleanAncestor(key(14, 100, 50));
    ASSERT_TRUE(a.has_value());
    EXPECT_EQ(*a, key(10, 6, 3));
    EXPECT_FALSE(pool.nearestCleanAncestor(key(10, 6, 3)).has_value());     // no hay más arriba
    EXPECT_FALSE(pool.nearestCleanAncestor(key(14, 999, 50)).has_value());  // otra rama
}

TEST(SurfacePool, MarkDirtyIfInvalidatesOnlyMatchingCleanCells) {
    SurfacePool pool;
    pool.beginFrame(1);
    for (uint32_t i = 0; i < 4; ++i) {
        pool.request(key(12, i, 0), 256);
        pool.markRendered(key(12, i, 0), 1);
    }
    const size_t n = pool.markDirtyIf([](const CellKey& k) { return k.x >= 2; });   // p. ej. cambió una capa que toca esas celdas
    EXPECT_EQ(n, 2u);
    EXPECT_EQ(pool.find(key(12, 0, 0))->state, CellState::Clean);
    EXPECT_EQ(pool.find(key(12, 3, 0))->state, CellState::Dirty);
    pool.markAllDirty();
    EXPECT_EQ(pool.find(key(12, 0, 0))->state, CellState::Dirty);
}

TEST(SurfaceScheduler, OrdersByMipThenDistanceAndStopsAtTheBudget) {
    SurfacePool pool;
    pool.beginFrame(1);
    SchedulerConfig cfg;
    cfg.budgetMs = 1000.0;
    SurfaceScheduler sched(cfg);

    // nivel 18 (mip 0) lejos, nivel 16 (mip 1) cerca, nivel 14 (mip 2) muy cerca
    const std::vector<VisibleCell> visible = {{key(14, 1, 1), 5.0}, {key(18, 9, 9), 300.0}, {key(16, 4, 4), 50.0}, {key(18, 9, 8), 100.0}};
    const auto plan = sched.plan(visible, pool, 0);
    ASSERT_EQ(plan.size(), 4u);
    EXPECT_EQ(plan[0].key, key(18, 9, 8));   // mip 0, la más cercana primero
    EXPECT_EQ(plan[1].key, key(18, 9, 9));
    EXPECT_EQ(plan[2].key, key(16, 4, 4));   // mip 1
    EXPECT_EQ(plan[3].key, key(14, 1, 1));   // mip 2
    EXPECT_EQ(plan[0].sizePx, 512u);
    EXPECT_EQ(plan[2].sizePx, 256u);
    EXPECT_EQ(plan[3].sizePx, 128u);
}

TEST(SurfaceScheduler, BudgetLimitsTheWorkPerFrameButAlwaysProgresses) {
    SurfacePool pool;
    pool.beginFrame(1);
    SchedulerConfig cfg;
    cfg.msPerMegapixel = 2.0;   // 5 ms y 2 ms por megapíxel: una celda de 512 px = 0,524 ms
    SurfaceScheduler sched(cfg);
    std::vector<VisibleCell> visible;
    for (uint32_t i = 0; i < 12; ++i) visible.push_back({key(15, i, 0), 10.0 + i});
    const auto plan = sched.plan(visible, pool, 0);
    EXPECT_EQ(plan.size(), 9u);              // 9 × 0,524 = 4,7 ms; la décima pasaría de 5
    EXPECT_EQ(plan.front().key, key(15, 0, 0));

    SchedulerConfig calibrated;   // con el costo medido en la F2 (10 ms/Mpx) caben 1 o 2 celdas de 512 px en 5 ms
    SurfaceScheduler real(calibrated);
    SurfacePool pool3;
    pool3.beginFrame(1);
    EXPECT_EQ(real.plan(visible, pool3, 0).size(), 1u);   // 2,62 ms la primera; la segunda pasaría de 5 ms

    SchedulerConfig tiny;
    tiny.budgetMs = 0.1;                     // una celda sola supera el presupuesto: igual sale una por frame
    SurfaceScheduler slow(tiny);
    SurfacePool pool2;
    pool2.beginFrame(1);
    EXPECT_EQ(slow.plan(visible, pool2, 0).size(), 1u);
}

TEST(SurfaceScheduler, QuietCameraMeansZeroPassesAndRefreshIsOptIn) {
    SurfacePool pool;
    SurfaceScheduler sched;   // refreshMs = 0
    const auto visible = visibleRings(16, 0, 3, 4);
    uint64_t frame = 1;
    for (int guard = 0; guard < 200; ++guard, ++frame) {
        pool.beginFrame(frame);
        const auto plan = sched.plan(visible, pool, 1000);
        for (const auto& it : plan) pool.markRendered(it.key, 1000);
        pool.endFrame();
        if (plan.empty()) break;
    }
    pool.beginFrame(frame + 1);
    EXPECT_TRUE(sched.plan(visible, pool, 1000 + 60000).empty());   // un minuto después, todo limpio: nada que hacer

    SchedulerConfig withRefresh;
    withRefresh.refreshMs = 3000;
    SurfaceScheduler refreshing(withRefresh);
    pool.beginFrame(frame + 2);
    EXPECT_TRUE(refreshing.plan(visible, pool, 1000 + 2000).empty());
    EXPECT_FALSE(refreshing.plan(visible, pool, 1000 + 3500).empty());
}

TEST(SurfaceScheduler, StaticSceneConvergesAndStaysUnderTheCap) {
    PoolConfig pcfg;
    pcfg.capBytes = 16 * 1024 * 1024;
    SurfacePool pool(pcfg);
    SurfaceScheduler sched;
    const auto visible = visibleRings(18, 0);   // 64 celdas de 8 niveles

    uint64_t frame = 1;
    int framesUntilDone = -1;
    for (int guard = 0; guard < 400; ++guard, ++frame) {
        pool.beginFrame(frame);
        const auto plan = sched.plan(visible, pool, 0);
        if (plan.empty()) { framesUntilDone = guard; break; }
        for (const auto& it : plan) pool.markRendered(it.key, 0);
        EXPECT_LE(pool.bytesInUse(), pcfg.capBytes);
        pool.endFrame();
    }
    ASSERT_GE(framesUntilDone, 0) << "la escena estática no convergió";
    EXPECT_LE(framesUntilDone, 60);
    for (const auto& v : visible) {
        const Entry* e = pool.find(v.key);
        ASSERT_NE(e, nullptr);
        EXPECT_EQ(e->state, CellState::Clean);
    }
}

TEST(SurfaceScheduler, FitSizesShrinksTheBiggestFirstAndNeverBelowTheMinimum) {
    const auto fit = fitSizes({512, 512, 512, 512}, 1024 * 1024, 2);
    EXPECT_EQ(fit, (std::vector<uint32_t>{256, 256, 256, 512}));
    const auto floor = fitSizes({512, 512}, 1, 2);
    EXPECT_EQ(floor, (std::vector<uint32_t>{32, 32}));
}

// Traza: la cámara se acerca (el nivel sube) y se desplaza; en cada frame se renderiza lo planificado. Invariantes: la memoria nunca pasa del tope, lo visible siempre
// tiene entrada, y las texturas se reusan en vez de crearse sin parar.
TEST(SurfaceScheduler, ApproachTraceKeepsMemoryBoundedAndReusesTextures) {
    PoolConfig pcfg;
    pcfg.capBytes = 6 * 1024 * 1024;
    SurfacePool pool(pcfg);
    SurfaceScheduler sched;

    uint64_t frame = 1;
    for (int t = 0; t < 600; ++t, ++frame) {
        const unsigned level0 = 12 + static_cast<unsigned>(t / 100);        // se acerca: 12 → 17
        const auto visible = visibleRings(level0, static_cast<uint32_t>(t / 10));
        pool.beginFrame(frame);
        for (const auto& it : sched.plan(visible, pool, t)) pool.markRendered(it.key, t);
        pool.endFrame();
        ASSERT_LE(pool.bytesInUse() + pool.bytesInReusePool(), pcfg.capBytes) << "frame " << t;
        for (const auto& v : visible) ASSERT_NE(pool.find(v.key), nullptr) << "frame " << t;
    }
    EXPECT_GT(pool.stats().evictions, 0u);
    EXPECT_GT(pool.stats().reuses, 0u);
    EXPECT_EQ(pool.stats().overBudgetFrames, 0u);
    EXPECT_LE(pool.stats().peakBytes, pcfg.capBytes);   // ni siquiera de forma transitoria
}
