#pragma once

// Superficie por celda del terreno 3D (ADR 0059, plan E2, fase F1): lógica PURA, sin GPU ni MapLibre, para poder probarla con trazas simuladas.
//
// Modelo de ATAK (`GLGlobeSurfaceRenderer` / `GLMegaTexture`, solo lectura): cada celda del LOD del terreno tiene su propia textura del tamaño que le toca
// (`mip = min((nivel0 − nivel)/2, 4)`, 512 px → 32 px), en un formato de 2 bytes por píxel (RGB565, porque la superficie es opaca), con un pool de reuso chico y
// presupuesto de tiempo por frame. Aquí:
//   - `CellKey`/`mipFor`/`sizeForMip`/`uvWindowInAncestor`: identidad, tamaño por mip y recorte de UV para el respaldo por ancestro.
//   - `SurfacePool`: qué celdas tienen textura, de qué tamaño, limpias o sucias; expulsión LRU de lo que no se pidió en el frame y reuso de texturas.
//   - `SurfaceScheduler`: qué celdas se (re)renderizan este frame, en qué orden y con qué presupuesto.
// ATAK marca TODO sucio cada 3 s (`DEFAULT_SURFACE_REFRESH_INTERVAL`); acá el refresco periódico está APAGADO por defecto (`refreshMs = 0`): con la cámara
// quieta y todo limpio no hay pasadas. La invalidación es por celda (`markDirty`, `markDirtyIf`).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace mln::util::surface {

/// Identidad de una celda de superficie: la misma de la celda del LOD (z/x/y mercator + copia del mundo).
struct CellKey {
    uint8_t z = 0;
    uint32_t x = 0;
    uint32_t y = 0;
    int16_t wrap = 0;

    friend bool operator==(const CellKey& a, const CellKey& b) {
        return a.z == b.z && a.x == b.x && a.y == b.y && a.wrap == b.wrap;
    }
    friend bool operator<(const CellKey& a, const CellKey& b) {
        return std::tie(a.z, a.wrap, a.x, a.y) < std::tie(b.z, b.wrap, b.x, b.y);
    }
    /// La celda de un nivel más grueso que contiene a esta (la raíz es su propio padre).
    CellKey parent() const { return CellKey{static_cast<uint8_t>(z > 0 ? z - 1 : 0), x >> 1, y >> 1, wrap}; }
};

/// Mip como ATAK: `min((nivel0 − nivel)/2, maxMip)`; `nivel0` es el nivel de la celda visible más fina.
inline unsigned mipFor(unsigned level, unsigned level0, unsigned maxMip = 4) {
    return level >= level0 ? 0u : std::min((level0 - level) / 2u, maxMip);
}

/// Lado en píxeles de la textura de una celda con ese mip (512 → 32 px).
inline uint32_t sizeForMip(unsigned mip, uint32_t maxTilePx = 512, uint32_t minTilePx = 32) {
    return std::max<uint32_t>(maxTilePx >> mip, minTilePx);
}

/// Subventana de UV de una celda dentro de la textura de un ancestro: `uvAncestro = offset + uvCelda × scale`.
struct UvWindow {
    float scale = 1.0f;
    float offsetX = 0.0f;
    float offsetY = 0.0f;
};

/// Respaldo por ancestro: dónde cae `cell` dentro de `ancestor`. `nullopt` si `ancestor` no contiene a `cell` (otro nivel más fino, otra copia del mundo o no es ancestro).
inline std::optional<UvWindow> uvWindowInAncestor(const CellKey& cell, const CellKey& ancestor) {
    if (ancestor.z > cell.z || ancestor.wrap != cell.wrap) return std::nullopt;
    const unsigned dz = static_cast<unsigned>(cell.z - ancestor.z);
    if ((static_cast<uint64_t>(cell.x) >> dz) != ancestor.x || (static_cast<uint64_t>(cell.y) >> dz) != ancestor.y) return std::nullopt;
    const double n = std::ldexp(1.0, static_cast<int>(dz));
    return UvWindow{static_cast<float>(1.0 / n),
                    static_cast<float>((static_cast<double>(cell.x) - static_cast<double>(static_cast<uint64_t>(ancestor.x) << dz)) / n),
                    static_cast<float>((static_cast<double>(cell.y) - static_cast<double>(static_cast<uint64_t>(ancestor.y) << dz)) / n)};
}

enum class CellState { Dirty, Clean };

/// ¿La celda (z,x,y,wrap) y la tesela (tz,tx,ty,twrap) cubren alguna zona en común? Es cierto si una es ancestro (o igual) de la otra.
inline bool cellOverlapsTile(uint8_t z, uint32_t x, uint32_t y, int16_t wrap, uint8_t tz, uint32_t tx, uint32_t ty, int16_t twrap) {
    if (wrap != twrap) return false;
    if (z <= tz) {
        const unsigned d = tz - z;
        return (tx >> d) == x && (ty >> d) == y;
    }
    const unsigned d = z - tz;
    return (x >> d) == tx && (y >> d) == ty;
}

struct Entry {
    CellKey key;
    uint32_t sizePx = 0;
    uint32_t slotId = 0;            // la textura de la GPU que respalda esta celda (la capa de GPU la crea con `takeCreations`)
    CellState state = CellState::Dirty;
    uint64_t lastUsedFrame = 0;
    int64_t renderedAtMs = 0;
};

struct PoolConfig {
    size_t capBytes = 256u * 1024u * 1024u;        // D2: techo de memoria de las texturas de superficie
    unsigned bytesPerPixel = 2;                     // RGB565, como ATAK (RGBA8 = 4)
    size_t reuseLimitBytes = 2u * 1024u * 1024u;    // pool de reuso de ATAK (`MT_POOL_LIMIT`)
    uint32_t maxTilePx = 1024;                      // D4: 512 y se prueba 1024 si de cerca sigue corto; con el tamaño por pantalla (F4) el tope sube a 1024 (el tope de memoria lo cuida)
    uint32_t minTilePx = 32;
    unsigned maxMip = 4;
    // Frames sin pedir una celda antes de expulsarla AUNQUE haya lugar bajo el tope (0 = sin retención por edad: caché LRU que se llena hasta el tope). ATAK no guarda lo que no
    // es visible (suelta la textura al salir de pantalla y su pool de reuso es de 2 MB): si el pool se llenara de celdas viejas hasta el tope, la memoria subiría al techo (medido
    // en la F2: +240 MB con el techo de 256 MB). Con 120 frames (~2-4 s) volver a una celda recién dejada sigue siendo gratis.
    uint64_t retainFrames = 120;
};

/// Texturas por celda: asignación, estado, expulsión LRU y reuso. No toca la GPU: informa qué texturas crear y cuáles destruir.
class SurfacePool {
public:
    struct Creation {
        uint32_t slotId;
        uint32_t sizePx;
    };
    struct Stats {
        size_t evictions = 0;          // celdas expulsadas por memoria
        size_t reuses = 0;             // texturas reusadas del pool de reuso
        size_t creations = 0;          // texturas nuevas pedidas a la GPU
        size_t destroyed = 0;          // texturas que la GPU debe destruir
        size_t overBudgetFrames = 0;   // frames en que lo visible solo ya superaba el tope (no se expulsa lo visible)
        size_t peakBytes = 0;
    };

    explicit SurfacePool(PoolConfig config = {}) : config_(config) {}

    const PoolConfig& config() const { return config_; }
    const Stats& stats() const { return stats_; }
    size_t bytesInUse() const { return bytes_; }
    size_t bytesInReusePool() const { return bytesFree_; }
    size_t count() const { return entries_.size(); }

    void beginFrame(uint64_t frame) { frame_ = frame; }

    /// Pide la textura de una celda con el tamaño que le toca (la marca como usada en este frame). Si es nueva queda sucia; si cambió de tamaño, se cambia de textura y queda sucia.
    /// Antes de reservar una textura nueva hace lugar bajo el tope: primero reusa una libre del mismo tamaño, si no destruye las libres y expulsa por LRU lo que NO se pidió en
    /// este frame (su textura se reusa si es del mismo tamaño). Así la memoria no pasa del tope ni de forma transitoria, y una escena que rueda no crea ni destruye texturas.
    Entry& request(const CellKey& key, uint32_t sizePx) {
        auto it = entries_.find(key);
        if (it != entries_.end()) {
            Entry& e = it->second;
            e.lastUsedFrame = frame_;
            if (e.sizePx != sizePx) {
                bytes_ -= bytesFor(e.sizePx);
                releaseSlot(e.slotId, e.sizePx);
                makeRoom(sizePx);
                e.sizePx = sizePx;
                e.slotId = allocateSlot(sizePx);
                bytes_ += bytesFor(sizePx);
                e.state = CellState::Dirty;
                e.renderedAtMs = 0;   // la textura nueva esta vacia: no se puede mostrar hasta renderizarla
                peak();
            }
            return e;
        }
        makeRoom(sizePx);
        Entry e;
        e.key = key;
        e.sizePx = sizePx;
        e.slotId = allocateSlot(sizePx);
        e.state = CellState::Dirty;
        e.lastUsedFrame = frame_;
        bytes_ += bytesFor(sizePx);
        peak();
        return entries_.emplace(key, e).first->second;
    }

    Entry* find(const CellKey& key) {
        auto it = entries_.find(key);
        return it == entries_.end() ? nullptr : &it->second;
    }
    const Entry* find(const CellKey& key) const {
        auto it = entries_.find(key);
        return it == entries_.end() ? nullptr : &it->second;
    }

    void markRendered(const CellKey& key, int64_t nowMs) {
        if (Entry* e = find(key)) {
            e->state = CellState::Clean;
            e->renderedAtMs = nowMs;
        }
    }

    void markDirty(const CellKey& key) {
        if (Entry* e = find(key)) e->state = CellState::Dirty;
    }

    /// Invalidación por área, capa o tesela fuente: marca sucias las celdas para las que `pred(key)` es verdadero. Devuelve cuántas.
    template <class Pred>
    size_t markDirtyIf(Pred pred) {
        size_t n = 0;
        for (auto& [key, e] : entries_) {
            if (e.state == CellState::Clean && pred(key)) {
                e.state = CellState::Dirty;
                ++n;
            }
        }
        return n;
    }

    void markAllDirty() { markDirtyIf([](const CellKey&) { return true; }); }

    /// Texturas nuevas que la GPU debe crear (las pedidas desde la última llamada); las reusadas del pool ya existen con ese tamaño.
    std::vector<Creation> takeCreations() { return std::exchange(creations_, {}); }

    /// El ancestro limpio más cercano (para pintar mientras la celda no tiene textura propia).
    std::optional<CellKey> nearestCleanAncestor(const CellKey& key) const {
        CellKey k = key;
        while (k.z > 0) {
            k = k.parent();
            const Entry* e = find(k);
            if (e && e->state == CellState::Clean) return k;
        }
        return std::nullopt;
    }

    /// Los hijos directos de `key` con textura limpia (se superponen al pintar de grueso a fino).
    std::vector<CellKey> cleanChildren(const CellKey& key) const {
        std::vector<CellKey> out;
        if (key.z >= 255) return out;
        for (uint32_t dy = 0; dy < 2; ++dy) {
            for (uint32_t dx = 0; dx < 2; ++dx) {
                const CellKey c{static_cast<uint8_t>(key.z + 1), key.x * 2 + dx, key.y * 2 + dy, key.wrap};
                const Entry* e = find(c);
                if (e && e->state == CellState::Clean) out.push_back(c);
            }
        }
        return out;
    }

    /// Cierra el frame: expulsa por LRU lo que NO se pidió en este frame hasta entrar en el tope (lo pedido nunca se expulsa) y recorta el pool de reuso.
    /// Devuelve las texturas que la GPU debe destruir ahora.
    std::vector<uint32_t> endFrame() {
        if (config_.retainFrames > 0) {
            std::vector<CellKey> stale;
            for (const auto& [key, e] : entries_) {
                if (frame_ > e.lastUsedFrame + config_.retainFrames) stale.push_back(key);
            }
            for (const CellKey& key : stale) evict(&entries_.at(key));
        }
        if (bytes_ > config_.capBytes) {
            std::vector<Entry*> candidates;
            for (auto& [key, e] : entries_) {
                if (e.lastUsedFrame < frame_) candidates.push_back(&e);
            }
            std::sort(candidates.begin(), candidates.end(), [](const Entry* a, const Entry* b) {
                return std::tie(a->lastUsedFrame, a->key) < std::tie(b->lastUsedFrame, b->key);
            });
            for (Entry* e : candidates) {
                if (bytes_ <= config_.capBytes) break;
                evict(e);
            }
            if (bytes_ > config_.capBytes) overBudget_ = true;
        }
        if (overBudget_) ++stats_.overBudgetFrames;
        overBudget_ = false;
        trimReusePool(destroy_);
        return takeDestroyed();
    }

    /// Texturas que la GPU debe destruir ahora (se acumulan al hacer lugar; `endFrame` también las entrega).
    std::vector<uint32_t> takeDestroyed() {
        stats_.destroyed += destroy_.size();
        return std::exchange(destroy_, {});
    }

private:
    size_t bytesFor(uint32_t sizePx) const { return static_cast<size_t>(sizePx) * sizePx * config_.bytesPerPixel; }
    void peak() { stats_.peakBytes = std::max(stats_.peakBytes, bytes_ + bytesFree_); }

    bool hasFree(uint32_t sizePx) const {
        auto it = free_.find(sizePx);
        return it != free_.end() && !it->second.empty();
    }

    // Destruye la textura libre más grande; false si no hay ninguna.
    bool destroyOneFree() {
        auto biggest = free_.end();
        for (auto it = free_.begin(); it != free_.end(); ++it) {
            if (!it->second.empty() && (biggest == free_.end() || it->first > biggest->first)) biggest = it;
        }
        if (biggest == free_.end()) return false;
        destroy_.push_back(biggest->second.back());
        biggest->second.pop_back();
        bytesFree_ -= bytesFor(biggest->first);
        return true;
    }

    // La celda menos usada que NO se pidió en este frame (empate por clave); nullptr si todas se pidieron.
    Entry* lruCandidate() {
        Entry* best = nullptr;
        for (auto& [key, e] : entries_) {
            if (e.lastUsedFrame >= frame_) continue;
            if (!best || std::tie(e.lastUsedFrame, e.key) < std::tie(best->lastUsedFrame, best->key)) best = &e;
        }
        return best;
    }

    void evict(Entry* e) {
        bytes_ -= bytesFor(e->sizePx);
        releaseSlot(e->slotId, e->sizePx);
        const CellKey key = e->key;
        entries_.erase(key);
        ++stats_.evictions;
    }

    // Hace lugar para una textura de `sizePx` bajo el tope: reuso, luego destruir libres, luego expulsar lo no pedido en este frame.
    void makeRoom(uint32_t sizePx) {
        const size_t need = bytesFor(sizePx);
        while (!hasFree(sizePx) && bytes_ + bytesFree_ + need > config_.capBytes) {
            if (destroyOneFree()) continue;
            Entry* victim = lruCandidate();
            if (!victim) {
                overBudget_ = true;   // todo lo que hay se pidió en este frame: no se puede expulsar
                break;
            }
            evict(victim);
        }
    }

    uint32_t allocateSlot(uint32_t sizePx) {
        auto it = free_.find(sizePx);
        if (it != free_.end() && !it->second.empty()) {
            const uint32_t slot = it->second.back();
            it->second.pop_back();
            bytesFree_ -= bytesFor(sizePx);
            ++stats_.reuses;
            return slot;
        }
        const uint32_t slot = nextSlot_++;
        creations_.push_back({slot, sizePx});
        ++stats_.creations;
        return slot;
    }

    void releaseSlot(uint32_t slot, uint32_t sizePx) {
        free_[sizePx].push_back(slot);
        bytesFree_ += bytesFor(sizePx);
    }

    // El pool de reuso no pasa de `reuseLimitBytes` ni de lo que queda libre bajo el tope; el resto se destruye (las más grandes primero).
    void trimReusePool(std::vector<uint32_t>&) {
        const size_t room = config_.capBytes > bytes_ ? config_.capBytes - bytes_ : 0;
        const size_t limit = std::min(config_.reuseLimitBytes, room);
        while (bytesFree_ > limit && destroyOneFree()) {
        }
    }

    PoolConfig config_;
    Stats stats_;
    std::map<CellKey, Entry> entries_;
    std::map<uint32_t, std::vector<uint32_t>> free_;   // tamaño → texturas libres para reusar
    std::vector<Creation> creations_;
    std::vector<uint32_t> destroy_;
    bool overBudget_ = false;
    size_t bytes_ = 0;
    size_t bytesFree_ = 0;
    uint32_t nextSlot_ = 1;
    uint64_t frame_ = 0;
};

/// Una celda visible del LOD con su distancia al ojo (m).
struct VisibleCell {
    CellKey key;
    double distanceM = 0.0;
    double screenPx = 0.0;   // lado de la celda en pixeles de pantalla (en su punto mas cercano); 0 = desconocido: el tamaño sale del mip como en ATAK
    bool base = false;       // celda gruesa de respaldo (ancestro de las visibles): chica y con prioridad maxima, para que toda celda nueva tenga algo que mostrar mientras se dibuja
};

/// Una celda que hay que (re)renderizar este frame.
struct PlanItem {
    CellKey key;
    uint32_t sizePx = 0;
    unsigned mip = 0;
    double estimatedMs = 0.0;
    bool base = false;
};

struct SchedulerConfig {
    double budgetMs = 10.0;         // D3: 5 ms por frame; 10 desde la prueba de campo (con 5 se dibujaba ~1 celda por cuadro y al moverse aparecian huecos)
    double msPerMegapixel = 10.0;   // costo estimado de una pasada: medido en la F2 en el S25 Ultra con la GPU sincronizada, ~2,7 ms por celda de 512 px (0,26 Mpx) = ~10 ms/Mpx
    size_t maxItems = 64;
    uint32_t sourceTilePx = 256;    // lado de las teselas de imagen de la fuente
    int sourceMaxZoom = 21;         // zoom maximo de la fuente: mas texels que los nativos no agregan detalle
    double oversample = 1.0;        // texels de la celda por pixel de pantalla (1 = nitido sin desperdicio)
    int64_t refreshMs = 3000;       // refresco periódico como ATAK (D3: 3 s): lo que no es imagen (capas del usuario, geocercas) no tiene invalidación precisa y se actualiza por aquí
};

/// Reduce los tamaños (de a un mip, empezando por los más grandes) hasta que entren en `capBytes`. Nunca baja de `minTilePx`.
inline std::vector<uint32_t> fitSizes(std::vector<uint32_t> sizes, size_t capBytes, unsigned bytesPerPixel, uint32_t minTilePx = 32) {
    auto total = [&] {
        size_t t = 0;
        for (uint32_t s : sizes) t += static_cast<size_t>(s) * s * bytesPerPixel;
        return t;
    };
    while (total() > capBytes) {
        auto biggest = std::max_element(sizes.begin(), sizes.end());
        if (biggest == sizes.end() || *biggest <= minTilePx) break;
        *biggest = std::max<uint32_t>(*biggest / 2u, minTilePx);
    }
    return sizes;
}

class SurfaceScheduler {
public:
    explicit SurfaceScheduler(SchedulerConfig config = {}) : config_(config) {}

    /// Qué celdas visibles se (re)renderizan este frame, de lo más urgente a lo menos. Pide al pool la textura de cada celda visible (la marca como usada).
    /// Orden: mip más fino primero (menos compresión = más cerca de la cámara en nivel), luego distancia al ojo. Se corta al llegar al presupuesto de ms, pero siempre sale
    /// al menos una celda si hay alguna pendiente (para que el trabajo avance aunque una celda sola supere el presupuesto).
    std::vector<PlanItem> plan(const std::vector<VisibleCell>& visible, SurfacePool& pool, int64_t nowMs) const {
        const PoolConfig& pc = pool.config();
        unsigned level0 = 0;
        for (const VisibleCell& v : visible) level0 = std::max<unsigned>(level0, v.key.z);

        std::vector<uint32_t> sizes;
        std::vector<unsigned> mips;
        sizes.reserve(visible.size());
        for (const VisibleCell& v : visible) {
            if (v.screenPx > 0.0) {
                // Tamaño por pantalla (F4): una potencia de 2 >= los pixeles que ocupa la celda, sin pasar de los texels nativos de la imagen ni del tope.
                const double native = v.key.z <= config_.sourceMaxZoom ? static_cast<double>(config_.sourceTilePx) * static_cast<double>(1ull << (config_.sourceMaxZoom - v.key.z))
                                                                       : std::max(1.0, static_cast<double>(config_.sourceTilePx) / static_cast<double>(1ull << std::min(30, v.key.z - config_.sourceMaxZoom)));
                const double want = std::min(v.screenPx * config_.oversample, native);
                uint32_t size = pc.minTilePx;
                while (size < pc.maxTilePx && static_cast<double>(size) < want) size <<= 1;
                // Histeresis: si la celda ya tiene una textura de entre 1x y 2x el tamaño buscado, se conserva (evita rehacerla por oscilar en el borde de un escalon).
                if (const Entry* existing = pool.find(v.key); existing && existing->sizePx >= size && existing->sizePx <= size * 2u) size = existing->sizePx;
                unsigned mip = 0;
                while ((pc.maxTilePx >> mip) > size && mip < 16) ++mip;
                mips.push_back(mip);
                sizes.push_back(size);
                continue;
            }
            const unsigned mip = mipFor(v.key.z, level0, pc.maxMip);
            mips.push_back(mip);
            sizes.push_back(sizeForMip(mip, pc.maxTilePx, pc.minTilePx));
        }
        sizes = fitSizes(std::move(sizes), pc.capBytes, pc.bytesPerPixel, pc.minTilePx);

        struct Candidate {
            PlanItem item;
            double distanceM;
        };
        std::vector<Candidate> pending;
        for (size_t i = 0; i < visible.size(); ++i) {
            const VisibleCell& v = visible[i];
            Entry& e = pool.request(v.key, sizes[i]);
            const bool stale = config_.refreshMs > 0 && e.state == CellState::Clean && nowMs - e.renderedAtMs >= config_.refreshMs;
            if (e.state == CellState::Clean && !stale) continue;
            const double mpx = static_cast<double>(sizes[i]) * sizes[i] / 1.0e6;
            PlanItem planItem{v.key, sizes[i], mips[i], mpx * config_.msPerMegapixel};
            planItem.base = v.base;
            pending.push_back({planItem, v.distanceM});
        }
        std::sort(pending.begin(), pending.end(), [](const Candidate& a, const Candidate& b) {
            // Las celdas base primero (baratas y son el respaldo de todas las demas), despues de lo mas fino a lo mas grueso.
            const bool aFirst = !a.item.base;
            const bool bFirst = !b.item.base;
            return std::tie(aFirst, a.item.mip, a.distanceM, a.item.key) < std::tie(bFirst, b.item.mip, b.distanceM, b.item.key);
        });

        std::vector<PlanItem> out;
        double spent = 0.0;
        for (const Candidate& c : pending) {
            if (!out.empty() && (spent + c.item.estimatedMs > config_.budgetMs || out.size() >= config_.maxItems)) break;
            out.push_back(c.item);
            spent += c.item.estimatedMs;
        }
        return out;
    }

    const SchedulerConfig& config() const { return config_; }

private:
    SchedulerConfig config_;
};

} // namespace mln::util::surface
