#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace mln {
namespace util {

/// Altura propia de los sprites de una capa de simbolos (ADR 0058, vuelo): el ancla deja de pegarse al
/// relieve y sube a `altitudeM` (metros sobre el elipsoide) cuando esta por encima del terreno mas de
/// `minAglM`; entre `minAglM` y `2 x minAglM` sube de forma gradual, asi el ruido del GPS en tierra no la
/// levanta y no hay umbral que parpadee. Con `stick` la capa dibuja en cambio el "palito" (linea) que une el
/// suelo con esa altura (lollipop de ATAK). Lo escribe la app (hilo principal) y lo lee el tweaker (hilo
/// de render): de ahi el candado.
struct SpriteAltitude {
    double altitudeM = 0.0;
    float minAglM = 150.0f;
    bool stick = false;
};

class SpriteAltitudeRegistry {
public:
    static SpriteAltitudeRegistry& get() {
        static SpriteAltitudeRegistry instance;
        return instance;
    }

    void set(const std::string& layerId, const SpriteAltitude& value) {
        std::lock_guard<std::mutex> lock(mutex);
        entries[layerId] = value;
    }

    void clear(const std::string& layerId) {
        std::lock_guard<std::mutex> lock(mutex);
        entries.erase(layerId);
    }

    std::optional<SpriteAltitude> find(const std::string& layerId) const {
        std::lock_guard<std::mutex> lock(mutex);
        const auto it = entries.find(layerId);
        if (it == entries.end()) return std::nullopt;
        return it->second;
    }

private:
    mutable std::mutex mutex;
    std::unordered_map<std::string, SpriteAltitude> entries;
};

} // namespace util
} // namespace mln
