#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

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

/// Capas de linea que se dibujan en el pase propio de la camara real (ADR 0058) en vez de en los drapes del terreno:
/// geometria con pixeles fijos y profundidad contra la malla, sin los bordes dobles/borrosos de los drapes multi-resolucion.
/// Es opt-in por capa: las demas lineas del estilo siguen igual. Solo lineas simples (sin patron, gradiente ni guiones).
class EcefLineRegistry {
public:
    static EcefLineRegistry& get() {
        static EcefLineRegistry instance;
        return instance;
    }

    void set(const std::string& layerId, bool on) {
        std::lock_guard<std::mutex> lock(mutex);
        if (on) {
            ids.insert(layerId);
        } else {
            ids.erase(layerId);
        }
    }

    bool contains(const std::string& layerId) const {
        std::lock_guard<std::mutex> lock(mutex);
        return ids.count(layerId) > 0;
    }

private:
    mutable std::mutex mutex;
    std::unordered_set<std::string> ids;
};

} // namespace util
} // namespace mln
