#pragma once

#include <mln/map/mode.hpp>
#include <mln/actor/scheduler.hpp>

#include <memory>
#include <numbers>
#include <vector>

#include <mapbox/std/weak.hpp>

namespace mln {

class TransformState;
class FileSource;
class AnnotationManager;
class ImageManager;
class GlyphManager;

namespace gfx {
class DynamicTextureAtlas;
using DynamicTextureAtlasPtr = std::shared_ptr<gfx::DynamicTextureAtlas>;
} // namespace gfx

class TileParameters {
public:
    const float pixelRatio;
    const MapDebugOptions debugOptions;
    const TransformState& transformState;
    std::shared_ptr<FileSource> fileSource;
    const MapMode mode;
    mapbox::base::WeakPtr<AnnotationManager> annotationManager;
    std::shared_ptr<ImageManager> imageManager;
    std::shared_ptr<GlyphManager> glyphManager;
    const uint8_t prefetchZoomDelta;
    TaggedScheduler threadPool;
    double tileLodMinRadius = 3;
    double tileLodScale = 1;
    double tileLodPitchThreshold = (60.0 / 180.0) * std::numbers::pi;
    double tileLodZoomShift = 0;
    TileLodMode tileLodMode = TileLodMode::Default;
    gfx::DynamicTextureAtlasPtr dynamicTextureAtlas;
    bool isUpdateSynchronous = false;

    // Drapes multi-resolucion del terreno 3D (ADR 0037): cuando la camara con altura real esta
    // activa, una vista nadir sintetica por drape (centrada en el punto de mira, con su propio
    // zoom) construida una vez por frame en RenderOrchestrator::createRenderTree.
    // TilePyramid::update calcula el covering de cada una SOLO para SourceType::Raster y une sus
    // tiles a los de la camara real (nunca los reemplaza): cada drape tiene imagen real de la
    // zona/resolucion que cubre, como cada render pass offscreen de ATAK.
    std::vector<std::shared_ptr<const TransformState>> drapeTransformStates;
};

} // namespace mln
