#include <mln/renderer/layers/symbol_layer_tweaker.hpp>

#include <mln/gfx/context.hpp>
#include <mln/gfx/drawable.hpp>
#include <mln/gfx/renderable.hpp>
#include <mln/gfx/renderer_backend.hpp>
#include <mln/gfx/symbol_drawable_data.hpp>
#include <mln/layout/symbol_projection.hpp>
#include <mln/renderer/buckets/symbol_bucket.hpp>
#include <mln/renderer/layer_group.hpp>
#include <mln/renderer/paint_parameters.hpp>
#include <mln/renderer/paint_property_binder.hpp>
#include <mln/renderer/layers/render_symbol_layer.hpp>
#include <mln/renderer/render_tree.hpp>
#include <mln/shaders/shader_program_base.hpp>
#include <mln/shaders/symbol_layer_ubo.hpp>
#include <mln/style/layers/symbol_layer_properties.hpp>
#include <mln/util/convert.hpp>
#include <mln/util/std.hpp>
#include <mln/math/angles.hpp>
#include <mln/util/constants.hpp>
#include <mln/util/ecef.hpp>
#include <mln/util/globe.hpp>

#include <cmath>

#if MLN_RENDER_BACKEND_METAL
#include <mln/shaders/mtl/symbol.hpp>
#endif // MLN_RENDER_BACKEND_METAL

namespace mln {

using namespace style;
using namespace shaders;

namespace {

Size getTexSize(const gfx::Drawable& drawable, const size_t texId) {
    if (const auto& tex = drawable.getTexture(texId)) {
        return tex->getSize();
    }
    return {0, 0};
}

std::array<float, 2> toArray(const Size& s) {
    return util::cast<float>(std::array<uint32_t, 2>{s.width, s.height});
}

template <typename TText, typename TIcon>
const auto& getProperty(const SymbolBucket::PaintProperties& paintProps, bool isText) {
    return isText ? paintProps.textBinders.get<TText>() : paintProps.iconBinders.get<TIcon>();
}

template <typename TText, typename TIcon, std::size_t N>
auto getInterpFactor(const SymbolBucket::PaintProperties& paintProps, bool isText, float currentZoom) {
    return std::get<N>(getProperty<TText, TIcon>(paintProps, isText)->interpolationFactor(currentZoom));
}

// Tile-local (x, y en [0,EXTENT], z = metros sobre el elipsoide) -> clip de la camara con altura real
// (ADR 0038): matriz RTE de la camara (TransformState::getEcefTileMatrix) por una afin local alrededor
// de la esquina NO del tile -- este y sur en metros por unidad de tile, arriba en metros. En un tile
// z14 (~2,4 km) el error por curvatura es de ~0,5 m.
mat4 ecefSpriteTileMatrix(const UnwrappedTileID& tileID,
                          const TransformState& state,
                          const TransformState::EcefCamera& camera) {
    namespace ecef = util::ecef;
    namespace globe = util::globe;

    const LatLng northWest(tileID.canonical);
    const vec3 origin = ecef::llaToEcef(northWest, 0.0);
    const vec3 up = ecef::surfaceNormal(northWest);
    const vec3 east = globe::normalize(globe::cross(vec3{0.0, 0.0, 1.0}, up));
    const vec3 north = globe::cross(up, east);
    const double metersPerUnit = util::M2PI * util::EARTH_RADIUS_M * std::cos(util::deg2rad(northWest.latitude())) /
                                 std::pow(2.0, tileID.canonical.z) / util::EXTENT;

    mat4 local = matrix::identity4();
    for (size_t row = 0; row < 3; ++row) {
        local[0 * 4 + row] = east[row] * metersPerUnit;
        local[1 * 4 + row] = -north[row] * metersPerUnit;
        local[2 * 4 + row] = up[row];
    }
    mat4 result;
    matrix::multiply(result, state.getEcefTileMatrix(origin, camera), local);
    return result;
}

} // namespace

void SymbolLayerTweaker::execute(LayerGroupBase& layerGroup, const PaintParameters& parameters) {
    if (layerGroup.empty()) {
        return;
    }

    auto& context = parameters.context;
    const auto& state = parameters.state;
    const auto& symbolLayerProperties = static_cast<const SymbolLayerProperties&>(*evaluatedProperties);
    const auto& evaluated = symbolLayerProperties.evaluated;

#if !defined(NDEBUG)
    const auto label = layerGroup.getName() + "-update-uniforms";
    const auto debugGroup = parameters.encoder->createDebugGroup(label.c_str());
#endif

    const auto zoom = static_cast<float>(state.getZoom());

    if (!evaluatedPropsUniformBuffer || propertiesUpdated) {
        const SymbolEvaluatedPropsUBO propsUBO = {.text_fill_color = constOrDefault<TextColor>(evaluated),
                                                  .text_halo_color = constOrDefault<TextHaloColor>(evaluated),
                                                  .text_opacity = constOrDefault<TextOpacity>(evaluated),
                                                  .text_halo_width = constOrDefault<TextHaloWidth>(evaluated),
                                                  .text_halo_blur = constOrDefault<TextHaloBlur>(evaluated),
                                                  .pad1 = 0,

                                                  .icon_fill_color = constOrDefault<IconColor>(evaluated),
                                                  .icon_halo_color = constOrDefault<IconHaloColor>(evaluated),
                                                  .icon_opacity = constOrDefault<IconOpacity>(evaluated),
                                                  .icon_halo_width = constOrDefault<IconHaloWidth>(evaluated),
                                                  .icon_halo_blur = constOrDefault<IconHaloBlur>(evaluated),
                                                  .pad2 = 0};
        context.emplaceOrUpdateUniformBuffer(evaluatedPropsUniformBuffer, &propsUBO);
        propertiesUpdated = false;
    }
    auto& layerUniforms = layerGroup.mutableUniformBuffers();
    layerUniforms.set(idSymbolEvaluatedPropsUBO, evaluatedPropsUniformBuffer);

#if MLN_UBO_CONSOLIDATION
    int i = 0;
    std::vector<SymbolDrawableUBO> drawableUBOVector(layerGroup.getDrawableCount());
    std::vector<SymbolTilePropsUBO> tilePropsUBOVector(layerGroup.getDrawableCount());
#endif

    const auto camDist = state.getCameraToCenterDistance();
    const auto screenSpaceProp = symbolLayerProperties.layerImpl().layout.get<SymbolScreenSpace>();
    const auto isScreenSpace = screenSpaceProp.isConstant() ? screenSpaceProp.asConstant()
                                                            : SymbolScreenSpace::defaultValue();

    visitLayerGroupDrawables(layerGroup, [&](gfx::Drawable& drawable) {
        if (!drawable.getTileID() || !drawable.getData()) {
            return;
        }

        const auto tileID = drawable.getTileID()->toUnwrapped();
        const auto& symbolData = static_cast<gfx::SymbolDrawableData&>(*drawable.getData());
        const auto isText = (symbolData.symbolType == SymbolType::Text);

        const auto* textBinders = isText ? static_cast<SymbolTextBinders*>(drawable.getBinders()) : nullptr;
        const auto* iconBinders = isText ? nullptr : static_cast<SymbolIconBinders*>(drawable.getBinders());

        const auto bucket = std::static_pointer_cast<SymbolBucket>(drawable.getBucket());
        const auto* tile = drawable.getRenderTile();
        if (!bucket || !tile || (!textBinders && !iconBinders)) {
            assert(false);
            return;
        }

        const auto& paintProperties = bucket->paintProperties.at(id);

        // from RenderTile::translatedMatrix
        const auto translate = isText ? evaluated.get<style::TextTranslate>() : evaluated.get<style::IconTranslate>();

        mat4 matrix;

        if (isScreenSpace) {
            matrix::ortho(matrix, 0, util::EXTENT, -util::EXTENT, 0, 0, 1);
            matrix::translate(matrix, matrix, 0, -util::EXTENT, 0);
            matrix::translate(matrix, matrix, translate[0], translate[1], 0);
        } else {
            constexpr bool nearClipped = false;
            constexpr bool inViewportPixelUnits = false;
            const auto anchor = isText ? evaluated.get<style::TextTranslateAnchor>()
                                       : evaluated.get<style::IconTranslateAnchor>();
            matrix = getTileMatrix(tileID, parameters, translate, anchor, nearClipped, inViewportPixelUnits, drawable);
        }

        // Iconos 3D con camara real (ADR 0038, pass Sprites de ATAK): el icono deja de ir en los drapes
        // y se dibuja despues del terreno como billboard anclado al relieve. u_matrix pasa a ser
        // tile-local (x, y, z = metros de elevacion) -> clip de la camara real; la elevacion de cada
        // ancla la pone el shader desde el mapa de alturas del terreno.
        const bool ecefSprites = !isScreenSpace && parameters.ecefHeightmap && parameters.ecefCamera &&
                                 state.isRealAltitudeModeEnabled();
        float ecefHeightmapScale = 0.0f;
        float ecefDepthB = 0.0f;
        float ecefStandPx = 0.0f;
        std::array<float, 2> ecefHeightmapOffset = {0.0f, 0.0f};
        if (ecefSprites) {
            matrix = ecefSpriteTileMatrix(tileID, state, *parameters.ecefCamera);
            const double nearM = parameters.ecefCamera->nearM;
            const double farM = parameters.ecefCamera->farM;
            ecefDepthB = static_cast<float>(2.0 * farM * nearM / (nearM - farM));
            // Con la vista inclinada el icono se para sobre su ancla en vez de centrarse en ella, como GLMarker2
            // de ATAK (updateDrawPosition): sube media altura uniforme de icono (48 px) x sin(tilt), asi el suelo
            // cercano -- que en pantalla queda debajo del ancla -- no le tapa la mitad inferior.
            constexpr double kUniformIconHalfHeightPx = 24.0;
            ecefStandPx = static_cast<float>(kUniformIconHalfHeightPx * std::sin(state.getPitch()));
            const auto& heightmapBounds = parameters.ecefHeightmapMercator;
            const double tilesAtZ = std::pow(2.0, tileID.canonical.z);
            const double tx0 = (tileID.canonical.x + (tileID.wrap * tilesAtZ)) / tilesAtZ;
            const double ty0 = tileID.canonical.y / tilesAtZ;
            ecefHeightmapScale = static_cast<float>(1.0 / (util::EXTENT * tilesAtZ * heightmapBounds[2]));
            ecefHeightmapOffset = {static_cast<float>((tx0 - heightmapBounds[0]) / heightmapBounds[2]),
                                   static_cast<float>((ty0 - heightmapBounds[1]) / heightmapBounds[2])};
            drawable.setTexture(parameters.ecefHeightmap, idSymbolEcefHeightmapTexture);
        }

        // from symbol_program, makeValues
        const auto currentZoom = static_cast<float>(parameters.state.getZoom());
        const float pixelsToTileUnits = tileID.pixelsToTileUnits(1.f, currentZoom);
        const bool pitchWithMap = !ecefSprites && symbolData.pitchAlignment == style::AlignmentType::Map;
        const bool rotateWithMap = symbolData.rotationAlignment == style::AlignmentType::Map;
        const bool alongLine = symbolData.placement != SymbolPlacementType::Point &&
                               symbolData.rotationAlignment == AlignmentType::Map;
        const bool hasVariablePlacement = symbolData.bucketVariablePlacement &&
                                          (isText || symbolData.textFit != IconTextFitType::None);
        // Camara real: la label plane solo pasa de NDC a pixeles (el ancla ya la proyecto u_matrix).
        const mat4 labelPlaneMatrix =
            ecefSprites ? getLabelPlaneMatrix(matrix::identity4(), false, false, state, pixelsToTileUnits)
            : (alongLine || hasVariablePlacement)
                ? matrix::identity4()
                : getLabelPlaneMatrix(matrix, pitchWithMap, rotateWithMap, state, pixelsToTileUnits);
        const mat4 glCoordMatrix = getGlCoordMatrix(matrix, pitchWithMap, rotateWithMap, state, pixelsToTileUnits);

        const float gammaScale = (pitchWithMap
                                      ? static_cast<float>(std::cos(state.getPitch())) * camDist
                                      : 1.0f);

        // Line label rotation happens in `updateLineLabels`/`reprojectLineLabels``
        // Pitched point labels are automatically rotated by the labelPlaneMatrix projection
        // Unpitched point labels need to have their rotation applied after projection
        const bool rotateInShader = rotateWithMap && !pitchWithMap && !alongLine;

        const auto& sizeBinder = isText ? bucket->textSizeBinder : bucket->iconSizeBinder;
        const auto size = sizeBinder->evaluateForZoom(currentZoom);

#if MLN_UBO_CONSOLIDATION
        drawableUBOVector[i] = {
#else
        const SymbolDrawableUBO drawableUBO = {
#endif
            .matrix = util::cast<float>(matrix),
            .label_plane_matrix = util::cast<float>(labelPlaneMatrix),
            .coord_matrix = util::cast<float>(glCoordMatrix),

            .texsize = toArray(getTexSize(drawable, idSymbolImageTexture)),
            .texsize_icon = toArray(getTexSize(drawable, idSymbolImageIconTexture)),

            .is_text_prop = isText,
            .rotate_symbol = rotateInShader,
            .pitch_with_map = pitchWithMap,
            .is_size_zoom_constant = size.isZoomConstant,
            .is_size_feature_constant = size.isFeatureConstant,
            .is_offset = symbolData.isOffset,

            .size_t = size.sizeT,
            .size = size.size,

            .fill_color_t = getInterpFactor<TextColor, IconColor, 0>(paintProperties, isText, zoom),
            .halo_color_t = getInterpFactor<TextHaloColor, IconHaloColor, 0>(paintProperties, isText, zoom),
            .opacity_t = getInterpFactor<TextOpacity, IconOpacity, 0>(paintProperties, isText, zoom),
            .halo_width_t = getInterpFactor<TextHaloWidth, IconHaloWidth, 0>(paintProperties, isText, zoom),
            .halo_blur_t = getInterpFactor<TextHaloBlur, IconHaloBlur, 0>(paintProperties, isText, zoom),
            .ecef_heightmap_scale = ecefHeightmapScale,
            .ecef_heightmap_offset = ecefHeightmapOffset,
            .ecef_depth_b = ecefDepthB,
            .ecef_stand_px = ecefStandPx,
            .ecef_pad2 = 0.0f,
            .ecef_pad3 = 0.0f,
        };

#if MLN_UBO_CONSOLIDATION
        tilePropsUBOVector[i] = {
#else
        const SymbolTilePropsUBO tilePropsUBO = {
#endif
            .is_text = isText,
            .is_halo = symbolData.isHalo,
            .gamma_scale = gammaScale,
            .pad1 = 0,
        };

#if MLN_UBO_CONSOLIDATION
        drawable.setUBOIndex(i++);
#else
        auto& drawableUniforms = drawable.mutableUniformBuffers();
        drawableUniforms.createOrUpdate(idSymbolDrawableUBO, &drawableUBO, context);
        drawableUniforms.createOrUpdate(idSymbolTilePropsUBO, &tilePropsUBO, context);
#endif
    });

#if MLN_UBO_CONSOLIDATION
    const size_t drawableUBOVectorSize = sizeof(SymbolDrawableUBO) * drawableUBOVector.size();
    if (!drawableUniformBuffer || drawableUniformBuffer->getSize() < drawableUBOVectorSize) {
        drawableUniformBuffer = context.createUniformBuffer(
            drawableUBOVector.data(), drawableUBOVectorSize, false, true);
    } else {
        drawableUniformBuffer->update(drawableUBOVector.data(), drawableUBOVectorSize);
    }

    const size_t tilePropsUBOVectorSize = sizeof(SymbolTilePropsUBO) * tilePropsUBOVector.size();
    if (!tilePropsUniformBuffer || tilePropsUniformBuffer->getSize() < tilePropsUBOVectorSize) {
        tilePropsUniformBuffer = context.createUniformBuffer(
            tilePropsUBOVector.data(), tilePropsUBOVectorSize, false, true);
    } else {
        tilePropsUniformBuffer->update(tilePropsUBOVector.data(), tilePropsUBOVectorSize);
    }

    layerUniforms.set(idSymbolDrawableUBO, drawableUniformBuffer);
    layerUniforms.set(idSymbolTilePropsUBO, tilePropsUniformBuffer);
#endif
}

} // namespace mln
