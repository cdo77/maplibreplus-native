// Generated code, do not modify this file!
#pragma once
#include <mln/shaders/shader_source.hpp>

namespace mln {
namespace shaders {

template <>
struct ShaderSource<BuiltIn::SymbolIconShader, gfx::Backend::Type::OpenGL> {
    static constexpr const char* name = "SymbolIconShader";
    static constexpr const char* vertex = R"(layout (location = 0) in vec4 a_pos_offset;
layout (location = 1) in vec4 a_data;
layout (location = 2) in vec4 a_pixeloffset;
layout (location = 3) in vec3 a_projected_pos;
layout (location = 4) in float a_fade_opacity;

layout (std140) uniform GlobalPaintParamsUBO {
    highp vec2 u_pattern_atlas_texsize;
    highp vec2 u_units_to_pixels;
    highp vec2 u_world_size;
    highp float u_camera_to_center_distance;
    highp float u_symbol_fade_change;
    highp float u_aspect_ratio;
    highp float u_pixel_ratio;
    highp float u_map_zoom;
    lowp float global_pad1;
};

layout (std140) uniform SymbolDrawableUBO {
    highp mat4 u_matrix;
    highp mat4 u_label_plane_matrix;
    highp mat4 u_coord_matrix;

    highp vec2 u_texsize;
    highp vec2 u_texsize_icon;

    bool u_is_text_prop;
    bool u_rotate_symbol;
    bool u_pitch_with_map;
    bool u_is_size_zoom_constant;
    bool u_is_size_feature_constant;
    bool u_is_offset;

    highp float u_size_t; // used to interpolate between zoom stops when size is a composite function
    highp float u_size; // used when size is both zoom and feature constant

    // Interpolations
    highp float u_fill_color_t;
    highp float u_halo_color_t;
    highp float u_opacity_t;
    highp float u_halo_width_t;
    highp float u_halo_blur_t;
    highp float u_ecef_heightmap_scale;
    highp vec2 u_ecef_heightmap_offset;
    highp float u_ecef_depth_b;
    highp float u_ecef_stand_px;
    lowp float ecef_pad2;
    lowp float ecef_pad3;
};

layout (std140) uniform SymbolEvaluatedPropsUBO {
    highp vec4 u_text_fill_color;
    highp vec4 u_text_halo_color;
    highp float u_text_opacity;
    highp float u_text_halo_width;
    highp float u_text_halo_blur;
    lowp float props_pad1;
    highp vec4 u_icon_fill_color;
    highp vec4 u_icon_halo_color;
    highp float u_icon_opacity;
    highp float u_icon_halo_width;
    highp float u_icon_halo_blur;
    lowp float props_pad2;
};

out vec2 v_tex;
out float v_fade_opacity;

#ifndef HAS_UNIFORM_u_opacity
layout (location = 5) in lowp vec2 a_opacity;
out lowp float opacity;
#endif

uniform sampler2D u_ecef_heightmap;

// Iconos 3D con camara real (ADR 0038): elevacion del terreno bajo el ancla, del mapa de alturas de
// RenderTerrain (terrarium RGBA8; bilineal a mano con texelFetch como la DEM del terreno -- filtrar
// lineal los bytes codificados daria alturas falsas).
float ecef_height_texel(ivec2 p, ivec2 hi) {
    vec3 rgb = texelFetch(u_ecef_heightmap, clamp(p, ivec2(0), hi), 0).rgb * 255.0;
    return rgb.r * 256.0 + rgb.g + rgb.b / 256.0 - 32768.0;
}

float ecef_anchor_elevation(vec2 anchor) {
    ivec2 dim = textureSize(u_ecef_heightmap, 0);
    vec2 coord = (anchor * u_ecef_heightmap_scale + u_ecef_heightmap_offset) * vec2(dim) - 0.5;
    ivec2 c = ivec2(floor(coord));
    vec2 f = fract(coord);
    ivec2 hi = dim - 1;
    float tl = ecef_height_texel(c, hi);
    float tr = ecef_height_texel(c + ivec2(1, 0), hi);
    float bl = ecef_height_texel(c + ivec2(0, 1), hi);
    float br = ecef_height_texel(c + ivec2(1, 1), hi);
    return mix(mix(tl, tr, f.x), mix(bl, br, f.x), f.y);
}

void main() {
    highp float u_opacity = u_is_text_prop ? u_text_opacity : u_icon_opacity;

    #ifndef HAS_UNIFORM_u_opacity
opacity = unpack_mix_vec2(a_opacity, u_opacity_t);
#else
lowp float opacity = u_opacity;
#endif

    vec2 a_pos = a_pos_offset.xy;
    vec2 a_offset = a_pos_offset.zw;

    vec2 a_tex = a_data.xy;
    vec2 a_size = a_data.zw;

    float a_size_min = floor(a_size[0] * 0.5);
    vec2 a_pxoffset = a_pixeloffset.xy;
    vec2 a_minFontScale = a_pixeloffset.zw / 256.0;

    highp float segment_angle = -a_projected_pos[2];
    float size;

    if (!u_is_size_zoom_constant && !u_is_size_feature_constant) {
        size = mix(a_size_min, a_size[1], u_size_t) / 128.0;
    } else if (u_is_size_zoom_constant && !u_is_size_feature_constant) {
        size = a_size_min / 128.0;
    } else {
        size = u_size;
    }

    bool ecef_mode = u_ecef_heightmap_scale > 0.0;
    float anchor_elevation = ecef_mode ? ecef_anchor_elevation(a_pos) : 0.0;
    vec4 projectedPoint = u_matrix * vec4(a_pos, anchor_elevation, 1);
    highp float camera_to_anchor_distance = projectedPoint.w;
    // See comments in symbol_sdf.vertex
    highp float distance_ratio = u_pitch_with_map ?
        camera_to_anchor_distance / u_camera_to_center_distance :
        u_camera_to_center_distance / camera_to_anchor_distance;
    highp float perspective_ratio = clamp(
            0.5 + 0.5 * distance_ratio,
            0.0, // Prevents oversized near-field symbols in pitched/overzoomed tiles
            4.0);

    // Camara real: tamano fijo en pixeles, como los sprites de ATAK.
    if (!u_is_offset && !ecef_mode) {
        size *= perspective_ratio;
    }

    float fontScale = u_is_text_prop ? size / 24.0 : size;

    highp float symbol_rotation = 0.0;
    if (u_rotate_symbol) {
        // See comments in symbol_sdf.vertex
        vec4 offsetProjectedPoint = u_matrix * vec4(a_pos + vec2(1, 0), anchor_elevation, 1);

        vec2 a = projectedPoint.xy / projectedPoint.w;
        vec2 b = offsetProjectedPoint.xy / offsetProjectedPoint.w;

        symbol_rotation = atan((b.y - a.y) / u_aspect_ratio, b.x - a.x);
    }

    highp float angle_sin = sin(segment_angle + symbol_rotation);
    highp float angle_cos = cos(segment_angle + symbol_rotation);
    mat2 rotation_matrix = mat2(angle_cos, -1.0 * angle_sin, angle_sin, angle_cos);

    // Camara real: el ancla (con su elevacion) ya la proyecto u_matrix; la label plane pasa de
    // NDC a pixeles.
    vec4 projected_pos = ecef_mode
        ? u_label_plane_matrix * vec4(projectedPoint.xy / projectedPoint.w, 0.0, 1.0)
        : u_label_plane_matrix * vec4(a_projected_pos.xy, 0.0, 1.0);
    // Camara real inclinada: el icono se para sobre el ancla (sube u_ecef_stand_px en pantalla), como ATAK.
    if (ecef_mode) {
        projected_pos.y -= u_ecef_stand_px * projected_pos.w;
    }
    gl_Position = u_coord_matrix * vec4(projected_pos.xy / projected_pos.w + rotation_matrix * (a_offset / 32.0 * max(a_minFontScale, fontScale) + a_pxoffset / 16.0), 0.0, 1.0);
    if (ecef_mode) {
        // Como los sprites de ATAK (BatchGeometryPoints.vert), el icono se adelanta hacia la camara su propio
        // radio en metros -- radio en px x metros/px a esa distancia --, asi el relieve no lo tapa aunque el
        // ancla quede unos metros bajo la malla. Escalar el punto por s alrededor del ojo deja x, y, w
        // multiplicados por s y z = s (z - B) + B; s = 1 - radio/distancia = 1 - radio_px tan(fov/2) / (alto/2)
        // (fov 45 de la camara real, alto/2 = -1/u_coord_matrix[1][1]). Detras de la camara (w <= 0): fuera.
        float radius_px = length(a_offset / 32.0 * max(a_minFontScale, fontScale));
        float half_height_px = -1.0 / u_coord_matrix[1][1];
        float s = max(1.0 - radius_px * 0.41421356 / half_height_px, 0.05);
        gl_Position.z = projectedPoint.w > 0.0
            ? (s * (projectedPoint.z - u_ecef_depth_b) + u_ecef_depth_b) / (s * projectedPoint.w)
            : 2.0;
    }

    v_tex = a_tex / u_texsize;
    vec2 fade_opacity = unpack_opacity(a_fade_opacity);
    float fade_change = fade_opacity[1] > 0.5 ? u_symbol_fade_change : -u_symbol_fade_change;
    v_fade_opacity = max(0.0, min(1.0, fade_opacity[0] + fade_change));
}
)";
    static constexpr const char* fragment = R"(uniform sampler2D u_texture;

layout (std140) uniform SymbolTilePropsUBO {
    bool u_is_text;
    bool u_is_halo;
    highp float u_gamma_scale;
    lowp float tileprops_pad1;
};

layout (std140) uniform SymbolEvaluatedPropsUBO {
    highp vec4 u_text_fill_color;
    highp vec4 u_text_halo_color;
    highp float u_text_opacity;
    highp float u_text_halo_width;
    highp float u_text_halo_blur;
    lowp float props_pad1;
    highp vec4 u_icon_fill_color;
    highp vec4 u_icon_halo_color;
    highp float u_icon_opacity;
    highp float u_icon_halo_width;
    highp float u_icon_halo_blur;
    lowp float props_pad2;
};

in vec2 v_tex;
in float v_fade_opacity;

#ifndef HAS_UNIFORM_u_opacity
in lowp float opacity;
#endif

void main() {
    highp float u_opacity = u_is_text ? u_text_opacity : u_icon_opacity;

    #ifdef HAS_UNIFORM_u_opacity
lowp float opacity = u_opacity;
#endif

    lowp float alpha = opacity * v_fade_opacity;
    fragColor = texture(u_texture, v_tex) * alpha;

#ifdef OVERDRAW_INSPECTOR
    fragColor = vec4(1.0);
#endif
}
)";
};

} // namespace shaders
} // namespace mln
