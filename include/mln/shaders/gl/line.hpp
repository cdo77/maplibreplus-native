// Generated code, do not modify this file!
#pragma once
#include <mln/shaders/shader_source.hpp>

namespace mln {
namespace shaders {

template <>
struct ShaderSource<BuiltIn::LineShader, gfx::Backend::Type::OpenGL> {
    static constexpr const char* name = "LineShader";
    static constexpr const char* vertex = R"(// floor(127 / 2) == 63.0
// the maximum allowed miter limit is 2.0 at the moment. the extrude normal is
// stored in a byte (-128..127). we scale regular normals up to length 63, but
// there are also "special" normals that have a bigger length (of up to 126 in
// this case).
// #define scale 63.0
#define scale 0.015873016

layout (location = 0) in vec2 a_pos_normal;
layout (location = 1) in vec4 a_data;

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

layout (std140) uniform LineDrawableUBO {
    highp mat4 u_matrix;
    mediump float u_ratio;
    // Interpolations
    lowp float u_color_t;
    lowp float u_blur_t;
    lowp float u_opacity_t;
    lowp float u_gapwidth_t;
    lowp float u_offset_t;
    lowp float u_width_t;
    lowp float drawable_pad1;
    highp float u_ecef_heightmap_scale;
    lowp float ecef_pad0;
    highp vec2 u_ecef_heightmap_offset;
    highp float u_ecef_depth_b;
    lowp float ecef_pad1;
    lowp float ecef_pad2;
    lowp float ecef_pad3;
};

layout (std140) uniform LineEvaluatedPropsUBO {
    highp vec4 u_color;
    lowp float u_blur;
    lowp float u_opacity;
    mediump float u_gapwidth;
    lowp float u_offset;
    mediump float u_width;
    lowp float u_floorwidth;
    lowp float props_pad1;
    lowp float props_pad2;
};

uniform sampler2D u_ecef_heightmap;

// Elevacion del terreno bajo un punto tile-local (ADR 0058), igual que los sprites (ADR 0038): bilineal a mano sobre el
// mapa de alturas terrarium; fuera del mapa se toma 0.
float ecef_height_texel(ivec2 p, ivec2 hi) {
    vec3 rgb = texelFetch(u_ecef_heightmap, clamp(p, ivec2(0), hi), 0).rgb * 255.0;
    return rgb.r * 256.0 + rgb.g + rgb.b / 256.0 - 32768.0;
}

float ecef_point_elevation(vec2 p) {
    vec2 uv = p * u_ecef_heightmap_scale + u_ecef_heightmap_offset;
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) {
        return 0.0;
    }
    ivec2 dim = textureSize(u_ecef_heightmap, 0);
    vec2 coord = uv * vec2(dim) - 0.5;
    ivec2 c = ivec2(floor(coord));
    vec2 f = fract(coord);
    ivec2 hi = dim - 1;
    float tl = ecef_height_texel(c, hi);
    float tr = ecef_height_texel(c + ivec2(1, 0), hi);
    float bl = ecef_height_texel(c + ivec2(0, 1), hi);
    float br = ecef_height_texel(c + ivec2(1, 1), hi);
    return mix(mix(tl, tr, f.x), mix(bl, br, f.x), f.y);
}

out vec2 v_normal;
out vec2 v_width2;
out float v_gamma_scale;
out highp float v_linesofar;
// Posicion tile-local del punto de la linea (ADR 0058): el pase de la camara real no recorta por stencil, asi que el fragmento
// recorta cada tesela a su propia extension; sin esto se ven los bordes del buffer del GeoJSON como cuadrados. 4096 = adentro.
out vec2 v_tile_pos;

#ifndef HAS_UNIFORM_u_color
layout (location = 2) in highp vec4 a_color;
out highp vec4 color;
#endif
#ifndef HAS_UNIFORM_u_blur
layout (location = 3) in lowp vec2 a_blur;
out lowp float blur;
#endif
#ifndef HAS_UNIFORM_u_opacity
layout (location = 4) in lowp vec2 a_opacity;
out lowp float opacity;
#endif
#ifndef HAS_UNIFORM_u_gapwidth
layout (location = 5) in mediump vec2 a_gapwidth;
#endif
#ifndef HAS_UNIFORM_u_offset
layout (location = 6) in lowp vec2 a_offset;
#endif
#ifndef HAS_UNIFORM_u_width
layout (location = 7) in mediump vec2 a_width;
#endif

void main() {
    #ifndef HAS_UNIFORM_u_color
color = unpack_mix_color(a_color, u_color_t);
#else
highp vec4 color = u_color;
#endif
    #ifndef HAS_UNIFORM_u_blur
blur = unpack_mix_vec2(a_blur, u_blur_t);
#else
lowp float blur = u_blur;
#endif
    #ifndef HAS_UNIFORM_u_opacity
opacity = unpack_mix_vec2(a_opacity, u_opacity_t);
#else
lowp float opacity = u_opacity;
#endif
    #ifndef HAS_UNIFORM_u_gapwidth
mediump float gapwidth = unpack_mix_vec2(a_gapwidth, u_gapwidth_t);
#else
mediump float gapwidth = u_gapwidth;
#endif
    #ifndef HAS_UNIFORM_u_offset
lowp float offset = unpack_mix_vec2(a_offset, u_offset_t);
#else
lowp float offset = u_offset;
#endif
    #ifndef HAS_UNIFORM_u_width
mediump float width = unpack_mix_vec2(a_width, u_width_t);
#else
mediump float width = u_width;
#endif

    // the distance over which the line edge fades out.
    // Retina devices need a smaller distance to avoid aliasing.
    float ANTIALIASING = 1.0 / DEVICE_PIXEL_RATIO / 2.0;

    vec2 a_extrude = a_data.xy - 128.0;
    float a_direction = mod(a_data.z, 4.0) - 1.0;

    v_linesofar = (floor(a_data.z / 4.0) + a_data.w * 64.0) * 2.0;

    vec2 pos = floor(a_pos_normal * 0.5);

    // x is 1 if it's a round cap, 0 otherwise
    // y is 1 if the normal points up, and -1 if it points down
    // We store these in the least significant bit of a_pos_normal
    mediump vec2 normal = a_pos_normal - 2.0 * pos;
    normal.y = normal.y * 2.0 - 1.0;
    v_normal = normal;

    // these transformations used to be applied in the JS and native code bases.
    // moved them into the shader for clarity and simplicity.
    gapwidth = gapwidth / 2.0;
    float halfwidth = width / 2.0;
    offset = -1.0 * offset;

    float inset = gapwidth + (gapwidth > 0.0 ? ANTIALIASING : 0.0);
    float outset = gapwidth + halfwidth * (gapwidth > 0.0 ? 2.0 : 1.0) + (halfwidth == 0.0 ? 0.0 : ANTIALIASING);

    // Scale the extrusion vector down to a normal and then up by the line width
    // of this vertex.
    mediump vec2 dist = outset * a_extrude * scale;

    // Calculate the offset when drawing a line that is to the side of the actual line.
    // We do this by creating a vector that points towards the extrude, but rotate
    // it when we're drawing round end points (a_direction = -1 or 1) since their
    // extrude vector points in another direction.
    mediump float u = 0.5 * a_direction;
    mediump float t = 1.0 - abs(u);
    mediump vec2 offset2 = offset * a_extrude * scale * normal.y * mat2(t, -u, u, t);

    vec4 projected_extrude = u_matrix * vec4(dist / u_ratio, 0.0, 0.0);
    gl_Position = u_matrix * vec4(pos + offset2 / u_ratio, 0.0, 1.0) + projected_extrude;

    // calculate how much the perspective view squishes or stretches the extrude
    float extrude_length_without_perspective = length(dist);
    float extrude_length_with_perspective = length(projected_extrude.xy / gl_Position.w * u_units_to_pixels);
    v_gamma_scale = extrude_length_without_perspective / extrude_length_with_perspective;

    v_width2 = vec2(outset, inset);
    v_tile_pos = vec2(4096.0);

    // Pase de lineas de la camara real (ADR 0058): la linea se pega al relieve punto a punto y el ancho se abre en PANTALLA
    // (pixeles fijos), perpendicular a la linea proyectada; con profundidad contra la malla. Sin drapes: sin bordes dobles.
    if (u_ecef_heightmap_scale > 0.0) {
        v_tile_pos = pos;
        float el = ecef_point_elevation(pos) + 0.5;
        vec4 c0 = u_matrix * vec4(pos, el, 1.0);
        vec2 ex = a_extrude * scale;
        float exl = length(ex);
        vec2 ed = exl > 0.0001 ? ex / exl : vec2(0.0, 1.0);
        vec4 c1 = u_matrix * vec4(pos + ed * 64.0, el, 1.0);
        vec2 s_px = (c1.xy / c1.w - c0.xy / c0.w) * u_units_to_pixels;
        float sl = length(s_px);
        vec2 dir_px = sl > 0.000001 ? s_px / sl : vec2(0.0);
        vec2 e_px = dir_px * outset * exl;
        gl_Position = c0;
        gl_Position.xy += (e_px / u_units_to_pixels) * c0.w;
        v_gamma_scale = 1.0;
        // Como los sprites: se adelanta unos pixeles hacia la camara para no enterrarse en la malla.
        float half_height_px = abs(u_units_to_pixels.y);
        float sd = max(1.0 - 8.0 * 0.41421356 / half_height_px, 0.05);
        gl_Position.z = (sd * (c0.z - u_ecef_depth_b) + u_ecef_depth_b) / sd;
        if (c0.w <= 0.0) {
            gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        }
    }
}
)";
    static constexpr const char* fragment = R"(layout (std140) uniform LineEvaluatedPropsUBO {
    highp vec4 u_color;
    lowp float u_blur;
    lowp float u_opacity;
    mediump float u_gapwidth;
    lowp float u_offset;
    mediump float u_width;
    lowp float u_floorwidth;
    lowp float props_pad1;
    lowp float props_pad2;
};

in vec2 v_width2;
in vec2 v_normal;
in float v_gamma_scale;
in vec2 v_tile_pos;

#ifndef HAS_UNIFORM_u_color
in highp vec4 color;
#endif
#ifndef HAS_UNIFORM_u_blur
in lowp float blur;
#endif
#ifndef HAS_UNIFORM_u_opacity
in lowp float opacity;
#endif

void main() {
    #ifdef HAS_UNIFORM_u_color
highp vec4 color = u_color;
#endif
    #ifdef HAS_UNIFORM_u_blur
lowp float blur = u_blur;
#endif
    #ifdef HAS_UNIFORM_u_opacity
lowp float opacity = u_opacity;
#endif

    // Recorte a la extension del tesela (solo el pase de la camara real lo pone fuera de [0, 8192]).
    if (v_tile_pos.x < 0.0 || v_tile_pos.y < 0.0 || v_tile_pos.x > 8192.0 || v_tile_pos.y > 8192.0) {
        discard;
    }

    // Calculate the distance of the pixel from the line in pixels.
    float dist = length(v_normal) * v_width2.s;

    // Calculate the antialiasing fade factor. This is either when fading in
    // the line in case of an offset line (v_width2.t) or when fading out
    // (v_width2.s)
    float blur2 = (blur + 1.0 / DEVICE_PIXEL_RATIO) * v_gamma_scale;
    float alpha = clamp(min(dist - (v_width2.t - blur2), v_width2.s - dist) / blur2, 0.0, 1.0);

    fragColor = color * (alpha * opacity);

#ifdef OVERDRAW_INSPECTOR
    fragColor = vec4(1.0);
#endif
}
)";
};

} // namespace shaders
} // namespace mln
