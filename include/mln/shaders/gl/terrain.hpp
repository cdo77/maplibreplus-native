// Generated code, do not modify this file!
#pragma once
#include <mln/shaders/shader_source.hpp>

namespace mln {
namespace shaders {

template <>
struct ShaderSource<BuiltIn::TerrainShader, gfx::Backend::Type::OpenGL> {
    static constexpr const char* name = "TerrainShader";
    static constexpr const char* vertex = R"(layout (location = 0) in vec3 a_pos3d;
layout (location = 1) in vec3 a_ecef_pos;
layout (location = 2) in vec3 a_ecef_normal;

layout (std140) uniform TerrainDrawableUBO {
    highp mat4 u_matrix;
    highp mat4 u_terrain_matrix;
    // Drapes multi-resolucion 1x/4x/32x (ADR 0037): tile-local de este sub-tile -> UV de cada
    // render target compartido por TODOS los sub-tiles del frame. Solo validas en u_ecef_mode.
    highp mat4 u_drape_matrix0;
    highp mat4 u_drape_matrix1;
    highp mat4 u_drape_matrix2;
    highp vec4 u_terrain_unpack;
    highp float u_terrain_dim;
    highp float u_terrain_exaggeration;
    highp float u_ele_delta;
    lowp float u_drape_global; // ver TerrainDrawableUBO::drape_global
    // Camara con altura real (ADR 0034): 1.0 = a_ecef_pos/a_ecef_normal validos (WGS84 real,
    // relativos al origen del sub-tile) y drapes activos (ADR 0037), 0.0 = modo planar
    // mercator de siempre (page propia, u_terrain_image).
    highp float u_ecef_mode;
    lowp float drawable_pad2;
    lowp float drawable_pad3;
    lowp float drawable_pad4;
};

uniform sampler2D u_terrain_dem;

out vec2 v_texture_pos;
out vec2 v_drape_uv0;
out vec2 v_drape_uv1;
out vec2 v_drape_uv2;

float terrain_texel_elevation(ivec2 pos) {
    vec4 rgb = (texelFetch(u_terrain_dem, pos, 0) * 255.0) * u_terrain_unpack;
    return rgb.r + rgb.g + rgb.b - u_terrain_unpack.a;
}

float terrain_elevation(vec2 pos) {
    vec2 coord = (u_terrain_matrix * vec4(pos, 0.0, 1.0)).xy * u_terrain_dim + 0.5;
    vec2 f = fract(coord);
    ivec2 c = ivec2(floor(coord));
    ivec2 hi = textureSize(u_terrain_dem, 0) - 1;
    float tl = terrain_texel_elevation(clamp(c, ivec2(0), hi));
    float tr = terrain_texel_elevation(clamp(c + ivec2(1, 0), ivec2(0), hi));
    float bl = terrain_texel_elevation(clamp(c + ivec2(0, 1), ivec2(0), hi));
    float br = terrain_texel_elevation(clamp(c + ivec2(1, 1), ivec2(0), hi));
    return mix(mix(tl, tr, f.x), mix(bl, br, f.x), f.y) * u_terrain_exaggeration;
}

void main() {
    float elevation = terrain_elevation(a_pos3d.xy);
    float ele_delta = a_pos3d.z == 1.0 ? u_ele_delta : 0.0;
    if (u_ecef_mode > 0.5) {
        // Camara con altura real (ADR 0034): posicion real sobre el elipsoide WGS84 (ya relativa
        // al origen del sub-tile, RTE por-tile), desplazada por el relieve a lo largo del normal
        // real en vez de en Z local -- mismo exaggeration/skirt dinamico de siempre.
        vec3 pos = a_ecef_pos + a_ecef_normal * (elevation - ele_delta);
        gl_Position = u_matrix * vec4(pos, 1.0);
        // Drapes (ADR 0037): la textura visual ya NO es "la del propio sub-tile" -- son las
        // capturas compartidas por todo el terreno del frame, proyectadas sobre la malla via
        // tile-local -> world-units -> UV de cada drape (mismo patron de u_terrain_matrix para
        // el DEM, con otro destino). La nitidez depende de los drapes, no del LOD geometrico.
        vec4 local = vec4(a_pos3d.xy, 0.0, 1.0);
        v_drape_uv0 = (u_drape_matrix0 * local).xy;
        v_drape_uv1 = (u_drape_matrix1 * local).xy;
        v_drape_uv2 = (u_drape_matrix2 * local).xy;
        v_texture_pos = vec2(0.0);
    } else {
        gl_Position = u_matrix * vec4(a_pos3d.xy, elevation - ele_delta, 1.0);
        // Modo planar de siempre: page propia por sub-tile (MegaTexture), UV directo.
        v_texture_pos = a_pos3d.xy / 8192.0;
        v_drape_uv0 = vec2(0.0);
        v_drape_uv1 = vec2(0.0);
        v_drape_uv2 = vec2(0.0);
    }
}
)";
    static constexpr const char* fragment = R"(layout (std140) uniform TerrainDrawableUBO {
    highp mat4 u_matrix;
    highp mat4 u_terrain_matrix;
    highp mat4 u_drape_matrix0;
    highp mat4 u_drape_matrix1;
    highp mat4 u_drape_matrix2;
    highp vec4 u_terrain_unpack;
    highp float u_terrain_dim;
    highp float u_terrain_exaggeration;
    highp float u_ele_delta;
    lowp float u_drape_global; // ver TerrainDrawableUBO::drape_global
    highp float u_ecef_mode;
    lowp float drawable_pad2;
    lowp float drawable_pad3;
    lowp float drawable_pad4;
};

uniform sampler2D u_terrain_image;
uniform sampler2D u_terrain_drape0;
uniform sampler2D u_terrain_drape1;
uniform sampler2D u_terrain_drape2;

in vec2 v_texture_pos;
in vec2 v_drape_uv0;
in vec2 v_drape_uv1;
in vec2 v_drape_uv2;

// 1.0 si la UV cae dentro de [0,1] (el drape tiene dato ahi), 0.0 si no -- el mismo corte duro
// por step() que usa ATAK para el alfa del terreno fuera de la captura offscreen.
float drape_inside(vec2 uv) {
    vec2 inside = step(vec2(0.0), uv) * step(uv, vec2(1.0));
    return inside.x * inside.y;
}

// Drape que es el planisferio entero (vista de globo, ADR 0042): la u da la vuelta al mundo y la v se recorta al
// borde -- mas alla de los 85 grados del mercator queda el borde estirado, como los polos de ATAK.
vec2 drape_global_uv(vec2 uv) {
    return vec2(fract(uv.x), clamp(uv.y, 0.0, 1.0));
}

void main() {
    if (u_ecef_mode > 0.5) {
        // Drapes multi-resolucion (ADR 0037): ATAK dibuja el terreno una vez por captura, de la
        // mas gruesa a la mas fina, con GL_BLEND por alfa y alfa 0 fuera de cada una
        // (GLMapView2::drawTerrainTiles + GLTerrainTile.cpp) -- cada punto queda con la captura
        // MAS FINA que tiene imagen ahi; donde la fina todavia no cargo (alfa 0) se ve la gruesa.
        // Aca el mismo resultado en una sola pasada ("over" premultiplicado, como el resto del
        // motor). Las UV de los drapes ya vienen en convencion de textura (norte = v 1, igual que
        // se renderizo el drape): NO se invierten como la UV tile-local del camino planar -- ese
        // flip espejaba la imagen norte-sur (texto espejado en campo).
        // El drape grueso estira su borde fuera de su area (horizonte lejano con la camara baja). En vista de globo
        // los drapes son el planisferio: debajo va azul oceano mientras la imagen no llega (no blanco).
        vec4 color;
        if (u_drape_global > 0.5) {
            vec4 coarse = texture(u_terrain_drape2, drape_global_uv(v_drape_uv2));
            color = vec4(0.07, 0.15, 0.30, 1.0) * (1.0 - coarse.a) + coarse;
        } else {
            color = texture(u_terrain_drape2, clamp(v_drape_uv2, 0.0, 1.0));
        }
        vec4 mid = u_drape_global > 1.5 ? texture(u_terrain_drape1, drape_global_uv(v_drape_uv1))
                                        : texture(u_terrain_drape1, v_drape_uv1) * drape_inside(v_drape_uv1);
        color = color * (1.0 - mid.a) + mid;
        vec4 fine = u_drape_global > 2.5 ? texture(u_terrain_drape0, drape_global_uv(v_drape_uv0))
                                         : texture(u_terrain_drape0, v_drape_uv0) * drape_inside(v_drape_uv0);
        color = color * (1.0 - fine.a) + fine;
        fragColor = color;
    } else {
        fragColor = texture(u_terrain_image, vec2(v_texture_pos.x, 1.0 - v_texture_pos.y));
    }
#ifdef OVERDRAW_INSPECTOR
    fragColor = vec4(1.0);
#endif
}
)";
};

} // namespace shaders
} // namespace mln
