// Generated code, do not modify this file!
#pragma once
#include <mln/shaders/shader_source.hpp>

namespace mln {
namespace shaders {

template <>
struct ShaderSource<BuiltIn::TerrainShader, gfx::Backend::Type::OpenGL> {
    static constexpr const char* name = "TerrainShader";
    static constexpr const char* vertex = R"(layout (location = 0) in vec3 a_pos3d;

layout (std140) uniform TerrainDrawableUBO {
    highp mat4 u_matrix;
    highp mat4 u_terrain_matrix;
    highp vec4 u_terrain_unpack;
    highp float u_terrain_dim;
    highp float u_terrain_exaggeration;
    highp float u_ele_delta;
    highp float u_center_elevation;
};

uniform sampler2D u_terrain_dem;

out vec2 v_texture_pos;

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
    v_texture_pos = a_pos3d.xy / 8192.0;
    gl_Position = u_matrix * vec4(a_pos3d.xy, elevation - u_center_elevation - ele_delta, 1.0);
}
)";
    static constexpr const char* fragment = R"(uniform sampler2D u_terrain_image;

in vec2 v_texture_pos;

void main() {
    fragColor = texture(u_terrain_image, vec2(v_texture_pos.x, 1.0 - v_texture_pos.y));
#ifdef OVERDRAW_INSPECTOR
    fragColor = vec4(1.0);
#endif
}
)";
};

} // namespace shaders
} // namespace mln
