layout (location = 0) in vec3 a_pos3d;

layout (std140) uniform TerrainDepthDrawableUBO {
    highp mat4 u_matrix;
    highp mat4 u_terrain_matrix;
    // sin uso aca: mismo struct C++ que TerrainDrawableUBO (ADR 0037)
    highp mat4 u_drape_matrix0;
    highp mat4 u_drape_matrix1;
    highp mat4 u_drape_matrix2;
    highp vec4 u_terrain_unpack;
    highp float u_terrain_dim;
    highp float u_terrain_exaggeration;
    highp float u_ele_delta;
    lowp float drawable_pad1;
    lowp float drawable_pad_ecef; // ver TerrainDrawableUBO (mismo struct C++, ADR 0034)
    lowp float drawable_pad2;
    lowp float drawable_pad3;
    lowp float drawable_pad4;
};

uniform sampler2D u_terrain_dem;

out float v_depth;

float terrain_depth_texel_elevation(ivec2 pos) {
    vec4 rgb = (texelFetch(u_terrain_dem, pos, 0) * 255.0) * u_terrain_unpack;
    return rgb.r + rgb.g + rgb.b - u_terrain_unpack.a;
}

float terrain_depth_elevation(vec2 pos) {
    vec2 coord = (u_terrain_matrix * vec4(pos, 0.0, 1.0)).xy * u_terrain_dim + 0.5;
    vec2 f = fract(coord);
    ivec2 c = ivec2(floor(coord));
    ivec2 hi = textureSize(u_terrain_dem, 0) - 1;
    float tl = terrain_depth_texel_elevation(clamp(c, ivec2(0), hi));
    float tr = terrain_depth_texel_elevation(clamp(c + ivec2(1, 0), ivec2(0), hi));
    float bl = terrain_depth_texel_elevation(clamp(c + ivec2(0, 1), ivec2(0), hi));
    float br = terrain_depth_texel_elevation(clamp(c + ivec2(1, 1), ivec2(0), hi));
    return mix(mix(tl, tr, f.x), mix(bl, br, f.x), f.y) * u_terrain_exaggeration;
}

void main() {
    float elevation = terrain_depth_elevation(a_pos3d.xy);
    float ele_delta = a_pos3d.z == 1.0 ? u_ele_delta : 0.0;
    gl_Position = u_matrix * vec4(a_pos3d.xy, elevation - ele_delta, 1.0);
    v_depth = gl_Position.z / gl_Position.w;
}
