layout (location = 0) in vec3 a_pos3d;
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
    lowp float drawable_pad1;
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
