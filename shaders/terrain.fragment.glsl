layout (std140) uniform TerrainDrawableUBO {
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
