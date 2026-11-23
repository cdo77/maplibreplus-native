layout (location = 0) in vec4 a_pos_offset;
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
    highp float u_ecef_altitude_m;
    highp float u_ecef_airborne_agl;
    highp vec4 u_ecef_exact;
    highp vec4 u_ecef_ground;
    highp vec4 u_ecef_model;
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

#pragma mapbox: define lowp float opacity

uniform sampler2D u_ecef_heightmap;

// Iconos 3D con camara real (ADR 0038): elevacion del terreno bajo el ancla, del mapa de alturas de
// RenderTerrain (terrarium RGBA8; bilineal a mano con texelFetch como la DEM del terreno -- filtrar
// lineal los bytes codificados daria alturas falsas).
float ecef_height_texel(ivec2 p, ivec2 hi) {
    vec3 rgb = texelFetch(u_ecef_heightmap, clamp(p, ivec2(0), hi), 0).rgb * 255.0;
    return rgb.r * 256.0 + rgb.g + rgb.b / 256.0 - 32768.0;
}

// El ancla cae dentro del mapa de alturas? Fuera, el borde repetido da alturas falsas: para un sprite en el aire
// (donde importa) se toma 0 en vez de una altura inventada.
bool ecef_anchor_inside(vec2 anchor) {
    vec2 uv = anchor * u_ecef_heightmap_scale + u_ecef_heightmap_offset;
    return uv.x >= 0.0 && uv.x <= 1.0 && uv.y >= 0.0 && uv.y <= 1.0;
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

// Posicion exacta por ancla (ADR 0058): para tiles gruesos el ancla (x, y tile-local) se convierte a mercator -> lat/lon ->
// ECEF (WGS84) en el shader en vez de usar la afin plana del tile, que por la curvatura hunde el ancla. u_matrix es en ese
// caso la matriz de la camara con origen en el centro de la Tierra. Con la bandera apagada es la afin de siempre.
vec4 ecef_project(vec2 p, float h) {
    if (u_ecef_exact.w < 0.5) {
        return u_matrix * vec4(p, h, 1);
    }
    float mx = u_ecef_exact.x + p.x * u_ecef_exact.z;
    float my = u_ecef_exact.y + p.y * u_ecef_exact.z;
    float lon = (mx - 0.5) * 6.28318530718;
    float lat = 2.0 * atan(exp(3.14159265359 * (1.0 - 2.0 * my))) - 1.57079632679;
    float sl = sin(lat);
    float cl = cos(lat);
    float n = 6378137.0 / sqrt(1.0 - 0.00669437999014 * sl * sl);
    vec3 ecef = vec3((n + h) * cl * cos(lon), (n + h) * cl * sin(lon), (n * (1.0 - 0.00669437999014) + h) * sl);
    return u_matrix * vec4(ecef, 1);
}

void main() {
    highp float u_opacity = u_is_text_prop ? u_text_opacity : u_icon_opacity;

    #pragma mapbox: initialize lowp float opacity

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
    float terrain_elevation = ecef_mode ? ((u_ecef_exact.w > 0.5 && !ecef_anchor_inside(a_pos)) ? 0.0 : ecef_anchor_elevation(a_pos)) : 0.0;
    // Altura propia (ADR 0058): con umbral > 0 (marcador en vuelo) o < 0 (su palito), el ancla sube de forma gradual
    // desde la altura sobre el terreno `gate` hasta `2 gate`; sin altura (umbral 0) todo queda como antes.
    float agl_gate = abs(u_ecef_airborne_agl);
    bool stick_mode = ecef_mode && u_ecef_airborne_agl < 0.0;
    float anchor_elevation = terrain_elevation;
    if (ecef_mode && agl_gate > 0.0) {
        float ground = ecef_anchor_inside(a_pos) ? terrain_elevation : 0.0;
        float agl = max(u_ecef_altitude_m - ground, 0.0);
        terrain_elevation = ground;
        anchor_elevation = ground + smoothstep(agl_gate, 2.0 * agl_gate, agl) * agl;
    }
    vec4 projectedPoint = ecef_project(a_pos, anchor_elevation);
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
        vec4 offsetProjectedPoint = ecef_project(a_pos + (u_ecef_exact.w > 0.5 ? vec2(512.0, 0.0) : vec2(1.0, 0.0)), anchor_elevation);

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
    // Apoyado en el suelo (ADR 0106): no se para sobre el ancla.
    bool ground_mode = ecef_mode && !stick_mode && u_ecef_ground.x > 0.5;
    // Camara real inclinada: el icono se para sobre el ancla (sube u_ecef_stand_px en pantalla), como ATAK.
    if (ecef_mode && !stick_mode && !ground_mode) {
        projected_pos.y -= u_ecef_stand_px * projected_pos.w;
    }
    vec2 corner_px = rotation_matrix * (a_offset / 32.0 * max(a_minFontScale, fontScale) + a_pxoffset / 16.0);
    vec4 depth_point = projectedPoint;
    bool stick_hidden = false;
    if (stick_mode) {
        // "Palito" (lollipop de ATAK): el cuadro del icono se estira entre el suelo y el ancla elevada. Las esquinas de
        // arriba van al extremo del aire, las de abajo al del suelo, y el ancho (el del propio icono, en px) se abre
        // perpendicular a la linea EN PANTALLA, asi se ve igual de fina mire la camara de frente o desde arriba.
        vec4 ground_clip = ecef_project(a_pos, terrain_elevation);
        vec4 air_px4 = projected_pos;
        if (u_ecef_ground.x > 0.5) {
            // Chevron apoyado y flotando (ADR 0106): el palito llega hasta su base, que flota (x - 1) px sobre el ancla (los mismos metros por
            // pixel, agrandados un cuarto, que usa el chevron), no hasta un chevron parado.
            float half_h_px_s = -1.0 / u_coord_matrix[1][1];
            float float_m_s = (u_ecef_ground.x - 1.0) * max(projectedPoint.w, 0.0) * 0.41421356 / half_h_px_s * 1.25;
            vec4 air_clip = ecef_project(a_pos, anchor_elevation + float_m_s);
            air_px4 = u_label_plane_matrix * vec4(air_clip.xy / air_clip.w, 0.0, 1.0);
        } else {
            air_px4.y -= u_ecef_stand_px * air_px4.w; // el palito llega hasta el centro del chevron, que se para sobre su ancla
        }
        vec4 ground_px4 = u_label_plane_matrix * vec4(ground_clip.xy / ground_clip.w, 0.0, 1.0);
        vec2 air_px = air_px4.xy / air_px4.w;
        vec2 ground_px = ground_px4.xy / ground_px4.w;
        vec2 dir = air_px - ground_px;
        float len = length(dir);
        vec2 dn = len > 0.001 ? dir / len : vec2(0.0, -1.0);
        bool top = a_offset.y < 0.0;
        float half_w = abs(a_offset.x) / 32.0 * max(a_minFontScale.x, fontScale);
        corner_px = vec2(-dn.y, dn.x) * (a_offset.x < 0.0 ? -half_w : half_w);
        projected_pos = vec4(top ? air_px : ground_px, 0.0, 1.0);
        depth_point = top ? projectedPoint : ground_clip;
        // En tierra no hay palito (ADR 0106: con el chevron apoyado en el suelo, el palito parado asomaba sobre el).
        stick_hidden = len < 1.0 || ground_clip.w <= 0.0 || anchor_elevation - terrain_elevation < 1.0;
    }
    gl_Position = u_coord_matrix * vec4(projected_pos.xy / projected_pos.w + corner_px, 0.0, 1.0);
    if (ground_mode) {
        // La esquina en el marco del icono, sin el giro que le aplico el layout (icon-rotate = el rumbo, en sentido horario con y hacia abajo).
        float rb = u_ecef_ground.y;
        vec2 o = a_offset / 32.0 * max(a_minFontScale, fontScale);
        // La pieza 3D y su sombra (cara >= 1) no giran en el layout (icon-rotate 0): el giro lo pone SOLO este rumbo, el mismo para todas sus
        // capas en el cuadro. Si dependieran del giro del layout, que se rehace por capa en otro hilo, al girar se desarmaban (ADR 0106).
        vec2 icon_px = u_ecef_model.x > 0.5 ? o : vec2(o.x * cos(rb) + o.y * sin(rb), -o.x * sin(rb) + o.y * cos(rb));
        // Metros por pixel a la distancia del ancla (camara real de 45 grados): el icono mide en el suelo lo que mide hoy en pantalla.
        float half_h_px = -1.0 / u_coord_matrix[1][1];
        // En 3D se agranda un cuarto: apoyado en el piso se ve menos que parado.
        float m_per_px = max(projectedPoint.w, 0.0) * 0.41421356 / half_h_px * 1.25;
        float fwd_m = -icon_px.y * m_per_px;
        float right_m = icon_px.x * m_per_px;
        // Adelante y a la derecha en coordenadas de tile (x al este, y al sur).
        vec2 fwd_dir = vec2(sin(rb), -cos(rb));
        vec2 right_dir = vec2(cos(rb), sin(rb));
        // Vista rasante: un metro adelante se ve mucho mas corto que uno de costado y la flecha se vuelve una raya. Estirarla la deformaba (la
        // punta se volvia una pua); en cambio se INCLINA: cuanto mas rasante la vista, mas se levanta la punta (de liftDeg a 60 grados), asi
        // siempre se la ve como con la camara a unos 45 grados, con la tapa y el canto a la vista. Igual en los cuatro vertices.
        vec4 lp0 = u_label_plane_matrix * vec4(projectedPoint.xy / projectedPoint.w, 0.0, 1.0);
        vec4 pf = ecef_project(a_pos + fwd_dir * u_ecef_ground.z, anchor_elevation);
        vec4 pr = ecef_project(a_pos + right_dir * u_ecef_ground.z, anchor_elevation);
        vec4 lpf = u_label_plane_matrix * vec4(pf.xy / pf.w, 0.0, 1.0);
        vec4 lpr = u_label_plane_matrix * vec4(pr.xy / pr.w, 0.0, 1.0);
        float len_f = length(lpf.xy / lpf.w - lp0.xy / lp0.w);
        float len_r = length(lpr.xy / lpr.w - lp0.xy / lp0.w);
        float squash = len_r > 0.0001 ? len_f / len_r : 1.0;
        float view_elev = asin(clamp(squash, 0.0, 1.0)); // angulo aproximado entre la mirada y el suelo, visto a lo largo de la flecha
        float lift = clamp(0.7679 - view_elev, u_ecef_ground.w, 1.0472); // asin(0,695) ~ 44 grados; tope 60
        vec2 tile_off = (fwd_dir * fwd_m * cos(lift) + right_dir * right_m) * u_ecef_ground.z;
        // Flota (px de pantalla pasados a metros) y la punta se levanta girando sobre la cola.
        float float_m = (u_ecef_ground.x - 1.0) * m_per_px;
        float h_off = float_m + (fwd_m + abs(fwd_m)) * sin(lift);
        vec4 corner_clip = ecef_project(a_pos + tile_off, anchor_elevation + h_off);
        vec4 corner_lp = u_label_plane_matrix * vec4(corner_clip.xy / corner_clip.w, 0.0, 1.0);
        gl_Position = u_coord_matrix * vec4(corner_lp.xy / corner_lp.w, 0.0, 1.0);
        depth_point = corner_clip;

        int face = int(u_ecef_model.x + 0.5);
        if (face >= 1 && face <= 6) {
            // Pieza 3D (ADR 0106): la silueta del chevron con un lomo recto de la punta a la muesca, arriba y abajo (rombo visto de atras,
            // cuna de costado). En el marco del icono, en fracciones del medio ancho (W) y medio alto (H): punta (0, 0,9H), alas (+-0,84W,
            // -0,88H), muesca arriba/abajo (0, -0,5H, +-t W). Esta capa dibuja UNA cara: sus tres vertices en las esquinas del quad (la cuarta
            // repite la tercera).
            float Wh = abs(icon_px.x);
            float Hh = abs(icon_px.y);
            float t = u_ecef_model.y;
            vec3 T = vec3(0.0, 0.9, 0.0);
            vec3 L = vec3(-0.84, -0.88, 0.0);
            vec3 Rv = vec3(0.84, -0.88, 0.0);
            vec3 NT = vec3(0.0, -0.5, t);
            vec3 NB = vec3(0.0, -0.5, -t);
            vec3 v0 = T; vec3 v1 = L; vec3 v2 = NT;                 // 1: arriba izquierda
            if (face == 2) { v0 = T; v1 = NT; v2 = Rv; }            // 2: arriba derecha
            else if (face == 3) { v0 = T; v1 = NB; v2 = L; }        // 3: abajo izquierda
            else if (face == 4) { v0 = T; v1 = Rv; v2 = NB; }       // 4: abajo derecha
            else if (face == 5) { v0 = L; v1 = NB; v2 = NT; }       // 5: pared de la muesca, ala izquierda
            else if (face == 6) { v0 = Rv; v1 = NT; v2 = NB; }      // 6: pared de la muesca, ala derecha
            // Que vertice le toca a esta esquina del quad: arriba-izq v0, arriba-der v1, abajo v2.
            vec3 mv = icon_px.y < 0.0 ? (icon_px.x < 0.0 ? v0 : v1) : v2;
            // Un vertice del modelo al espacio de recorte: derecha y adelante en metros sobre el suelo, arriba en metros sobre lo que flota.
            float fl_m = (u_ecef_ground.x - 1.0) * m_per_px;
            vec4 c0 = ecef_project(a_pos + (right_dir * v0.x * Wh + fwd_dir * v0.y * Hh) * m_per_px * u_ecef_ground.z,
                                   anchor_elevation + fl_m + v0.z * Wh * m_per_px);
            vec4 c1 = ecef_project(a_pos + (right_dir * v1.x * Wh + fwd_dir * v1.y * Hh) * m_per_px * u_ecef_ground.z,
                                   anchor_elevation + fl_m + v1.z * Wh * m_per_px);
            vec4 c2 = ecef_project(a_pos + (right_dir * v2.x * Wh + fwd_dir * v2.y * Hh) * m_per_px * u_ecef_ground.z,
                                   anchor_elevation + fl_m + v2.z * Wh * m_per_px);
            vec4 cm = ecef_project(a_pos + (right_dir * mv.x * Wh + fwd_dir * mv.y * Hh) * m_per_px * u_ecef_ground.z,
                                   anchor_elevation + fl_m + mv.z * Wh * m_per_px);
            vec4 l0 = u_label_plane_matrix * vec4(c0.xy / c0.w, 0.0, 1.0);
            vec4 l1 = u_label_plane_matrix * vec4(c1.xy / c1.w, 0.0, 1.0);
            vec4 l2 = u_label_plane_matrix * vec4(c2.xy / c2.w, 0.0, 1.0);
            vec2 p0 = l0.xy / l0.w; vec2 p1 = l1.xy / l1.w; vec2 p2 = l2.xy / l2.w;
            float winding = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
            // Cara de espaldas: se descarta (la pieza se ve solida sin ordenar las caras). El sentido lo fija la app (cullSign).
            bool back = u_ecef_model.z != 0.0 && winding * u_ecef_model.z > 0.0;
            vec4 lm = u_label_plane_matrix * vec4(cm.xy / cm.w, 0.0, 1.0);
            // Cada cara se agranda 1,5 px hacia afuera de su centro: el borde de la imagen se mezcla con lo transparente del atlas y entre
            // caras vecinas quedaba una rendija por la que se veia el mapa. Solapadas, la pieza queda cerrada.
            vec2 pm = lm.xy / lm.w;
            vec2 cen = (p0 + p1 + p2) / 3.0;
            vec2 away = pm - cen;
            float away_len = length(away);
            pm += away_len > 0.001 ? away / away_len * 1.5 : vec2(0.0);
            gl_Position = u_coord_matrix * vec4(pm, 0.0, 1.0);
            depth_point = back ? vec4(0.0, 0.0, 0.0, -1.0) : cm;
        }
    }
    if (ecef_mode) {
        // Como los sprites de ATAK (BatchGeometryPoints.vert), el icono se adelanta hacia la camara su propio
        // radio en metros -- radio en px x metros/px a esa distancia --, asi el relieve no lo tapa aunque el
        // ancla quede unos metros bajo la malla. Escalar el punto por s alrededor del ojo deja x, y, w
        // multiplicados por s y z = s (z - B) + B; s = 1 - radio/distancia = 1 - radio_px tan(fov/2) / (alto/2)
        // (fov 45 de la camara real, alto/2 = -1/u_coord_matrix[1][1]). Detras de la camara (w <= 0): fuera.
        // Apoyado en el suelo, adelantarlo todo su radio lo despegaria: alcanza con un cuarto para no pelearse con la malla.
        float radius_px = stick_mode ? 0.0 : length(a_offset / 32.0 * max(a_minFontScale, fontScale)) * (ground_mode ? (u_ecef_model.x > 0.5 ? 0.1 : 0.25) : 1.0);
        float half_height_px = -1.0 / u_coord_matrix[1][1];
        float s = max(1.0 - radius_px * 0.41421356 / half_height_px, 0.05);
        gl_Position.z = (depth_point.w > 0.0 && !stick_hidden)
            ? (s * (depth_point.z - u_ecef_depth_b) + u_ecef_depth_b) / (s * depth_point.w)
            : 2.0;
    }

    if ((u_ecef_model.x > 0.5 && !ecef_mode) || (u_ecef_model.x < -0.5 && ecef_mode)) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0); // fuera del volumen: no se dibuja
    }
    v_tex = a_tex / u_texsize;
    vec2 fade_opacity = unpack_opacity(a_fade_opacity);
    float fade_change = fade_opacity[1] > 0.5 ? u_symbol_fade_change : -u_symbol_fade_change;
    v_fade_opacity = max(0.0, min(1.0, fade_opacity[0] + fade_change));
}
