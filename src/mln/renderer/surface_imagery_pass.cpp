#include <mln/renderer/surface_imagery_pass.hpp>

#if MLN_RENDER_BACKEND_OPENGL

#include <mln/gfx/drawable.hpp>
#include <mln/gl/context.hpp>
#include <mln/gl/defines.hpp>
#include <mln/gl/texture2d.hpp>
#include <mln/renderer/layer_group.hpp>
#include <mln/renderer/paint_parameters.hpp>
#include <mln/renderer/render_orchestrator.hpp>
#include <mln/shaders/shader_defines.hpp>
#include <mln/util/logging.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace mln {
namespace surface_pass {

namespace {

using namespace platform;

// Shader minimo: un cuadro con una textura, sin uniformes de capa. La textura de las teselas es premultiplicada.
constexpr const char* kVertexShader = R"(#version 300 es
in vec2 a_pos;
in vec2 a_uv;
out vec2 v_uv;
void main() {
    v_uv = a_uv;
    gl_Position = vec4(a_pos, 0.0, 1.0);
}
)";

constexpr const char* kFragmentShader = R"(#version 300 es
precision mediump float;
uniform sampler2D u_tex;
in vec2 v_uv;
out vec4 fragColor;
void main() {
    fragColor = texture(u_tex, v_uv);
}
)";

struct Gpu {
    GLuint program = 0;
    GLuint vao = 0;
    GLuint vbo = 0;
    GLint texLocation = -1;
    bool failureLogged = false;
};

Gpu& gpu() {
    static Gpu g;
    return g;
}

GLuint compile(GLenum type, const char* source, std::string& error) {
    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLchar log[512] = {};
        glGetShaderInfoLog(shader, sizeof(log) - 1, nullptr, log);
        error = std::string("no compila el shader de la pasada de imagen: ") + log;
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

// Crea (o recrea si se perdio el contexto de GL) el programa, el VAO y el VBO de la pasada. Devuelve false y registra el motivo si falla.
bool ensureGpu(Gpu& g) {
    if (g.program && glIsProgram(g.program) && g.vao && glIsVertexArray(g.vao) && g.vbo && glIsBuffer(g.vbo)) return true;

    std::string error;
    const GLuint vs = compile(GL_VERTEX_SHADER, kVertexShader, error);
    const GLuint fs = vs ? compile(GL_FRAGMENT_SHADER, kFragmentShader, error) : 0;
    if (!vs || !fs) {
        if (vs) glDeleteShader(vs);
        if (!g.failureLogged) {
            g.failureLogged = true;
            Log::Error(Event::General, error);
        }
        return false;
    }
    g.program = glCreateProgram();
    glAttachShader(g.program, vs);
    glAttachShader(g.program, fs);
    glBindAttribLocation(g.program, 0, "a_pos");
    glBindAttribLocation(g.program, 1, "a_uv");
    glLinkProgram(g.program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint linked = 0;
    glGetProgramiv(g.program, GL_LINK_STATUS, &linked);
    if (!linked) {
        GLchar log[512] = {};
        glGetProgramInfoLog(g.program, sizeof(log) - 1, nullptr, log);
        if (!g.failureLogged) {
            g.failureLogged = true;
            Log::Error(Event::General, std::string("no enlaza el programa de la pasada de imagen: ") + log);
        }
        glDeleteProgram(g.program);
        g.program = 0;
        return false;
    }
    g.texLocation = glGetUniformLocation(g.program, "u_tex");

    glGenVertexArrays(1, &g.vao);
    glGenBuffers(1, &g.vbo);
    glBindVertexArray(g.vao);
    glBindBuffer(GL_ARRAY_BUFFER, g.vbo);
    glBufferData(GL_ARRAY_BUFFER, 16 * sizeof(float), nullptr, GL_STREAM_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), reinterpret_cast<const void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), reinterpret_cast<const void*>(2 * sizeof(float)));
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return true;
}

struct Quad {
    uint8_t z = 0;
    GLuint texture = 0;
    float vertices[16] = {};
};

} // namespace

void drawImagery(RenderOrchestrator& orchestrator, PaintParameters& parameters, const util::SurfaceCull& cell) {
    std::vector<Quad> quads;
    {
        const util::SurfaceCullScope scope(&cell);
        orchestrator.visitLayerGroups([&](LayerGroupBase& group) {
            if (!orchestrator.layerGroupIsType(group, "raster")) return;
            static_cast<TileLayerGroup&>(group).visitDrawables([&](gfx::Drawable& drawable) {
                const auto& id = drawable.getTileID();
                if (!drawable.getEnabled() || !id || id->wrap != cell.wrap) return;
                const auto& texture = drawable.getTexture(shaders::idRasterImage0Texture);
                if (!texture) return;

                // La tesela en coordenadas de la celda: u hacia el este, v hacia el sur, [0,1] = la celda. Una tesela mas gruesa que la celda la cubre con mas de 1.
                const double s = std::ldexp(1.0, static_cast<int>(id->canonical.z) - static_cast<int>(cell.z));
                const double u0 = static_cast<double>(id->canonical.x) / s - static_cast<double>(cell.x);
                const double v0 = static_cast<double>(id->canonical.y) / s - static_cast<double>(cell.y);
                const double w = 1.0 / s;
                // Recortado de la celda: el norte es +1 en NDC y la fila 0 de la imagen de la tesela es el norte (t = 0).
                const auto x0 = static_cast<float>(2.0 * u0 - 1.0);
                const auto x1 = static_cast<float>(2.0 * (u0 + w) - 1.0);
                const auto yn = static_cast<float>(1.0 - 2.0 * v0);
                const auto ys = static_cast<float>(1.0 - 2.0 * (v0 + w));

                Quad q;
                q.z = id->canonical.z;
                q.texture = static_cast<const gl::Texture2D&>(*texture).getTextureID();
                const float v[16] = {x0, yn, 0.0f, 0.0f, x1, yn, 1.0f, 0.0f, x0, ys, 0.0f, 1.0f, x1, ys, 1.0f, 1.0f};
                std::copy(std::begin(v), std::end(v), q.vertices);
                quads.push_back(q);
            });
        });
    }
    if (quads.empty()) return;
    // De la tesela mas gruesa a la mas fina: la fina pisa a la gruesa donde se solapan.
    std::stable_sort(quads.begin(), quads.end(), [](const Quad& a, const Quad& b) { return a.z < b.z; });

    Gpu& g = gpu();
    if (!ensureGpu(g)) return;

    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_CULL_FACE);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_ONE, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(g.program);
    glBindVertexArray(g.vao);
    glBindBuffer(GL_ARRAY_BUFFER, g.vbo);
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(g.texLocation, 0);
    // Un solo envio de vertices para todos los cuadros de la celda; cada cuadro se dibuja con su desplazamiento.
    std::vector<float> vertices;
    vertices.reserve(quads.size() * 16);
    for (const Quad& q : quads) vertices.insert(vertices.end(), std::begin(q.vertices), std::end(q.vertices));
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(float)), vertices.data(), GL_STREAM_DRAW);
    GLint first = 0;
    for (const Quad& q : quads) {
        glBindTexture(GL_TEXTURE_2D, q.texture);
        // Las teselas de imagen se muestrean con filtro lineal y borde repetido; se fijan aca porque el motor aplica el muestreo de una textura recien al usarla en su propio dibujo.
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glDrawArrays(GL_TRIANGLE_STRIP, first, 4);
        first += 4;
    }
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    // Todo el estado de GL que toco esta pasada se declara sucio: lo que sigue (los grupos de vectores, el pase principal) lo vuelve a aplicar entero.
    static_cast<gl::Context&>(parameters.context).setDirtyState();
}

} // namespace surface_pass
} // namespace mln

#else

namespace mln {
namespace surface_pass {
void drawImagery(RenderOrchestrator&, PaintParameters&, const util::SurfaceCull&) {}
} // namespace surface_pass
} // namespace mln

#endif
