#include "SkeletonRenderer.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace pmgviewer {

namespace {

// --- Tunable rendering constants (named per readability guidelines) ---------
constexpr int kShadowMapResolution = 2048;
constexpr int kCylinderRadialSegments = 16;
constexpr int kSphereStacks = 14;
constexpr int kSphereSectors = 20;
constexpr float kFloorHalfExtent = 10000.0f;
constexpr float kJointRadius = 1.5f;

constexpr float kLightDistance = 120.0f;    // light placed this far along its direction
constexpr float kShadowOrthoHalfExtent = 120.0f;
constexpr float kShadowNearPlane = 1.0f;
constexpr float kShadowFarPlane = 400.0f;

constexpr float kPi = 3.14159265358979323846f;

const glm::vec3 kFloorColor(0.52f, 0.55f, 0.60f);
const glm::vec3 kBoneColor(0.86f, 0.44f, 0.24f);
const glm::vec3 kJointColor(0.96f, 0.78f, 0.26f);
const glm::vec3 kMarkerColor(0.30f, 0.85f, 0.40f);
constexpr float kMarkerPointRadius = 2.0f;
constexpr float kMarkerLineRadius = 0.5f;

const char* const kSceneVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;
uniform mat4 uLightSpace;
out vec3 vWorldPos;
out vec3 vNormal;
out vec4 vLightSpacePos;
void main() {
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorldPos = world.xyz;
    vNormal = mat3(transpose(inverse(uModel))) * aNormal;
    vLightSpacePos = uLightSpace * world;
    gl_Position = uProj * uView * world;
}
)GLSL";

const char* const kSceneFragmentShader = R"GLSL(
#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;
in vec4 vLightSpacePos;
uniform vec3 uLightToDir;
uniform vec3 uColor;
uniform int uIsFloor;
uniform float uAlpha;
uniform int uFloorHasOrigin;
uniform vec2 uFloorOriginXz;
uniform sampler2D uShadowMap;
out vec4 FragColor;

float ShadowFactor(vec3 normal, vec3 lightDir) {
    vec3 proj = vLightSpacePos.xyz / vLightSpacePos.w;
    proj = proj * 0.5 + 0.5;
    if (proj.z > 1.0) {
        return 0.0;
    }
    float bias = max(0.0030 * (1.0 - dot(normal, lightDir)), 0.0010);
    float shadow = 0.0;
    vec2 texel = 1.0 / vec2(textureSize(uShadowMap, 0));
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            float closest = texture(uShadowMap, proj.xy + vec2(x, y) * texel).r;
            shadow += (proj.z - bias > closest) ? 1.0 : 0.0;
        }
    }
    return shadow / 9.0;
}

void main() {
    vec3 normal = normalize(vNormal);
    vec3 lightDir = normalize(uLightToDir);
    vec3 baseColor = uColor;
    if (uIsFloor == 1) {
        // Checkerboard tiles give an unambiguous scale/depth cue the flat-grey
        // floor lacked. Fade to a flat mid-grey in the distance so far tiles
        // (sub-pixel) do not shimmer, and add crisp seam lines for readability
        // up close.
        float cellSize = 50.0;
        vec2 cellUv = vWorldPos.xz / cellSize;
        vec2 cellId = floor(cellUv);
        float checker = mod(cellId.x + cellId.y, 2.0);
        vec3 darkTile = uColor * 0.60;
        vec3 lightTile = uColor;
        vec3 flatTile = mix(darkTile, lightTile, 0.5);
        vec2 fw = fwidth(cellUv);
        float blur = clamp(max(fw.x, fw.y), 0.0, 1.0);
        baseColor = mix(mix(darkTile, lightTile, checker), flatTile, blur);
        if (uFloorHasOrigin == 1 &&
            all(equal(cellId, floor(uFloorOriginXz / cellSize)))) {
            // The motion-origin cell: gold, keeping a faint checker variation so
            // it still reads as one tile of the same grid.
            vec3 goldTile = vec3(0.85, 0.62, 0.13);
            baseColor = mix(goldTile * 0.78, goldTile, checker);
        }
        vec2 seam = abs(fract(cellUv - 0.5) - 0.5) / max(fw, vec2(1.0e-5));
        float line = 1.0 - min(min(seam.x, seam.y), 1.0);
        baseColor = mix(baseColor, baseColor * 0.70, line * 0.6);
    }
    float diffuse = max(dot(normal, lightDir), 0.0);
    float shadow = ShadowFactor(normal, lightDir);
    vec3 ambient = 0.32 * baseColor;
    vec3 lit = ambient + (1.0 - shadow) * 0.85 * diffuse * baseColor;
    lit = pow(lit, vec3(1.0 / 2.2));
    FragColor = vec4(lit, uAlpha);
}
)GLSL";

const char* const kDepthVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uModel;
uniform mat4 uLightSpace;
void main() {
    gl_Position = uLightSpace * uModel * vec4(aPos, 1.0);
}
)GLSL";

const char* const kDepthFragmentShader = R"GLSL(
#version 330 core
void main() {}
)GLSL";

GLuint CompileShader(GLenum shader_type, const char* source) {
    const GLuint shader = glCreateShader(shader_type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint compiled = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (compiled != GL_TRUE) {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        throw std::runtime_error(std::string("shader compile failed: ") + log);
    }
    return shader;
}

GLuint LinkProgram(const char* vertex_source, const char* fragment_source) {
    const GLuint vertex_shader = CompileShader(GL_VERTEX_SHADER, vertex_source);
    const GLuint fragment_shader = CompileShader(GL_FRAGMENT_SHADER, fragment_source);

    const GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    GLint linked = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (linked != GL_TRUE) {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        throw std::runtime_error(std::string("program link failed: ") + log);
    }

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
    return program;
}

// Model matrix that maps the unit +Y cylinder onto the segment a->b.
glm::mat4 BoneModelMatrix(const glm::vec3& a, const glm::vec3& b, float radius) {
    const glm::vec3 delta = b - a;
    const float length = glm::length(delta);
    if (length < 1.0e-6f) {
        return glm::scale(glm::translate(glm::mat4(1.0f), a), glm::vec3(radius));
    }

    const glm::vec3 direction = delta / length;
    const glm::vec3 up(0.0f, 1.0f, 0.0f);
    const float cosine = glm::dot(up, direction);

    glm::mat4 rotation(1.0f);
    if (cosine < -0.99999f) {
        rotation = glm::rotate(glm::mat4(1.0f), kPi, glm::vec3(1.0f, 0.0f, 0.0f));
    } else if (cosine < 0.99999f) {
        const glm::vec3 axis = glm::normalize(glm::cross(up, direction));
        const float angle = std::acos(cosine);
        rotation = glm::rotate(glm::mat4(1.0f), angle, axis);
    }

    glm::mat4 model = glm::translate(glm::mat4(1.0f), a);
    model = model * rotation;
    model = glm::scale(model, glm::vec3(radius, length, radius));
    return model;
}

glm::mat4 LightSpaceMatrix(const glm::vec3& focus_point, const glm::vec3& light_to_direction) {
    const glm::vec3 direction = glm::normalize(light_to_direction);
    const glm::vec3 light_position = focus_point + direction * kLightDistance;
    const glm::vec3 up = std::abs(direction.y) > 0.99f ? glm::vec3(0.0f, 0.0f, 1.0f)
                                                       : glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::mat4 light_view = glm::lookAt(light_position, focus_point, up);
    const glm::mat4 light_projection = glm::ortho(
        -kShadowOrthoHalfExtent, kShadowOrthoHalfExtent,
        -kShadowOrthoHalfExtent, kShadowOrthoHalfExtent,
        kShadowNearPlane, kShadowFarPlane);
    return light_projection * light_view;
}

}  // namespace

void SkeletonRenderer::Initialize() {
    scene_program_ = LinkProgram(kSceneVertexShader, kSceneFragmentShader);
    depth_program_ = LinkProgram(kDepthVertexShader, kDepthFragmentShader);

    cylinder_mesh_ = CreateCylinderMesh(kCylinderRadialSegments);
    sphere_mesh_ = CreateSphereMesh(kSphereStacks, kSphereSectors);
    floor_mesh_ = CreateFloorMesh(kFloorHalfExtent);

    glGenFramebuffers(1, &shadow_framebuffer_);
    glGenTextures(1, &shadow_depth_texture_);
    glBindTexture(GL_TEXTURE_2D, shadow_depth_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24,
                 kShadowMapResolution, kShadowMapResolution, 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    const float border_color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border_color);

    glBindFramebuffer(GL_FRAMEBUFFER, shadow_framebuffer_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                           shadow_depth_texture_, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        throw std::runtime_error("SkeletonRenderer: shadow framebuffer incomplete");
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glEnable(GL_DEPTH_TEST);
}

void SkeletonRenderer::Shutdown() {
    cylinder_mesh_.Destroy();
    sphere_mesh_.Destroy();
    floor_mesh_.Destroy();
    if (shadow_depth_texture_   != 0)   glDeleteTextures(1, &shadow_depth_texture_);
    if (shadow_framebuffer_     != 0)   glDeleteFramebuffers(1, &shadow_framebuffer_);
    if (scene_program_          != 0)   glDeleteProgram(scene_program_);
    if (depth_program_          != 0)   glDeleteProgram(depth_program_);
}

void SkeletonRenderer::DrawSceneGeometry(const RenderScene& scene, GLuint program,
                                         bool is_depth_pass) {
    const GLint model_location = glGetUniformLocation(program, "uModel");
    const GLint color_location = glGetUniformLocation(program, "uColor");
    const GLint is_floor_location = glGetUniformLocation(program, "uIsFloor");
    const GLint alpha_location = glGetUniformLocation(program, "uAlpha");

    auto set_model = [&](const glm::mat4& model) {
        glUniformMatrix4fv(model_location, 1, GL_FALSE, glm::value_ptr(model));
    };
    auto set_object_color = [&](const glm::vec3& color, int is_floor) {
        if (!is_depth_pass) {
            glUniform3fv(color_location, 1, glm::value_ptr(color));
            glUniform1i(is_floor_location, is_floor);
        }
    };
    // Opaque geometry below; diagnostic lines override per-line. (uAlpha is a no-op
    // on the depth program, whose location resolves to -1.)
    if (!is_depth_pass) {
        glUniform1f(alpha_location, 1.0f);
    }

    set_object_color(kFloorColor, 1);
    if (!is_depth_pass) {
        glUniform1i(glGetUniformLocation(program, "uFloorHasOrigin"),
                    scene.has_floor_origin ? 1 : 0);
        glUniform2f(glGetUniformLocation(program, "uFloorOriginXz"),
                    scene.floor_origin.x, scene.floor_origin.z);
    }
    set_model(glm::mat4(1.0f));
    floor_mesh_.Draw();

    set_object_color(kBoneColor, 0);
    for (const BoneSegment& bone : scene.bones) {
        set_model(BoneModelMatrix(bone.parent_position, bone.child_position, bone.radius));
        cylinder_mesh_.Draw();
        // Rounded caps at both ends turn the cylinder into a capsule, so a limb
        // chain reads as one smooth body instead of a stack of cut tubes.
        set_model(glm::scale(glm::translate(glm::mat4(1.0f), bone.parent_position),
                             glm::vec3(bone.radius)));
        sphere_mesh_.Draw();
        set_model(glm::scale(glm::translate(glm::mat4(1.0f), bone.child_position),
                             glm::vec3(bone.radius)));
        sphere_mesh_.Draw();
    }

    set_object_color(kJointColor, 0);
    for (const glm::vec3& joint : scene.joints) {
        set_model(glm::scale(glm::translate(glm::mat4(1.0f), joint), glm::vec3(kJointRadius)));
        sphere_mesh_.Draw();
    }

    set_object_color(kMarkerColor, 0);
    for (const glm::vec3& point : scene.marker_points) {
        set_model(glm::scale(glm::translate(glm::mat4(1.0f), point),
                             glm::vec3(kMarkerPointRadius)));
        sphere_mesh_.Draw();
    }
    for (const BoneSegment& line : scene.marker_lines) {
        set_model(BoneModelMatrix(line.parent_position, line.child_position,
                                  kMarkerLineRadius));
        cylinder_mesh_.Draw();
    }

    if (!is_depth_pass) {
        for (const DiagnosticPoint& point : scene.diagnostic_points) {
            set_object_color(point.color, 0);
            set_model(glm::scale(
                glm::translate(glm::mat4(1.0f), point.position),
                glm::vec3(point.radius)));
            sphere_mesh_.Draw();
        }
        // Translucent trails blend over the opaque scene; depth-write stays on so
        // they still occlude each other plausibly (order-independent enough for
        // faint ghost clouds).
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        for (const DiagnosticLine& line : scene.diagnostic_lines) {
            set_object_color(line.color, 0);
            glUniform1f(alpha_location, line.alpha);
            set_model(BoneModelMatrix(line.start, line.end, line.radius));
            cylinder_mesh_.Draw();
        }
        glDisable(GL_BLEND);
    }
}

void SkeletonRenderer::RenderShadowPass(const RenderScene& scene,
                                        const glm::mat4& light_space_matrix) {
    glViewport(0, 0, kShadowMapResolution, kShadowMapResolution);
    glBindFramebuffer(GL_FRAMEBUFFER, shadow_framebuffer_);
    glClear(GL_DEPTH_BUFFER_BIT);

    glUseProgram(depth_program_);
    glUniformMatrix4fv(glGetUniformLocation(depth_program_, "uLightSpace"), 1, GL_FALSE,
                       glm::value_ptr(light_space_matrix));

    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);  // cull front faces to reduce shadow acne / peter-panning
    DrawSceneGeometry(scene, depth_program_, /*is_depth_pass=*/true);
    glCullFace(GL_BACK);
    glDisable(GL_CULL_FACE);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void SkeletonRenderer::RenderScenePass(const RenderScene& scene,
                                       const glm::mat4& view_matrix,
                                       const glm::mat4& projection_matrix,
                                       const glm::vec3& camera_position,
                                       const glm::mat4& light_space_matrix) {
    (void)camera_position;
    glUseProgram(scene_program_);
    glUniformMatrix4fv(glGetUniformLocation(scene_program_, "uView"), 1, GL_FALSE,
                       glm::value_ptr(view_matrix));
    glUniformMatrix4fv(glGetUniformLocation(scene_program_, "uProj"), 1, GL_FALSE,
                       glm::value_ptr(projection_matrix));
    glUniformMatrix4fv(glGetUniformLocation(scene_program_, "uLightSpace"), 1, GL_FALSE,
                       glm::value_ptr(light_space_matrix));
    glUniform3fv(glGetUniformLocation(scene_program_, "uLightToDir"), 1,
                 glm::value_ptr(scene.light_to_direction));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadow_depth_texture_);
    glUniform1i(glGetUniformLocation(scene_program_, "uShadowMap"), 0);

    DrawSceneGeometry(scene, scene_program_, /*is_depth_pass=*/false);
}

void SkeletonRenderer::Render(const RenderScene& scene,
                              const glm::mat4& view_matrix,
                              const glm::mat4& projection_matrix,
                              const glm::vec3& camera_position,
                              int framebuffer_width,
                              int framebuffer_height) {
    const glm::mat4 light_space_matrix =
        LightSpaceMatrix(scene.focus_point, scene.light_to_direction);

    RenderShadowPass(scene, light_space_matrix);

    glViewport(0, 0, framebuffer_width, framebuffer_height);
    glClearColor(0.12f, 0.13f, 0.16f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    RenderScenePass(scene, view_matrix, projection_matrix, camera_position, light_space_matrix);
}

}  // namespace pmgviewer
