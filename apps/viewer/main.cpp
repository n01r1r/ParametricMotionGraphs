// Real-time BVH skeleton viewer: lit floor with shadows, orbit camera, and an
// ImGui control panel (playback, clip picker, camera, parametric blend).
//
// Inputs:  BVH corpus directory (compiled in as PMG_BVH_DIRECTORY).
// Outputs: an interactive window; no files are written.
// Failure modes: GLFW/GLEW init failure or shader compile errors abort with a
// message on stderr and a non-zero exit code.

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include <cstdio>
#include <exception>

#include "Camera.h"
#include "SkeletonRenderer.h"
#include "ViewerApp.h"

namespace {

constexpr int kInitialWindowWidth = 1280;
constexpr int kInitialWindowHeight = 800;
constexpr float kOrbitRadiansPerPixel = 0.01f;

struct InputState {
    pmgviewer::OrbitCamera* camera = nullptr;
    bool dragging = false;
    double last_cursor_x = 0.0;
    double last_cursor_y = 0.0;
};

void CursorPositionCallback(GLFWwindow* window, double x, double y) {
    ImGui_ImplGlfw_CursorPosCallback(window, x, y);
    auto* input = static_cast<InputState*>(glfwGetWindowUserPointer(window));
    if (input == nullptr) {
        return;
    }
    const double delta_x = x - input->last_cursor_x;
    const double delta_y = y - input->last_cursor_y;
    input->last_cursor_x = x;
    input->last_cursor_y = y;
    if (input->dragging && !ImGui::GetIO().WantCaptureMouse) {
        input->camera->Orbit(static_cast<float>(delta_x) * kOrbitRadiansPerPixel,
                             -static_cast<float>(delta_y) * kOrbitRadiansPerPixel);
    }
}

void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
    auto* input = static_cast<InputState*>(glfwGetWindowUserPointer(window));
    if (input == nullptr || button != GLFW_MOUSE_BUTTON_LEFT) {
        return;
    }
    input->dragging = (action == GLFW_PRESS) && !ImGui::GetIO().WantCaptureMouse;
}

void ScrollCallback(GLFWwindow* window, double x_offset, double y_offset) {
    ImGui_ImplGlfw_ScrollCallback(window, x_offset, y_offset);
    auto* input = static_cast<InputState*>(glfwGetWindowUserPointer(window));
    if (input == nullptr || ImGui::GetIO().WantCaptureMouse) {
        return;
    }
    input->camera->Zoom(static_cast<float>(y_offset));
}

}  // namespace

int main() {
    if (glfwInit() != GLFW_TRUE) {
        std::fprintf(stderr, "error: failed to initialize GLFW\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    GLFWwindow* window = glfwCreateWindow(kInitialWindowWidth, kInitialWindowHeight,
                                          "Parametric Motion Graphs - BVH Viewer",
                                          nullptr, nullptr);
    if (window == nullptr) {
        std::fprintf(stderr, "error: failed to create GLFW window\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glewExperimental = GL_TRUE;
    const GLenum glew_status = glewInit();
    if (glew_status != GLEW_OK) {
        std::fprintf(stderr, "error: failed to initialize GLEW: %s\n",
                     glewGetErrorString(glew_status));
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    glEnable(GL_MULTISAMPLE);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& imgui_io = ImGui::GetIO();
    imgui_io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    // Multi-viewport: panels (e.g. the Distance Grid heatmap) can be dragged out
    // of the main window into their own OS-level windows / second monitor.
    imgui_io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    ImGui::StyleColorsDark();
    // When viewports are enabled, platform windows look best with opaque
    // backgrounds and square corners.
    if (imgui_io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        ImGuiStyle& style = ImGui::GetStyle();
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }
    ImGui_ImplGlfw_InitForOpenGL(window, false);  // we install + chain callbacks ourselves
    ImGui_ImplOpenGL3_Init("#version 330");

    try {
        pmgviewer::ViewerApp app;
        app.Initialize();

        pmgviewer::SkeletonRenderer renderer;
        renderer.Initialize();

        InputState input;
        input.camera = &app.Camera();
        glfwSetWindowUserPointer(window, &input);
        glfwSetCursorPosCallback(window, CursorPositionCallback);
        glfwSetMouseButtonCallback(window, MouseButtonCallback);
        glfwSetScrollCallback(window, ScrollCallback);
        glfwSetKeyCallback(window, ImGui_ImplGlfw_KeyCallback);
        glfwSetCharCallback(window, ImGui_ImplGlfw_CharCallback);

        double previous_time = glfwGetTime();
        while (glfwWindowShouldClose(window) == GLFW_FALSE) {
            glfwPollEvents();

            const double now = glfwGetTime();
            const float delta_seconds = static_cast<float>(now - previous_time);
            previous_time = now;

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            app.Update(delta_seconds);
            app.BuildUi();

            int framebuffer_width = 0;
            int framebuffer_height = 0;
            glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
            if (framebuffer_height > 0) {
                app.Camera().SetAspectRatio(static_cast<float>(framebuffer_width) /
                                            static_cast<float>(framebuffer_height));
            }

            renderer.Render(app.Scene(),
                            app.Camera().ViewMatrix(),
                            app.Camera().ProjectionMatrix(),
                            app.Camera().Position(),
                            framebuffer_width,
                            framebuffer_height);

            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            // Render and present any torn-off platform windows, then restore the
            // main GL context for the next frame.
            if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
                GLFWwindow* backup_context = glfwGetCurrentContext();
                ImGui::UpdatePlatformWindows();
                ImGui::RenderPlatformWindowsDefault();
                glfwMakeContextCurrent(backup_context);
            }

            glfwSwapBuffers(window);
        }

        renderer.Shutdown();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "error: %s\n", error.what());
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
