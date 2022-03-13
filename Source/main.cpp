
#include "Util/Options.hpp"
#include "Util/FrameBuffer.hpp"
#include "Core/ThreadPool.hpp"
#include "Core/Scene.hpp"
#include "Core/Camera.hpp"

#include "GLFW/glfw3.h"

void framebuffer_size_callback(GLFWwindow*, int width, int height)
{
    glViewport(0, 0, width, height);
}

void error_callback(int, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}


int main(int argc, const char **argv)
{
    {
        Util::Options options(argv, argc);

        glfwInit();

        uint32_t* frame_memory = new uint32_t[512 * 512];
        memset(frame_memory, ~0, 512 * 512 * 4);

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        auto* window = glfwCreateWindow(1920, 1080, "Pico", nullptr, nullptr);

        glfwSetErrorCallback(error_callback);
        glfwMakeContextCurrent(window);
        glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

        gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

        Util::FrameBuffer frame_buffer(512, 512);

        ThreadPool threadPool{};
        Scene::Scene scene(threadPool, "");

        Scene::RenderParams params{};
        params.m_Height = 512;
        params.m_Width = 512;
        params.m_maxRayDepth = 10;
        params.m_maxSamples = 3;
        params.m_Pixels = frame_memory;

        Scene::Camera camera{glm::vec3{0.0f, 0.0f, -30.0f}, glm::vec3{0.0f, 0.0f, 1.0f}, 1.0f, 0.01, 100.0f};

        scene.render_scene_to_memory(camera, params);

        while(!glfwWindowShouldClose(window))
        {
            frame_buffer.set_image(frame_memory);

            glfwSwapBuffers(window);
            glfwPollEvents();
        }

        glfwDestroyWindow(window);
        delete[] frame_memory;
    }
    glfwTerminate();
}
