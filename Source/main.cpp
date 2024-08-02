
#include "Util/Options.hpp"
#include "Util/FrameBuffer.hpp"
#include "Core/ThreadPool.hpp"
#include "Core/Scene.hpp"
#include "Core/Camera.hpp"

#include "GLFW/glfw3.h"

#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"

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
    glm::vec4* frame_memory;
    uint32_t* sample_count_buffer;
    glm::vec4* variance;

    {
        Util::Options options(argv, argc);

        glm::ivec2 resolution = options.get_option<Util::Option::kResolution>();

        frame_memory = new glm::vec4[resolution.x * resolution.y];
        memset(frame_memory, ~0, resolution.x * resolution.y * 4 * 4);

        sample_count_buffer = new uint32_t[resolution.x * resolution.y];
        memset(sample_count_buffer, 0, resolution.x * resolution.y * 4);

        variance = new glm::vec4[resolution.x * resolution.y];
        memset(variance, 0, resolution.x * resolution.y * 4 * 4);

        std::filesystem::path scene_file = options.get_option<Util::Option::kSceneFile>();
        ThreadPool threadPool{};
        std::unique_ptr<Scene::Scene> scene{};
        if(scene_file.extension() == ".json")
        {
            scene = std::make_unique<Scene::Scene>(threadPool, scene_file);
        }
        else
        {
            Assimp::Importer importer;

            const aiScene* assimp_scene = importer.ReadFile(scene_file.string().c_str(),
                                                     aiProcess_Triangulate |
                                                     aiProcess_JoinIdenticalVertices |
                                                     aiProcess_GenNormals |
                                                     aiProcess_CalcTangentSpace |
                                                     aiProcess_GlobalScale |
                                                     aiProcess_FlipUVs |
                                                     aiProcess_GenBoundingBoxes);

            scene = std::make_unique<Scene::Scene>(threadPool, scene_file.parent_path(),  assimp_scene, options);
        }

        uint32_t sample_count = 256;
        if(options.has_option(Util::Option::kSampleCount))
        {
            sample_count = options.get_option<Util::Option::kSampleCount>();
        }

        Scene::RenderParams params{};
        params.m_Height = resolution.y;
        params.m_Width = resolution.x;
        params.m_maxRayDepth = 10;
        params.m_maxSamples = sample_count;
        params.m_Pixels = frame_memory;
        params.m_SampleCount = sample_count_buffer;
        params.m_variance = variance;

        const glm::vec3 camera_pos = options.get_option<Util::Option::kCameraPosition>();
        const glm::vec3 camera_dir = options.get_option<Util::Option::kCameraDirection>();

        Scene::Camera camera(camera_pos, camera_dir, resolution.x / resolution.y, 0.1f, 10000.0f);

        if(options.has_option(Util::Option::kCameraName))
        {
            camera = *scene->get_camera(options.get_option<Util::Option::kCameraName>());
        }
        camera.set_resolution(glm::uvec2(resolution));

        if(options.has_option(Util::Option::kOutputFile))
        {
            const std::string output_file_path = options.get_option<Util::Option::kOutputFile>();

            scene->render_scene_to_file(camera, params, output_file_path.c_str());
        }
        else
        {
            glfwInit();

            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
            auto* window = glfwCreateWindow(1920, 1080, "Pico", nullptr, nullptr);

            glfwSetErrorCallback(error_callback);
            glfwMakeContextCurrent(window);
            glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

            gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

            {
                Util::FrameBuffer frame_buffer(resolution.x, resolution.y);

                bool should_quit = false;
                auto render_func = [](std::unique_ptr<Scene::Scene>& scene, const Scene::Camera& cam, const Scene::RenderParams& params, bool* quit)
                {
                    scene->render_scene_to_memory(cam, params, quit);
                };

                std::thread render_thread(render_func, std::ref(scene), std::ref(camera), std::ref(params), &should_quit);

                while(!glfwWindowShouldClose(window))
                {
                    frame_buffer.set_image(frame_memory);

                    glfwSwapBuffers(window);
                    glfwPollEvents();
                }

                should_quit = true;
                render_thread.join();
            }

            glfwDestroyWindow(window);

            glfwTerminate();
        }
    }

    delete[] frame_memory;
    delete[] sample_count_buffer;
    delete[] variance;
}
