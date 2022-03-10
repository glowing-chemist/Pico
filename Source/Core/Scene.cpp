#include "Core/Scene.hpp"
#include "Core/Camera.hpp"
#include "Core/vectorUtils.hpp"
#include "Render/Sampler.hpp"
#include "Render/PBR.hpp"
#include "Render/Integrators.hpp"
#include "Render/BasicMaterials.hpp"
#include "Core/LowerLevelImplicitShapesBVH.hpp"

#include <algorithm>
#include <memory>
#include <thread>

#include "stbi_image_write.h"
#include "stb_image.h"
#include "glm/ext.hpp"
#include "glm/gtx/compatibility.hpp"
#include "glm/gtc/matrix_transform.hpp"

namespace Scene
{

    enum MaterialType : uint32_t
    {
        Albedo = 1,
        Diffuse = 1 << 1,
        Normals = 1 << 2,
        Roughness = 1 << 3,
        Gloss = 1 << 4,
        Metalness = 1 << 5,
        Specular = 1 << 6,
        CombinedMetalnessRoughness = 1 << 7,
        AmbientOcclusion = 1 << 8,
        Emisive = 1 << 9,
        CombinedSpecularGloss = 1 << 10,
        HeightMap = 1 << 11,

        AlphaTested = 1 << 20,
        Transparent = 1 << 21
    };

    Scene::Scene(ThreadPool& pool, const std::string& sceneFile) :
        m_threadPool(pool)
    {
        std::unique_ptr<Core::Material> plastic_material = std::make_unique<Render::MattPlasticMaterial>(glm::vec3{0.5f, 0.1f, 0.3f});
        std::unique_ptr<Core::Material> metal_material = std::make_unique<Render::SmoothMetalMaterial>(glm::vec3{0.8f, 0.8f, 0.8f});

        const auto plastic_mat_id = m_material_manager.add_material(plastic_material);
        const auto metal_mat_id   = m_material_manager.add_material(metal_material);

        std::shared_ptr<Core::BVH::LowerLevelBVH> sphere_bvh = std::make_shared<Core::BVH::LowerLevelSphereBVH>(5.0f);

        m_bvh.add_lower_level_bvh(sphere_bvh, glm::mat4x4(1.0f), metal_mat_id);
        //m_bvh.add_lower_level_bvh(sphere_bvh, glm::translate(glm::mat4x4(1.0f), glm::vec3(5.0f, 5.0f, 0.0f)), plastic_mat_id);

        //m_bvh.add_lower_level_bvh(sphere_bvh, glm::translate(glm::mat4x4(1.0f), glm::vec3(5.0f, -5.0f, 0.0f)), plastic_mat_id);

        //m_bvh.add_lower_level_bvh(sphere_bvh, glm::translate(glm::mat4x4(1.0f), glm::vec3(-5.0f, 5.0f, 0.0f)), metal_mat_id);

        //m_bvh.add_lower_level_bvh(sphere_bvh, glm::translate(glm::mat4x4(1.0f), glm::vec3(-5.0f, -5.0f, 0.0f)), metal_mat_id);

        m_bvh.build();

        // Load test skybox
        std::array<std::string, 6> skyboxPaths{ "./Skybox/px.png",
                                                "./Skybox/nx.png",
                                                "./Skybox/py.png",
                                                "./Skybox/ny.png",
                                                "./Skybox/pz.png",
                                                "./Skybox/nz.png"};

        std::vector<unsigned char> skyboxData{};

        for(const std::string& file : skyboxPaths)
        {
            int x, y, comp;
            auto* data = stbi_load(file.c_str(), &x, &y, &comp, 4);

            skyboxData.insert(skyboxData.end(), data, data + (x * y * comp));
        }

        // create CPU skybox.
        Core::ImageExtent extent = {1024, 1024, 6};
        unsigned char* data = new unsigned char[skyboxData.size()];
        std::memcpy(data, skyboxData.data(), skyboxData.size());
        mSkybox = std::make_shared<Core::ImageCube>(data, extent, Core::Format::kRBGA_8UNorm);
    }

    void Scene::render_scene_to_memory(const Camera& camera, const RenderParams& params)
    {
        const glm::vec3 forward = camera.getDirection();
        const glm::vec3 up = camera.getUp();
        const glm::vec3 right = camera.getRight();

        const glm::vec3 origin = camera.getPosition();
        const float farPlane = camera.getFarPlane();
        const float aspect = camera.getAspect();

        auto trace_ray = [&](const uint32_t pix, const uint32_t piy) -> glm::vec4
        {
            glm::vec3 dir = {((float(pix) / float(params.m_Width)) - 0.5f) * aspect, (float(piy) / float(params.m_Height)) - 0.5f, 1.0f};
            dir = glm::normalize((dir.z * forward) + (dir.y * up) + (dir.x * right));

            Core::Ray ray;
            ray.mDirection = dir;

            ray.mOrigin = glm::vec4(origin, 1.0f);

            ray.mLenght = farPlane;

            std::unique_ptr<Render::Diffuse_Sampler> diffuse_sampler = std::make_unique<Render::Hammersley_GGX_Diffuse_Sampler>(100);
            std::unique_ptr<Render::Specular_Sampler> specular_sampler = std::make_unique<Render::Hammersley_GGX_Specular_Sampler>(100);

            Render::Monte_Carlo_Integrator integrator(m_bvh, m_material_manager, mSkybox, diffuse_sampler, specular_sampler);

            return integrator.integrate_ray(ray, params.m_maxRayDepth, params.m_maxSamples);
        };

        auto trace_rays = [&](const uint32_t start, const uint32_t stepSize)
        {
            uint32_t location = start;
            while(location < (params.m_Height * params.m_Width))
            {
                const uint32_t pix = location % params.m_Height;
                const uint32_t piy = location / params.m_Height;
                glm::vec4 result =  trace_ray(pix, piy);
                //result = glm::clamp(glm::vec4(0.0f), glm::vec4(1.0f), result);
                const uint32_t colour = Core::pack_colour(result);
                params.m_Pixels[(pix + (piy * params.m_Height))] = colour;

                location += stepSize;
            }
        };

        const uint32_t processor_count = m_threadPool.get_worker_count(); // use this many threads for tracing rays.
        std::vector<std::future<void>> handles{};
        for(uint32_t i = 1; i < processor_count; ++i)
        {
            handles.push_back(m_threadPool.add_task(trace_rays, i, processor_count));
        }

        trace_rays(0, processor_count);

        for(auto& thread : handles)
            thread.wait();
    }


    void Scene::render_scene_to_file(const Camera& camera, RenderParams& params, const char* path)
    {
        render_scene_to_memory(camera, params);

        stbi_write_jpg(path, params.m_Width, params.m_Height, 4, params.m_Pixels, 100);
    }

}
