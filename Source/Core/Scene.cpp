#include "Core/Scene.hpp"
#include "Core/Camera.hpp"
#include "Core/vectorUtils.hpp"
#include "Render/Sampler.hpp"
#include "Render/PBR.hpp"
#include "Render/Integrators.hpp"
#include "Render/BasicMaterials.hpp"
#include "Core/LowerLevelImplicitShapesBVH.hpp"
#include "Core/LowerLevelMeshBVH.hpp"
#include "Render/BasicMaterials.hpp"

#include <algorithm>
#include <memory>
#include <thread>
#include <fstream>

#include "stbi_image_write.h"
#include "stb_image.h"
#include "glm/ext.hpp"
#include "glm/gtx/compatibility.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "assimp/pbrmaterial.h"
#include "assimp/scene.h"


namespace Scene
{
    Scene::Scene(ThreadPool& pool, const std::filesystem::path& path) :
        mWorkingDir{path.parent_path()},
        m_threadPool(pool)
    {
        std::ifstream sceneFile;
        sceneFile.open(path);

        Json::Value sceneRoot;
        sceneFile >> sceneRoot;

        std::shared_ptr<Core::BVH::LowerLevelBVH> sphere = std::make_shared<Core::BVH::LowerLevelSphereBVH>(1.0f);
        mInstanceIDs["Sphere"] = m_lowerLevelBVhs.size();
        m_lowerLevelBVhs.push_back(sphere);

        std::array<std::string, 5> sections{"GLOBALS", "MESH", "MATERIALS", "INSTANCE",
                                            "CAMERA"};
        std::array<void(Scene::*)(const std::string&, const Json::Value&), 5> sectionFunctions
            {
                &Scene::process_globals, &Scene::add_mesh, &Scene::add_material, &Scene::add_mesh_instance,
                &Scene::add_camera
            };

        for(uint32_t i = 0; i < sections.size(); ++i)
        {
            if(sceneRoot.isMember(sections[i]))
            {
                for(std::string& entityName : sceneRoot[sections[i]].getMemberNames())
                    std::invoke(sectionFunctions[i], this, entityName, sceneRoot[sections[i]][entityName]);
            }

        }

        m_bvh.build();
    }

    Scene::Scene(ThreadPool& pool, const Assimp::Importer* scene) :
        m_threadPool(pool)
    {

    }

    void Scene::render_scene_to_memory(const Camera& camera, const RenderParams& params)
    {        
        const glm::vec3 forward = camera.getDirection();
        const glm::vec3 up = camera.getUp();
        const glm::vec3 right = camera.getRight();

        const glm::vec3 origin = camera.getPosition();
        const float farPlane = camera.getFarPlane();
        const float aspect = camera.getAspect();

        auto trace_ray = [&](const uint32_t pix, const uint32_t piy, const uint64_t seed) -> glm::vec4
        {
            glm::vec3 dir = {((float(pix) / float(params.m_Width)) - 0.5f) * aspect, (float(piy) / float(params.m_Height)) - 0.5f, 1.0f};
            dir = glm::normalize((dir.z * forward) + (dir.y * up) + (dir.x * right));

            Core::Ray ray;
            ray.mDirection = dir;

            ray.mOrigin = glm::vec4(origin, 1.0f);

            ray.mLenght = farPlane;

            std::unique_ptr<Render::Diffuse_Sampler> diffuse_sampler = std::make_unique<Render::Hammersley_GGX_Diffuse_Sampler>(100, seed);
            std::unique_ptr<Render::Specular_Sampler> specular_sampler = std::make_unique<Render::Hammersley_GGX_Specular_Sampler>(100, seed);

            Render::Monte_Carlo_Integrator integrator(m_bvh, m_material_manager, mSkybox, diffuse_sampler, specular_sampler, seed);

            return integrator.integrate_ray(ray, params.m_maxRayDepth, params.m_sample);
        };

        auto trace_rays = [&](const uint32_t start, const uint32_t stepSize)
        {
            std::random_device random_device{};
            std::mt19937 random_generator(random_device());

            uint32_t location = start;
            while(location < (params.m_Height * params.m_Width))
            {                
                const uint32_t pix = location % params.m_Height;
                const uint32_t piy = location / params.m_Height;

                const uint32_t pixel_index = pix + (piy * params.m_Height);

                if(params.m_SampleCount[pixel_index] >= params.m_maxSamples)
                {
                    location += stepSize;
                    continue;
                }

                uint32_t prev_sample_count = params.m_SampleCount[pixel_index];
                glm::vec4 pixel_result =  trace_ray(pix, piy, random_generator());

                if(prev_sample_count < 1)
                {
                    params.m_SampleCount[pixel_index] = 1;
                }
                else
                {
                    const glm::vec4& previous_pixle = params.m_Pixels[pixel_index];
                    pixel_result = previous_pixle + ((pixel_result - previous_pixle) * (1.0f / prev_sample_count));
                    params.m_SampleCount[pixel_index] += 1;
                }

                params.m_Pixels[pixel_index] = pixel_result;

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


    // Scene loading functions.
    void Scene::add_mesh(const std::string& name, const Json::Value& entry)
    {
        const std::string path = mWorkingDir / entry["Path"].asString();

        Assimp::Importer importer;

        const aiScene* scene = importer.ReadFile(path.c_str(),
                                                 aiProcess_Triangulate |
                                                 aiProcess_JoinIdenticalVertices |
                                                 aiProcess_GenNormals |
                                                 aiProcess_CalcTangentSpace |
                                                 aiProcess_GlobalScale |
                                                 aiProcess_FlipUVs |
                                                 aiProcess_GenBoundingBoxes);


        auto mesh_bvh = std::make_shared<Core::BVH::LowerLevelMeshBVH>(scene->mMeshes[1]);

        mInstanceIDs[name] = m_lowerLevelBVhs.size();
        m_lowerLevelBVhs.push_back(mesh_bvh);
    }


    void Scene::add_mesh_instance(const std::string&, const Json::Value& entry)
    {
        const std::string assetName = entry["Asset"].asString();
        const uint32_t assetID = mInstanceIDs[assetName];

        glm::vec3 position{0.0f, 0.0f, 0.0f};
        glm::vec3 scale{1.0f, 1.0f, 1.0f};
        glm::quat   rotation{1.0f, 0.0f, 0.0f, 0.f};

        if(entry.isMember("Position"))
        {
            const Json::Value& positionEntry = entry["Position"];
            position.x = positionEntry[0].asFloat();
            position.y = positionEntry[1].asFloat();
            position.z = positionEntry[2].asFloat();
        }

        if(entry.isMember("Scale"))
        {
            const Json::Value& scaleEntry = entry["Scale"];
            scale.x = scaleEntry[0].asFloat();
            scale.y = scaleEntry[1].asFloat();
            scale.z = scaleEntry[2].asFloat();
        }

        if(entry.isMember("Rotation"))
        {
            const Json::Value& rotationEntry = entry["Rotation"];
            rotation.x = rotationEntry[0].asFloat();
            rotation.y = rotationEntry[1].asFloat();
            rotation.z = rotationEntry[2].asFloat();
            rotation.w = rotationEntry[3].asFloat();
            rotation = glm::normalize(rotation);
        }

        uint32_t material = 0;
        if(entry.isMember("Material"))
        {
            const std::string materialName = entry["Material"].asString();
            material = mMaterials[materialName];
        }

        const glm::mat4x4 transform =  glm::translate(glm::mat4x4(1.0f), position) *
                                    glm::mat4_cast(rotation) *
                                    glm::scale(glm::mat4x4(1.0f), scale);

        m_bvh.add_lower_level_bvh(m_lowerLevelBVhs[assetID], transform, material);
    }

    void Scene::add_material(const std::string &name, const Json::Value &entry)
    {
        std::unique_ptr<Core::Material> material;

        if(entry["Type"].asString() == "Metalic")
        {
            std::unique_ptr<Core::Image2D> albedo{};
            if(entry.isMember("Albedo"))
            {
                const std::string path = entry["Albedo"].asString();
                albedo = std::make_unique<Core::Image2D>((mWorkingDir / path).string());
            }

            std::unique_ptr<Core::Image2D> roughness{};
            if(entry.isMember("Roughness"))
            {
                const std::string path = entry["Roughness"].asString();
                roughness = std::make_unique<Core::Image2D>((mWorkingDir / path).string());
            }

            std::unique_ptr<Core::Image2D> metalness{};
            if(entry.isMember("Metalness"))
            {
                const std::string path = entry["Metalness"].asString();
                metalness = std::make_unique<Core::Image2D>((mWorkingDir / path).string());
            }

            std::unique_ptr<Core::Image2D> emmissive{};
            if(entry.isMember("Emissive"))
            {
                const std::string path = entry["Emissive"].asString();
                emmissive = std::make_unique<Core::Image2D>((mWorkingDir / path).string());
            }

            material = std::make_unique<Render::MetalnessRoughnessMaterial>(albedo, metalness, roughness, emmissive);
        }
        else if(entry["Type"].asString() == "Gloss")
        {
            std::unique_ptr<Core::Image2D> diffuse{};
            if(entry.isMember("Diffuse"))
            {
                const std::string path = entry["Diffuse"].asString();
                diffuse = std::make_unique<Core::Image2D>((mWorkingDir / path).string());
            }

            std::unique_ptr<Core::Image2D> specular{};
            if(entry.isMember("Specular"))
            {
                const std::string path = entry["Specular"].asString();
                specular = std::make_unique<Core::Image2D>((mWorkingDir / path).string());
            }

            std::unique_ptr<Core::Image2D> gloss{};
            if(entry.isMember("Gloss"))
            {
                const std::string path = entry["Gloss"].asString();
                gloss = std::make_unique<Core::Image2D>((mWorkingDir / path).string());
            }

            std::unique_ptr<Core::Image2D> emmissive{};
            if(entry.isMember("Emissive"))
            {
                const std::string path = entry["Emissive"].asString();
                emmissive = std::make_unique<Core::Image2D>((mWorkingDir / path).string());
            }

            material = std::make_unique<Render::SpecularGlossMaterial>(diffuse, specular, gloss, emmissive);
        }
        else if(entry["Type"].asString() == "Constant")
        {
            glm::vec3 albedo(0.0f);
            if(entry.isMember("Albedo"))
            {
                const Json::Value& albedo_enrty = entry["Albedo"];
                albedo.x = albedo_enrty[0].asFloat();
                albedo.y = albedo_enrty[1].asFloat();
                albedo.z = albedo_enrty[2].asFloat();
            }

            float metalness;
            if(entry.isMember("Metalness"))
            {
                metalness = entry["Metalness"].asFloat();
            }

            float roughness = 0.0f;
            if(entry.isMember("Roughness"))
            {
                roughness = entry["Roughness"].asFloat();
            }

            glm::vec3 emmissive(0.0f);
            if(entry.isMember("Emmissive"))
            {
                const Json::Value& albedo_enrty = entry["Emmissive"];
                emmissive.x = albedo_enrty[0].asFloat();
                emmissive.y = albedo_enrty[1].asFloat();
                emmissive.z = albedo_enrty[2].asFloat();
            }

            material = std::make_unique<Render::ConstantMetalnessRoughnessMaterial>(albedo, metalness, roughness, emmissive);
        }

        mMaterials[name] = m_material_manager.add_material(material);

    }

    void Scene::add_camera(const std::string& name, const Json::Value& entry)
    {
        Camera newCamera({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 1.0f);
        if(entry.isMember("Position"))
        {
            const Json::Value& positionEntry = entry["Position"];
            glm::vec3 position;
            position.x = positionEntry[0].asFloat();
            position.y = positionEntry[1].asFloat();
            position.z = positionEntry[2].asFloat();

            newCamera.setPosition(position);
        }

        if(entry.isMember("Direction"))
        {
            const Json::Value& directionEntry = entry["Direction"];
            glm::vec3 direction;
            direction.x = directionEntry[0].asFloat();
            direction.y = directionEntry[1].asFloat();
            direction.z = directionEntry[2].asFloat();
            direction = glm::normalize(direction);

            newCamera.setDirection(direction);
        }

        if(entry.isMember("Aspect"))
        {
            const float aspect = entry["Aspect"].asFloat();
            newCamera.setAspect(aspect);
        }

        if(entry.isMember("NearPlane"))
        {
            const float NearPlane = entry["NearPlane"].asFloat();
            newCamera.setNearPlane(NearPlane);
        }

        if(entry.isMember("FarPlane"))
        {
            const float FarPlane = entry["FarPlane"].asFloat();
            newCamera.setFarPlane(FarPlane);
        }

        if(entry.isMember("FOV"))
        {
            const float fov = entry["FOV"].asFloat();
            newCamera.setFOVDegrees(fov);
        }

        mCamera.insert({name, newCamera});
    }


    void Scene::process_globals(const std::string&, const Json::Value& entry)
    {
        if(entry.isMember("Skybox"))
        {
            const Json::Value skyboxes = entry["Skybox"];
            std::array<std::string, 6> skyboxPaths{};
            for(uint32_t i = 0; i < 6; ++i)
            {
                skyboxPaths[i] = (mWorkingDir / skyboxes[i].asString()).string();
            }

            std::vector<unsigned char> skyboxData{};

            uint32_t width, height;
            for(const std::string& file : skyboxPaths)
            {
                int x, y, comp;
                auto* data = stbi_load(file.c_str(), &x, &y, &comp, 4);
                width = x;
                height = y;

                skyboxData.insert(skyboxData.end(), data, data + (x * y * 4));
            }

            // create CPU skybox.
            Core::ImageExtent extent = {width, height, 6};
            unsigned char* data = new unsigned char[skyboxData.size()];
            std::memcpy(data, skyboxData.data(), skyboxData.size());
            mSkybox = std::make_shared<Core::ImageCube>(data, extent, Core::Format::kRBGA_8UNorm);
        }
    }
}
