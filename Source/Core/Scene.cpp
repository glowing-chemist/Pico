#include "Core/Scene.hpp"
#include "Core/Camera.hpp"
#include "Core/vectorUtils.hpp"
#include "Core/Asserts.hpp"
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
        m_threadPool(pool),
        m_file_mapper{}
    {
        m_file_mapper = std::make_unique<Core::File_System_Mappings>(path.parent_path());

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

    Scene::Scene(ThreadPool& pool, const std::filesystem::path& working_dir, const aiScene* scene) :
        mWorkingDir(working_dir),
        m_threadPool(pool)
    {
        PICO_ASSERT(scene);

        m_file_mapper = std::make_unique<Core::File_System_Mappings>(working_dir);

        // Create a black cubemap.
        unsigned char* black_cube_map = static_cast<unsigned char*>(malloc(sizeof(unsigned char) * 24));
        memset(black_cube_map, 0, 24);
        Core::ImageExtent extent{1, 1, 1};
        mSkybox = std::make_shared<Core::ImageCube>(black_cube_map, extent, Core::Format::kRBGA_8UNorm);

        for(uint32_t i_mat = 0; i_mat < scene->mNumMaterials; ++i_mat)
        {
            aiMaterial* mat = scene->mMaterials[i_mat];

            add_material(mat);
        }

        parse_node(scene, scene->mRootNode, aiMatrix4x4{});

        m_bvh.build();
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
            ray.push_index_of_refraction(1.0f);

            Render::Monte_Carlo_Integrator integrator(m_bvh, m_material_manager, m_lights, mSkybox, seed);

            return integrator.integrate_ray(ray, params.m_maxRayDepth, params.m_sample);
        };

        auto trace_rays = [&](const glm::uvec2 start, const glm::uvec2& tile_size, const size_t random_seed)
        {
            std::mt19937 random_generator(random_seed);

            glm::uvec2 location = start;
            glm::uvec2 offset = {0, 0};
            while(offset.x < tile_size.x && offset.y < tile_size.y && location.x < params.m_Width && location.y < params.m_Height)
            {
                const uint32_t flat_location = (location.y * params.m_Width) + location.x;

                if(params.m_SampleCount[flat_location] >= params.m_maxSamples)
                {
                    if(offset.x ==  (tile_size.x - 1))
                    {
                        offset.y += 1;
                        offset.x = 0;
                    }
                    else
                        offset.x += 1;
                    continue;
                }

                uint32_t prev_sample_count = params.m_SampleCount[flat_location];
                glm::vec4 pixel_result =  trace_ray(location.x, location.y, random_generator());
                pixel_result = glm::clamp(pixel_result, 0.0f, 1.0f);

                if(prev_sample_count < 1)
                {
                    params.m_SampleCount[flat_location] = 1;
                }
                else
                {
                    const glm::vec4& previous_pixle = params.m_Pixels[flat_location];
                    pixel_result = previous_pixle + ((pixel_result - previous_pixle) * (1.0f / prev_sample_count));
                    params.m_SampleCount[flat_location] += 1;
                }

                params.m_Pixels[flat_location] = pixel_result;

                if(offset.x ==  (tile_size.x - 1))
                {
                    offset.y += 1;
                    offset.x = 0;
                }
                else
                    offset.x += 1;

                location = start + offset;
            }
        };

        std::random_device random_device{};
        std::mt19937 random_generator(random_device());

        const uint32_t processor_count = m_threadPool.get_worker_count(); // use this many threads for tracing rays.
        const uint32_t xCount = std::sqrt(processor_count);
        const uint32_t yCount = processor_count / xCount;
        PICO_ASSERT(xCount * yCount == processor_count);

        glm::uvec2 tile_size{(params.m_Width + (xCount - 1)) / xCount, (params.m_Height + (yCount - 1)) / yCount};

        std::vector<std::future<void>> handles{};
        for(uint32_t i = 1; i < processor_count; ++i)
        {
            handles.push_back(m_threadPool.add_task(trace_rays, glm::uvec2((i % xCount) * tile_size.x, (i / xCount) * tile_size.y), tile_size, random_generator()));
        }

        trace_rays(glm::uvec2(0, 0), tile_size, random_generator());

        for(auto& thread : handles)
            thread.wait();
    }


    void Scene::render_scene_to_file(const Camera& camera, RenderParams& params, const char* path)
    {
        render_scene_to_memory(camera, params);

        std::vector<uint32_t> buffer{};
        std::transform(params.m_Pixels, params.m_Pixels + params.m_Height * params.m_Width, std::back_inserter(buffer), [](glm::vec4& pixel) { return Core::pack_colour(pixel); });

        stbi_write_jpg(path, params.m_Width, params.m_Height, 4, buffer.data(), 100);
    }


    // Scene loading functions.
    void Scene::add_mesh(const std::string& name, const Json::Value& entry)
    {
        const std::string path = m_file_mapper->resolve_path(entry["Path"].asString());

        Assimp::Importer importer;

        const aiScene* scene = importer.ReadFile(path.c_str(),
                                                 aiProcess_Triangulate |
                                                 aiProcess_JoinIdenticalVertices |
                                                 aiProcess_GenNormals |
                                                 aiProcess_CalcTangentSpace |
                                                 aiProcess_GlobalScale |
                                                 aiProcess_FlipUVs |
                                                 aiProcess_GenBoundingBoxes);


        auto mesh_bvh = std::make_shared<Core::BVH::LowerLevelMeshBVH>(scene->mMeshes[0]);

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


        std::shared_ptr<Render::BSRDF> bsrdf;
        if(entry.isMember("BSRDF"))
        {
            const std::string bsrdf_type = entry["BSRDF"].asString();
            if(bsrdf_type == "Diffuse")
            {
                std::unique_ptr<Render::Distribution> distribution = std::make_unique<Render::Cos_Weighted_Hemisphere_Distribution>();
                bsrdf = std::make_shared<Render::Diffuse_BRDF>(distribution, m_material_manager, material);
            }
            else if(bsrdf_type == "Specular")
            {
                std::unique_ptr<Render::Distribution> distribution = std::make_unique<Render::Beckmann_All_Microfacet_Distribution>();
                bsrdf = std::make_shared<Render::Specular_BRDF>(distribution, m_material_manager, material);
            }
            else if(bsrdf_type == "Transparent")
            {
                std::unique_ptr<Render::Distribution> distribution = std::make_unique<Render::Beckmann_All_Microfacet_Distribution>();
                bsrdf = std::make_shared<Render::Transparent_BTDF>(distribution, m_material_manager, material);
            }
            else if(bsrdf_type == "Fresnel")
            {
                std::unique_ptr<Render::Distribution> specular_distribution = std::make_unique<Render::Beckmann_All_Microfacet_Distribution>();
                std::unique_ptr<Render::Distribution> transmission_distribution = std::make_unique<Render::Beckmann_All_Microfacet_Distribution>();
                bsrdf = std::make_shared<Render::Fresnel_BTDF>(specular_distribution, transmission_distribution, m_material_manager, material);
            }
            else if(bsrdf_type == "Light")
            {
                bsrdf = std::make_shared<Render::Light_BRDF>(m_material_manager, material);

                m_lowerLevelBVhs[assetID]->generate_sampling_data();
                m_lights.push_back({ transform, glm::inverse(transform), m_lowerLevelBVhs[assetID] });
            }
            else if(bsrdf_type == "Delta")
            {
                bsrdf = std::make_shared<Render::Specular_Delta_BRDF>(m_material_manager, material);
            }
        }
        PICO_ASSERT(bsrdf);

        m_bvh.add_lower_level_bvh(m_lowerLevelBVhs[assetID], transform, bsrdf);
    }

    void Scene::add_material(const std::string& name, const Json::Value& entry)
    {
        std::unique_ptr<Core::Material> material;

        if(entry["Type"].asString() == "Metalic")
        {
            std::unique_ptr<Core::Image2D> albedo{};
            if(entry.isMember("Albedo"))
            {
                const std::string path = m_file_mapper->resolve_path(entry["Albedo"].asString());
                albedo = std::make_unique<Core::Image2D>(path);
            }

            std::unique_ptr<Core::Image2D> roughness{};
            if(entry.isMember("Roughness"))
            {
                const std::string path = m_file_mapper->resolve_path(entry["Roughness"].asString());
                roughness = std::make_unique<Core::Image2D>(path);
            }

            std::unique_ptr<Core::Image2D> metalness{};
            if(entry.isMember("Metalness"))
            {
                const std::string path = m_file_mapper->resolve_path(entry["Metalness"].asString());
                metalness = std::make_unique<Core::Image2D>(path);
            }

            std::unique_ptr<Core::Image2D> emmissive{};
            if(entry.isMember("Emissive"))
            {
                const std::string path = m_file_mapper->resolve_path(entry["Emissive"].asString());
                emmissive = std::make_unique<Core::Image2D>(path);
            }

            material = std::make_unique<Render::MetalnessRoughnessMaterial>(albedo, metalness, roughness, emmissive);
        }
        else if(entry["Type"].asString() == "Gloss")
        {
            std::unique_ptr<Core::Image2D> diffuse{};
            if(entry.isMember("Diffuse"))
            {
                const std::string path = m_file_mapper->resolve_path(entry["Diffuse"].asString());
                diffuse = std::make_unique<Core::Image2D>(path);
            }

            std::unique_ptr<Core::Image2D> specular{};
            if(entry.isMember("Specular"))
            {
                const std::string path = m_file_mapper->resolve_path(entry["Specular"].asString());
                specular = std::make_unique<Core::Image2D>(path);
            }

            std::unique_ptr<Core::Image2D> gloss{};
            if(entry.isMember("Gloss"))
            {
                const std::string path = m_file_mapper->resolve_path(entry["Gloss"].asString());
                gloss = std::make_unique<Core::Image2D>(path);
            }

            std::unique_ptr<Core::Image2D> emmissive{};
            if(entry.isMember("Emissive"))
            {
                const std::string path = m_file_mapper->resolve_path(entry["Emissive"].asString());
                emmissive = std::make_unique<Core::Image2D>(path);
            }

            material = std::make_unique<Render::SpecularGlossMaterial>(diffuse, specular, gloss, emmissive);
        }
        else if(entry["Type"].asString() == "Constant")
        {
            if(entry.isMember("Albedo"))
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
                if(entry.isMember("Emissive"))
                {
                    const Json::Value& albedo_enrty = entry["Emissive"];
                    emmissive.x = albedo_enrty[0].asFloat();
                    emmissive.y = albedo_enrty[1].asFloat();
                    emmissive.z = albedo_enrty[2].asFloat();
                }

                if(entry.isMember("IndexOfRefraction"))
                {
                    const float index_of_refraction = entry["IndexOfRefraction"].asFloat();
                    material = std::make_unique<Render::ConstantTransparentMetalnessRoughnessMaterial>(albedo, metalness, roughness, 1.0f, index_of_refraction);
                }
                else
                    material = std::make_unique<Render::ConstantMetalnessRoughnessMaterial>(albedo, metalness, roughness, emmissive);
            }
            else if(entry.isMember("Diffuse") || entry.isMember("Specular"))
            {
                glm::vec3 diffuse(0.0f);
                if(entry.isMember("Diffuse"))
                {
                    const Json::Value& albedo_enrty = entry["Diffuse"];
                    diffuse.x = albedo_enrty[0].asFloat();
                    diffuse.y = albedo_enrty[1].asFloat();
                    diffuse.z = albedo_enrty[2].asFloat();
                }

                glm::vec3 specular(0.0f);
                if(entry.isMember("Specular"))
                {
                    const Json::Value& albedo_enrty = entry["Specular"];
                    specular.x = albedo_enrty[0].asFloat();
                    specular.y = albedo_enrty[1].asFloat();
                    specular.z = albedo_enrty[2].asFloat();
                }

                float gloss = 0.0f;
                if(entry.isMember("Gloss"))
                {
                    gloss = entry["Gloss"].asFloat();
                }

                glm::vec3 emmissive(0.0f);
                if(entry.isMember("Emissive"))
                {
                    const Json::Value& albedo_enrty = entry["Emissive"];
                    emmissive.x = albedo_enrty[0].asFloat();
                    emmissive.y = albedo_enrty[1].asFloat();
                    emmissive.z = albedo_enrty[2].asFloat();
                }

                if(entry.isMember("IndexOfRefraction"))
                {
                    const float index_of_refraction = entry["IndexOfRefraction"].asFloat();
                    material = std::make_unique<Render::ConstantTransparentDiffuseSpecularMaterial>(diffuse, specular, gloss, 1.0f, index_of_refraction);
                }
                else
                    material = std::make_unique<Render::ConstantDiffuseSpecularMaterial>(diffuse, specular, gloss, emmissive);
            }
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
        else
        {
            // Create a black cubemap.
            unsigned char* black_cube_map = static_cast<unsigned char*>(malloc(sizeof(unsigned char) * 24));
            memset(black_cube_map, 0, 24);
            Core::ImageExtent extent{1, 1, 1};
            mSkybox = std::make_shared<Core::ImageCube>(black_cube_map, extent, Core::Format::kRBGA_8UNorm);
        }
    }

    void Scene::parse_node(const aiScene* scene,
                          const aiNode* node,
                          const aiMatrix4x4& parentTransofrmation)
    {
        aiMatrix4x4 transformation = parentTransofrmation * node->mTransformation;

        for(uint32_t i = 0; i < node->mNumMeshes; ++i)
        {
            const aiMesh* current_mesh = scene->mMeshes[node->mMeshes[i]];
            uint32_t material_index = current_mesh->mMaterialIndex;

            std::shared_ptr<Core::BVH::LowerLevelBVH> meshBVH = std::make_shared<Core::BVH::LowerLevelMeshBVH>(current_mesh);

            glm::mat4x4 transformationMatrix{};
            transformationMatrix[0][0] = transformation.a1; transformationMatrix[0][1] = transformation.b1;  transformationMatrix[0][2] = transformation.c1; transformationMatrix[0][3] = transformation.d1;
            transformationMatrix[1][0] = transformation.a2; transformationMatrix[1][1] = transformation.b2;  transformationMatrix[1][2] = transformation.c2; transformationMatrix[1][3] = transformation.d2;
            transformationMatrix[2][0] = transformation.a3; transformationMatrix[2][1] = transformation.b3;  transformationMatrix[2][2] = transformation.c3; transformationMatrix[2][3] = transformation.d3;
            transformationMatrix[3][0] = transformation.a4; transformationMatrix[3][1] = transformation.b4;  transformationMatrix[3][2] = transformation.c4; transformationMatrix[3][3] = transformation.d4;

            std::shared_ptr<Render::BSRDF> brdf;
            if(m_material_manager.get_material(material_index)->is_light())
            {
                brdf = std::make_shared<Render::Light_BRDF>(m_material_manager, material_index);

                meshBVH->generate_sampling_data();
                m_lights.push_back({ transformationMatrix, glm::inverse(transformationMatrix), meshBVH });
            }
            else
            {
                std::unique_ptr<Render::Distribution> distribution = std::make_unique<Render::Cos_Weighted_Hemisphere_Distribution>();
                brdf = std::make_shared<Render::Diffuse_BRDF>(distribution, m_material_manager, material_index);
            }

            m_bvh.add_lower_level_bvh(meshBVH, transformationMatrix, brdf);
        }

        // Recurse through all child nodes
        for(uint32_t i = 0; i < node->mNumChildren; ++i)
        {
            parse_node(scene,
                       node->mChildren[i],
                       transformation);
        }
    }

    void Scene::add_material(const aiMaterial* material)
    {
        aiString name;
        material->Get(AI_MATKEY_NAME, name);

        PICO_LOG("Adding material %s\n", name.C_Str());

        std::unique_ptr<Core::Material> pico_material;
        if(material->GetTextureCount(aiTextureType_BASE_COLOR) > 0 || material->GetTextureCount(aiTextureType_DIFFUSE) > 1)
        {
            std::unique_ptr<Core::Image2D> albedo;
            std::unique_ptr<Core::Image2D> metalness;
            std::unique_ptr<Core::Image2D> roughness;
            std::unique_ptr<Core::Image2D> combined_metalness_roughness;
            std::unique_ptr<Core::Image2D> emissive;

            if(material->GetTextureCount(aiTextureType_BASE_COLOR) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_BASE_COLOR, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                std::string albedo_path = m_file_mapper->resolve_path(fsPath);
                albedo = std::make_unique<Core::Image2D>(albedo_path);
            }
            else if(material->GetTextureCount(aiTextureType_DIFFUSE) > 1)
            {
                aiString path;
                material->GetTexture(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_BASE_COLOR_TEXTURE, &path);

                std::filesystem::path fsPath(path.C_Str());
                std::string albedo_path = m_file_mapper->resolve_path(fsPath);
                albedo = std::make_unique<Core::Image2D>(albedo_path);

            }

            // AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_METALLICROUGHNESS_TEXTURE maps to aiTextureType_UNKNOWN for gltf becuase why not.
            if(material->GetTextureCount(aiTextureType_UNKNOWN) > 0)
            {
                aiString path;
                material->GetTexture(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_METALLICROUGHNESS_TEXTURE, &path);

                std::filesystem::path fsPath(path.C_Str());
                std::string combined_path = m_file_mapper->resolve_path(fsPath);
                combined_metalness_roughness = std::make_unique<Core::Image2D>(combined_path);
            }
            else
            {

                if(material->GetTextureCount(aiTextureType_DIFFUSE_ROUGHNESS) > 0)
                {
                    aiString path;
                    material->GetTexture(aiTextureType_DIFFUSE_ROUGHNESS, 0, &path);

                    std::filesystem::path fsPath(path.C_Str());
                    std::string roughness_path = m_file_mapper->resolve_path(fsPath);
                    roughness = std::make_unique<Core::Image2D>(roughness_path);
                }

                if(material->GetTextureCount(aiTextureType_METALNESS) > 0)
                {
                    aiString path;
                    material->GetTexture(aiTextureType_METALNESS, 0, &path);

                    std::filesystem::path fsPath(path.C_Str());
                    std::string metalness_path = m_file_mapper->resolve_path(fsPath);
                    metalness = std::make_unique<Core::Image2D>(metalness_path);
                }
            }

            if(material->GetTextureCount(aiTextureType_EMISSIVE) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_EMISSIVE, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                std::string emissive_path = m_file_mapper->resolve_path(fsPath);
                emissive = std::make_unique<Core::Image2D>(emissive_path);
            }
            else if(material->GetTextureCount(aiTextureType_EMISSION_COLOR) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_EMISSIVE, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                std::string emissive_path = m_file_mapper->resolve_path(fsPath);
                emissive = std::make_unique<Core::Image2D>(emissive_path);
            }

            if(combined_metalness_roughness)
                pico_material = std::make_unique<Render::MetalnessRoughnessMaterial>(albedo, combined_metalness_roughness, emissive);
            else
                pico_material = std::make_unique<Render::MetalnessRoughnessMaterial>(albedo, metalness, roughness, emissive);
        }
        else if(material->GetTextureCount(aiTextureType_DIFFUSE) == 1)
        {
            std::unique_ptr<Core::Image2D> diffuse;
            std::unique_ptr<Core::Image2D> specular;
            std::unique_ptr<Core::Image2D> gloss;
            std::unique_ptr<Core::Image2D> emissive;


            if(material->GetTextureCount(aiTextureType_DIFFUSE) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_DIFFUSE, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                std::string diffuse_path = m_file_mapper->resolve_path(fsPath);
                diffuse = std::make_unique<Core::Image2D>(diffuse_path);
            }

            if(material->GetTextureCount(aiTextureType_SPECULAR) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_SPECULAR, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                std::string specular_path = m_file_mapper->resolve_path(fsPath);
                specular = std::make_unique<Core::Image2D>(specular_path);
            }

            if(material->GetTextureCount(aiTextureType_SHININESS) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_SHININESS, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                std::string gloss_path = m_file_mapper->resolve_path(fsPath);
                gloss = std::make_unique<Core::Image2D>(gloss_path);
            }

            if(material->GetTextureCount(aiTextureType_EMISSIVE) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_EMISSIVE, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                std::string emissive_path = m_file_mapper->resolve_path(fsPath);
                emissive = std::make_unique<Core::Image2D>(emissive_path);
            }
            else if(material->GetTextureCount(aiTextureType_EMISSION_COLOR) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_EMISSIVE, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                std::string emissive_path = m_file_mapper->resolve_path(fsPath);
                emissive = std::make_unique<Core::Image2D>(emissive_path);
            }

            pico_material = std::make_unique<Render::SpecularGlossMaterial>(diffuse, specular, gloss, emissive);
        }
        else
        {
            aiColor3D diffuse;
            material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse);

            aiColor3D specular;
            material->Get(AI_MATKEY_COLOR_SPECULAR, specular);

            aiColor3D emissive;
            material->Get(AI_MATKEY_COLOR_EMISSIVE, emissive);

            float roughness = 1.0f;
            material->Get(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_ROUGHNESS_FACTOR, roughness);

            float transparency = 0.0f;
            material->Get(AI_MATKEY_TRANSPARENCYFACTOR, transparency);

            float index_of_refraction = 1.0f;
            material->Get(AI_MATKEY_REFRACTI, index_of_refraction);

            if(transparency > 0.0f)
            {
                pico_material =  std::make_unique<Render::ConstantTransparentDiffuseSpecularMaterial>(  glm::vec3(diffuse.r, diffuse.g, diffuse.b),
                                                                                                        glm::vec3(specular.r, specular.g, specular.b), roughness,
                                                                                                        transparency, index_of_refraction);
            }
            else
                pico_material =  std::make_unique<Render::ConstantDiffuseSpecularMaterial>(  glm::vec3(diffuse.r, diffuse.g, diffuse.b),
                                                                                             glm::vec3(specular.r, specular.g, specular.b), roughness,
                                                                                             glm::vec3(emissive.r, emissive.g, emissive.b));
       }

        m_material_manager.add_material(pico_material);
    }
}
