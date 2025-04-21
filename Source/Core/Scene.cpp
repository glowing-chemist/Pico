#include "Core/Scene.hpp"
#include "Core/AABB.hpp"
#include "Core/Camera.hpp"
#include "Core/LowerLevelBVH.hpp"
#include "Core/RandUtils.hpp"
#include "Core/Asserts.hpp"
#include "Render/Integrators.hpp"
#include "Render/BasicMaterials.hpp"
#include "Core/LowerLevelImplicitShapesBVH.hpp"
#include "Core/LowerLevelMeshBVH.hpp"
#include "Render/BasicMaterials.hpp"
#include "Util/ToneMappers.hpp"
#include "Util/Denoisers.hpp"
#include "Util/Tiler.hpp"

#include <algorithm>
#include <numeric>
#include <memory>
#include <fstream>
#include <random>

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

        {
            std::unique_ptr<Core::Acceleration_Structures::LowerLevelBVH> sphere = std::make_unique<Core::Acceleration_Structures::LowerLevelSphereBVH>(0.5f);
            mInstanceIDs["Sphere"] = m_lowerLevelBVhs.size();
            m_lowerLevelBVhs.push_back(std::move(sphere));

            std::unique_ptr<Core::Acceleration_Structures::LowerLevelCube> cube = std::make_unique<Core::Acceleration_Structures::LowerLevelCube>();
            mInstanceIDs["Cube"] = m_lowerLevelBVhs.size();
            m_lowerLevelBVhs.push_back(std::move(cube));
        }
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
                std::vector<std::future<void>> loading_task_handles{};

                for(std::string& entityName : sceneRoot[sections[i]].getMemberNames())
                {
                    auto f = [&](void(Scene::* f)(const std::string&, const Json::Value&), Scene* s, const std::string& n, const Json::Value& j){ std::invoke(f, s, n, j); };
                    loading_task_handles.emplace_back(m_threadPool.add_task(f, sectionFunctions[i], this, entityName, sceneRoot[sections[i]][entityName]));
                }

                m_threadPool.wait_for_work_to_finish(loading_task_handles);
            }

        }

        m_bvh.build();
    }

    Scene::Scene(ThreadPool& pool, const std::filesystem::path& working_dir, const aiScene* scene, const Util::Options& options) :
        mWorkingDir(working_dir),
        m_threadPool(pool)
    {
        PICO_ASSERT(scene);

        m_file_mapper = std::make_unique<Core::File_System_Mappings>(working_dir);

        if(options.has_option(Util::Option::kSkybox))
        {
            const std::string sky_box_path = m_file_mapper->resolve_path(options.m_skybox).string();

            int width, height, comp;
            auto* data = stbi_loadf(sky_box_path.c_str(), &width, &height, &comp, 4);

            Core::ImageExtent extent = {static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1};
            mSkybox = std::make_unique<Core::ImageCube>(reinterpret_cast<unsigned char*>(data), extent, Core::Format::kRGBA_F);
        }
        else
        {
            // Create a black cubemap.
            float* white_cube_map = static_cast<float*>(malloc(sizeof(float) * 24));
            for(uint32_t i = 0; i < 24; ++i)
            {
                white_cube_map[i] = 0;
            }
            Core::ImageExtent extent{1, 1, 1};
            mSkybox = std::make_unique<Core::ImageCube>(reinterpret_cast<unsigned char*>(white_cube_map), extent, Core::Format::kRGBA_F);
        }

        m_sky_desc.m_sky_box = mSkybox.get();
        m_sky_desc.m_use_sun = false;
        if(options.has_option(Util::Option::kSunDirection) && options.has_option(Util::Option::kSunColour))
        {
            m_sky_desc.m_use_sun = true;
            m_sky_desc.m_sun_direction = options.m_sun_direction;
            m_sky_desc.m_sun_colour = options.m_sun_colour;
        }

        std::vector<std::future<void>> material_loading_task_handles{};
        for(uint32_t i_mat = 0; i_mat < scene->mNumMaterials; ++i_mat)
        {
            const aiMaterial* mat = scene->mMaterials[i_mat];

            add_material(mat);
        }

        m_threadPool.wait_for_work_to_finish(material_loading_task_handles);

        std::vector<std::future<void>> handles{};
        parse_node(scene, scene->mRootNode, aiMatrix4x4{}, handles);

        m_threadPool.wait_for_work_to_finish(handles);

        m_bvh.build();
    }

    void Scene::render_scene_to_memory(const Camera& camera, const RenderParams& params, const bool* should_quit)
    {
        auto trace_rays_for_tile = [&](const glm::uvec2 start, const glm::uvec2& tile_size, const glm::uvec2& resolution, const uint32_t random_seed, const Camera& camera) -> bool
        {
            Core::Rand::xorshift_random random_generator(random_seed);

            std::vector<uint32_t> pixel_indicies(tile_size.x * tile_size.y);
            for(uint32_t tile_row = 0; tile_row < tile_size.y; ++tile_row)
            {
                auto row_start = std::begin(pixel_indicies) + (tile_row * tile_size.x);
                std::iota(row_start, row_start + tile_size.x, ((start.y + tile_row) * resolution.x) + start.x);
            }
            std::shuffle(std::begin(pixel_indicies), std::end(pixel_indicies), random_generator);

            for(const uint32_t flat_location : pixel_indicies)
            {
                if(*should_quit)
                {
                    return true;
                }

                if(params.m_SampleCount[flat_location] >= params.m_maxSamples)
                {
                    continue;
                }

                const glm::uvec2 pixel_location = glm::uvec2(flat_location % resolution.x, flat_location / resolution.x);

                Render::Monte_Carlo_Integrator integrator(m_bvh, m_material_manager, m_lights, m_sky_desc, random_generator.next());

                for (uint32_t i = 0; i < params.m_maxSamples; ++i)
                {
                    uint32_t prev_sample_count = params.m_SampleCount[flat_location];
                    glm::vec3 pixel_result = integrator.integrate_ray(camera, glm::uvec2(pixel_location.x, pixel_location.y), params.m_maxRayDepth);

                    if (prev_sample_count < 1)
                    {
                        params.m_SampleCount[flat_location] = 1;
                        params.m_variance[flat_location] =  glm::vec4(0, 0, 0, 0);
                    }
                    else
                    {
                        const glm::vec3& previous_pixle = params.m_Pixels[flat_location];
                        const glm::vec3& previous_variance = params.m_variance[flat_location];
                        const glm::vec3 new_pixel_result = previous_pixle + ((pixel_result - previous_pixle) * (1.0f / static_cast<float>(prev_sample_count)));

                        params.m_variance[flat_location] = ((previous_variance * prev_sample_count) + ((pixel_result - previous_pixle) * (pixel_result - new_pixel_result))) / (prev_sample_count + 1);
                        params.m_SampleCount[flat_location] = prev_sample_count + 1;
                        pixel_result = new_pixel_result;
                    }

                    params.m_Pixels[flat_location] = pixel_result;

                    if (prev_sample_count > 4 && glm::all(glm::lessThanEqual(params.m_variance[flat_location], glm::vec3(params.m_maxVariance))))
                        break;
                }
            }
            return false;
        };

        std::random_device random_device{};
        Core::Rand::xorshift_random random_generator(random_device());

        Util::Tiler tiler(m_threadPool, random_generator, glm::uvec2(params.m_Width, params.m_Height), glm::uvec2(64, 64));
        tiler.execute_over_surface(trace_rays_for_tile, camera);

        // Apply denoising and tonemapping
        glm::vec3* tone_mapping_input = params.m_Pixels;
        if(params.m_denoise)
        {
            denoiser_inputs denoising = generate_denoiser_inputs(camera, random_generator, glm::uvec2(params.m_Width, params.m_Height));
            tone_mapping_input = Util::atrous_denoise(  params.m_Pixels,
                                                        denoising.normals,
                                                        denoising.positions,
                                                        denoising.diffuse,
                                                        tiler);

            delete[] denoising.diffuse;
            delete[] denoising.normals;
            delete[] denoising.positions;
        }

        if(params.m_tonemap)
        {
            Util::reinhard_tone_mapping(tone_mapping_input, glm::uvec2(params.m_Width, params.m_Height), tiler);
        }

        if(params.m_denoise)
        {
            std::memcpy(params.m_Pixels, tone_mapping_input, sizeof(glm::vec3) * params.m_Width * params.m_Height);
            delete[] tone_mapping_input;
        }
    }


    void Scene::render_scene_to_file(const Camera& camera, RenderParams& params, const char* path)
    {
        bool shouldQuit = false;
        render_scene_to_memory(camera, params, &shouldQuit);

        // Flip the image right side up.
        std::reverse(params.m_Pixels, params.m_Pixels + (params.m_Width * params.m_Height));
        stbi_write_hdr(path, params.m_Width, params.m_Height, 3, reinterpret_cast<const float*>(params.m_Pixels));
    }

    Scene::denoiser_inputs Scene::generate_denoiser_inputs(const Camera& cam, Core::Rand::xorshift_random random_generator, const glm::uvec2& res) const
    {
        denoiser_inputs results{};
        results.normals = new glm::vec3[res.x * res.y];
        results.positions = new glm::vec3[res.x * res.y];
        results.diffuse = new glm::vec3[res.x * res.y];

        auto trace_rays_for_tile = [this](const glm::uvec2 start, 
                                        const glm::uvec2& tile_size,
                                        const glm::uvec2& res,
                                        const uint32_t, 
                                        const Camera& camera, 
                                        glm::vec3* normal,
                                        glm::vec3* position,
                                        glm::vec3* diffuse) -> bool
        {
            for(uint32_t x = start.x; x < start.x + tile_size.x; ++x)
            {
                for(uint32_t y = start.y; y < start.y + tile_size.y; ++y)
                {
                    const uint32_t flat_location = (y * res.x) + x;
                    const glm::uvec2 pixel_location = glm::uvec2(x, y);

                    Core::Ray ray = camera.generate_ray(glm::vec2(0.0f, 0.0f), pixel_location);
                    Core::Acceleration_Structures::InterpolatedVertex frag{}; 
                    if(m_bvh.get_closest_intersection(ray, &frag))
                    {
                        normal[flat_location] = frag.mNormal;
                        position[flat_location] = camera.getViewMatrix() * frag.mPosition;
                        diffuse[flat_location] = m_material_manager.evaluate_material(frag.m_bsrdf->get_material_id(), frag.mUV).diffuse;
                    }
                    else
                    {
                        normal[flat_location] = -ray.mDirection;
                        position[flat_location] = ray.mDirection * ray.mLenght;
                        diffuse[flat_location] = m_sky_desc.m_sky_box->sample4(ray.mDirection);
                    }
                }
            }

            return true;
        };
        Util::Tiler tiler(m_threadPool, random_generator, res, glm::uvec2(64, 64));
        tiler.execute_over_surface(trace_rays_for_tile, cam, results.normals, results.positions, results.diffuse);

        return results;
    }

    // Scene loading functions.
    void Scene::add_mesh(const std::string& name, const Json::Value& entry)
    {
        const std::filesystem::path path = m_file_mapper->resolve_path(entry["Path"].asString());

        Assimp::Importer importer;

        const aiScene* scene = importer.ReadFile(path.string().c_str(),
                                                 aiProcess_Triangulate |
                                                 aiProcess_JoinIdenticalVertices |
                                                 aiProcess_GenNormals |
                                                 aiProcess_CalcTangentSpace |
                                                 aiProcess_GlobalScale |
                                                 aiProcess_FlipUVs |
                                                 aiProcess_GenBoundingBoxes);


        auto mesh_bvh = std::make_unique<Core::Acceleration_Structures::LowerLevelMeshBVH>(scene->mMeshes[0]);

        std::unique_lock l(m_SceneLoadingMutex);
        mInstanceIDs[name] = m_lowerLevelBVhs.size();
        m_lowerLevelBVhs.push_back(std::move(mesh_bvh));
    }


    void Scene::add_mesh_instance(const std::string&, const Json::Value& entry)
    {
        std::shared_lock l(m_SceneLoadingMutex);
        const std::string assetName = entry["Asset"].asString();
        const uint32_t assetID = mInstanceIDs[assetName];
        l.unlock();

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
            std::shared_lock ml(m_SceneLoadingMutex);
            material = mMaterials[materialName];
        }

        const glm::mat4x4 transform =  glm::translate(glm::mat4x4(1.0f), position) *
                                    glm::mat4_cast(rotation) *
                                    glm::scale(glm::mat4x4(1.0f), scale);


        std::unique_ptr<Render::BSRDF> bsrdf;
        if(entry.isMember("BSRDF"))
        {
            const std::string bsrdf_type = entry["BSRDF"].asString();
            if(bsrdf_type == "Diffuse")
            {
                std::unique_ptr<Render::Distribution> distribution = std::make_unique<Render::Cos_Weighted_Hemisphere_Distribution>();
                bsrdf = std::make_unique<Render::Diffuse_BRDF>(distribution, m_material_manager, material);
            }
            else if(bsrdf_type == "Specular")
            {
                std::unique_ptr<Render::Distribution> distribution = std::make_unique<Render::Beckmann_All_Microfacet_Distribution>();
                bsrdf = std::make_unique<Render::Specular_BRDF>(distribution, m_material_manager, material);
            }
            else if(bsrdf_type == "Dielectric")
            {
                std::unique_ptr<Render::Distribution> diffuse_distribution = std::make_unique<Render::Cos_Weighted_Hemisphere_Distribution>();
                std::unique_ptr<Render::Distribution> specular_distribution = std::make_unique<Render::Beckmann_All_Microfacet_Distribution>();

                bsrdf = std::make_unique<Render::Dielectric_BRDF>(diffuse_distribution, specular_distribution, m_material_manager, material);
            }
            else if(bsrdf_type == "Transparent")
            {
                // Transparent BSRDF access the material manager on creation so needs a lock around it. same for fresnel
                std::shared_lock ml(m_SceneLoadingMutex);
                std::unique_ptr<Render::Distribution> distribution = std::make_unique<Render::Beckmann_All_Microfacet_Distribution>();
                bsrdf = std::make_unique<Render::Transparent_BTDF>(distribution, m_material_manager, material);
            }
            else if(bsrdf_type == "Fresnel")
            {
                std::shared_lock ml(m_SceneLoadingMutex);
                std::unique_ptr<Render::Distribution> specular_distribution = std::make_unique<Render::Beckmann_All_Microfacet_Distribution>();
                std::unique_ptr<Render::Distribution> transmission_distribution = std::make_unique<Render::Beckmann_All_Microfacet_Distribution>();
                bsrdf = std::make_unique<Render::Fresnel_BTDF>(specular_distribution, transmission_distribution, m_material_manager, material);
            }
            else if(bsrdf_type == "Light")
            {
                bsrdf = std::make_unique<Render::Light_BRDF>(m_material_manager, material);

                m_lowerLevelBVhs[assetID]->generate_sampling_data();
                m_lights.push_back({ transform, glm::inverse(transform), m_lowerLevelBVhs[assetID].get() });
            }
            else if(bsrdf_type == "Delta")
            {
                bsrdf = std::make_unique<Render::Specular_Delta_BRDF>(m_material_manager, material);
            }
        }
        PICO_ASSERT(bsrdf);

        std::unique_lock ul(m_SceneLoadingMutex);
        m_bvh.add_lower_level_bvh(m_lowerLevelBVhs[assetID].get(), transform, bsrdf);
    }

    void Scene::add_material(const std::string& name, const Json::Value& entry)
    {
        std::unique_ptr<Core::Material> material;

        if(entry["Type"].asString() == "Metalic")
        {
            std::unique_ptr<Core::Image2D> albedo{};
            if(entry.isMember("Albedo"))
            {
                const std::filesystem::path path = m_file_mapper->resolve_path(entry["Albedo"].asString());
                albedo = std::make_unique<Core::Image2D>(path);
            }

            std::unique_ptr<Core::Image2D> roughness{};
            if(entry.isMember("Roughness"))
            {
                const std::filesystem::path path = m_file_mapper->resolve_path(entry["Roughness"].asString());
                roughness = std::make_unique<Core::Image2D>(path);
            }

            std::unique_ptr<Core::Image2D> metalness{};
            if(entry.isMember("Metalness"))
            {
                const std::filesystem::path path = m_file_mapper->resolve_path(entry["Metalness"].asString());
                metalness = std::make_unique<Core::Image2D>(path);
            }

            std::unique_ptr<Core::Image2D> emmissive{};
            if(entry.isMember("Emissive"))
            {
                const std::filesystem::path path = m_file_mapper->resolve_path(entry["Emissive"].asString());
                emmissive = std::make_unique<Core::Image2D>(path);
            }

            material = std::make_unique<Render::MetalnessRoughnessMaterial>(albedo, metalness, roughness, emmissive);
        }
        else if(entry["Type"].asString() == "Gloss")
        {
            std::unique_ptr<Core::Image2D> diffuse{};
            if(entry.isMember("Diffuse"))
            {
                const std::filesystem::path path = m_file_mapper->resolve_path(entry["Diffuse"].asString());
                diffuse = std::make_unique<Core::Image2D>(path);
            }

            std::unique_ptr<Core::Image2D> specular{};
            if(entry.isMember("Specular"))
            {
                const std::filesystem::path path = m_file_mapper->resolve_path(entry["Specular"].asString());
                specular = std::make_unique<Core::Image2D>(path);
            }

            std::unique_ptr<Core::Image2D> gloss{};
            if(entry.isMember("Gloss"))
            {
                const std::filesystem::path path = m_file_mapper->resolve_path(entry["Gloss"].asString());
                gloss = std::make_unique<Core::Image2D>(path);
            }

            std::unique_ptr<Core::Image2D> emmissive{};
            if(entry.isMember("Emissive"))
            {
                const std::filesystem::path path = m_file_mapper->resolve_path(entry["Emissive"].asString());
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

        std::unique_lock l(m_SceneLoadingMutex);
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

        std::unique_lock l(m_SceneLoadingMutex);
        mCamera.insert({name, newCamera});
    }


    void Scene::process_globals(const std::string&, const Json::Value& entry)
    {
        if(entry.isMember("Skybox"))
        {
            const Json::Value skyboxes = entry["Skybox"];
            if(skyboxes.isArray())
            {
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
                mSkybox = std::make_unique<Core::ImageCube>(data, extent, Core::Format::kRBGA_8UNorm);
            }
            // using a hdri
            else if(skyboxes.isString())
            {
                const std::string sky_box_path = m_file_mapper->resolve_path(skyboxes.asString()).string();

                int width, height, comp;
                auto* data = stbi_loadf(sky_box_path.c_str(), &width, &height, &comp, 4);

                Core::ImageExtent extent = {static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1};
                mSkybox = std::make_unique<Core::ImageCube>(reinterpret_cast<unsigned char*>(data), extent, Core::Format::kRGBA_F);
            }
        }
        else
        {
            // Create a black cubemap.
            unsigned char* black_cube_map = static_cast<unsigned char*>(malloc(sizeof(unsigned char) * 24));
            memset(black_cube_map, 0, 24);
            Core::ImageExtent extent{1, 1, 1};
            mSkybox = std::make_unique<Core::ImageCube>(black_cube_map, extent, Core::Format::kRBGA_8UNorm);
        }

        if(entry.isMember("SunDirection"))
        {
            const Json::Value direction = entry["SunDirection"];
            m_sky_desc.m_sun_direction[0] = direction[0].asFloat();
            m_sky_desc.m_sun_direction[1] = direction[1].asFloat();
            m_sky_desc.m_sun_direction[2] = direction[2].asFloat();

            m_sky_desc.m_sun_direction = glm::normalize(m_sky_desc.m_sun_direction);
        }
        else
        {
            m_sky_desc.m_sun_direction = glm::vec3(0.0f, -1.0f, 0.0f);
        }

        if(entry.isMember("SunColour"))
        {
            const Json::Value colour = entry["SunColour"];
            m_sky_desc.m_sun_colour[0] = colour[0].asFloat();
            m_sky_desc.m_sun_colour[1] = colour[1].asFloat();
            m_sky_desc.m_sun_colour[2] = colour[2].asFloat();
        }
        else
        {
            m_sky_desc.m_sun_colour = glm::vec3(1.0f, 1.0f, 1.0f);
        }

        m_sky_desc.m_sky_box = mSkybox.get();
        m_sky_desc.m_use_sun = entry.isMember("SunDirection") && entry.isMember("SunColour");
    }

    void Scene::parse_node(const aiScene* scene,
                          const aiNode* node,
                          const aiMatrix4x4& parentTransofrmation,
                          std::vector<std::future<void>>& tasks)
    {
        aiMatrix4x4 transformation = parentTransofrmation * node->mTransformation;

        auto add_mesh = [this](const aiMesh* mesh, uint32_t material_index, aiMatrix4x4 transformation)
        {
            std::unique_ptr<Core::Acceleration_Structures::LowerLevelBVH> meshBVH = std::make_unique<Core::Acceleration_Structures::LowerLevelMeshBVH>(mesh);

            glm::mat4x4 transformationMatrix{};
            transformationMatrix[0][0] = transformation.a1; transformationMatrix[0][1] = transformation.b1;  transformationMatrix[0][2] = transformation.c1; transformationMatrix[0][3] = transformation.d1;
            transformationMatrix[1][0] = transformation.a2; transformationMatrix[1][1] = transformation.b2;  transformationMatrix[1][2] = transformation.c2; transformationMatrix[1][3] = transformation.d2;
            transformationMatrix[2][0] = transformation.a3; transformationMatrix[2][1] = transformation.b3;  transformationMatrix[2][2] = transformation.c3; transformationMatrix[2][3] = transformation.d3;
            transformationMatrix[3][0] = transformation.a4; transformationMatrix[3][1] = transformation.b4;  transformationMatrix[3][2] = transformation.c4; transformationMatrix[3][3] = transformation.d4;

        bool is_light;
        {
            std::shared_lock l(this->m_SceneLoadingMutex);
            is_light = this->m_material_manager.get_material(material_index)->is_light();
        }

            std::unique_ptr<Render::BSRDF> brdf;
            if(is_light)
            {
                brdf = std::make_unique<Render::Light_BRDF>(this->m_material_manager, material_index);

                meshBVH->generate_sampling_data();
                std::unique_lock l(this->m_SceneLoadingMutex);
                m_lights.push_back({ transformationMatrix, glm::inverse(transformationMatrix), meshBVH.get() });
            }
            else
            {
                std::unique_ptr<Render::Distribution> diffuse_distribution = std::make_unique<Render::Cos_Weighted_Hemisphere_Distribution>();
                std::unique_ptr<Render::Distribution> specular_distribution = std::make_unique<Render::Beckmann_All_Microfacet_Distribution>();

                brdf = std::make_unique<Render::Dielectric_BRDF>(diffuse_distribution, specular_distribution, this->m_material_manager, material_index);
            }

            std::unique_lock l(this->m_SceneLoadingMutex);
            m_bvh.add_lower_level_bvh(meshBVH.get(), transformationMatrix, brdf);
            m_lowerLevelBVhs.push_back(std::move(meshBVH));
        };

        {
            std::shared_lock l(this->m_SceneLoadingMutex);
            for(uint32_t i = 0; i < node->mNumMeshes; ++i)
            {
                tasks.push_back(m_threadPool.add_task(add_mesh, scene->mMeshes[node->mMeshes[i]], scene->mMeshes[node->mMeshes[i]]->mMaterialIndex, transformation));
            }
        }

        // Recurse through all child nodes
        for(uint32_t i = 0; i < node->mNumChildren; ++i)
        {
            parse_node(scene,
                       node->mChildren[i],
                       transformation,
                       tasks);
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
                const std::filesystem::path albedo_path = m_file_mapper->resolve_path(fsPath);
                albedo = std::make_unique<Core::Image2D>(albedo_path);
            }
            else if(material->GetTextureCount(aiTextureType_DIFFUSE) > 1)
            {
                aiString path;
                material->GetTexture(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_BASE_COLOR_TEXTURE, &path);

                std::filesystem::path fsPath(path.C_Str());
                const std::filesystem::path albedo_path = m_file_mapper->resolve_path(fsPath);
                albedo = std::make_unique<Core::Image2D>(albedo_path);

            }

            // AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_METALLICROUGHNESS_TEXTURE maps to aiTextureType_UNKNOWN for gltf becuase why not.
            if(material->GetTextureCount(aiTextureType_UNKNOWN) > 0)
            {
                aiString path;
                material->GetTexture(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_METALLICROUGHNESS_TEXTURE, &path);

                std::filesystem::path fsPath(path.C_Str());
                const std::filesystem::path combined_path = m_file_mapper->resolve_path(fsPath);
                combined_metalness_roughness = std::make_unique<Core::Image2D>(combined_path);
            }
            else
            {

                if(material->GetTextureCount(aiTextureType_DIFFUSE_ROUGHNESS) > 0)
                {
                    aiString path;
                    material->GetTexture(aiTextureType_DIFFUSE_ROUGHNESS, 0, &path);

                    std::filesystem::path fsPath(path.C_Str());
                    const std::filesystem::path roughness_path = m_file_mapper->resolve_path(fsPath);
                    roughness = std::make_unique<Core::Image2D>(roughness_path);
                }

                if(material->GetTextureCount(aiTextureType_METALNESS) > 0)
                {
                    aiString path;
                    material->GetTexture(aiTextureType_METALNESS, 0, &path);

                    std::filesystem::path fsPath(path.C_Str());
                    const std::filesystem::path metalness_path = m_file_mapper->resolve_path(fsPath);
                    metalness = std::make_unique<Core::Image2D>(metalness_path);
                }
            }

            if(material->GetTextureCount(aiTextureType_EMISSIVE) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_EMISSIVE, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                const std::filesystem::path emissive_path = m_file_mapper->resolve_path(fsPath);
                emissive = std::make_unique<Core::Image2D>(emissive_path);
            }
            else if(material->GetTextureCount(aiTextureType_EMISSION_COLOR) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_EMISSIVE, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                const std::filesystem::path emissive_path = m_file_mapper->resolve_path(fsPath);
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
                const std::filesystem::path diffuse_path = m_file_mapper->resolve_path(fsPath);
                diffuse = std::make_unique<Core::Image2D>(diffuse_path);
            }

            if(material->GetTextureCount(aiTextureType_SPECULAR) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_SPECULAR, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                const std::filesystem::path specular_path = m_file_mapper->resolve_path(fsPath);
                specular = std::make_unique<Core::Image2D>(specular_path);
            }

            if(material->GetTextureCount(aiTextureType_SHININESS) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_SHININESS, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                const std::filesystem::path gloss_path = m_file_mapper->resolve_path(fsPath);
                gloss = std::make_unique<Core::Image2D>(gloss_path);
            }

            if(material->GetTextureCount(aiTextureType_EMISSIVE) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_EMISSIVE, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                const std::filesystem::path emissive_path = m_file_mapper->resolve_path(fsPath);
                emissive = std::make_unique<Core::Image2D>(emissive_path);
            }
            else if(material->GetTextureCount(aiTextureType_EMISSION_COLOR) > 0)
            {
                aiString path;
                material->GetTexture(aiTextureType_EMISSIVE, 0, &path);

                std::filesystem::path fsPath(path.C_Str());
                const std::filesystem::path emissive_path = m_file_mapper->resolve_path(fsPath);
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
