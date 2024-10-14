#ifndef RAY_TRACING_SCENE_HPP
#define RAY_TRACING_SCENE_HPP

#include "json/json.h"
#include "assimp/Importer.hpp"
#include "assimp/scene.h"

#include "Image.hpp"
#include "Core/ThreadPool.hpp"
#include "Core/LowerLevelBVH.hpp"
#include "Core/UpperLevelBVH.hpp"
#include "Core/MaterialManager.hpp"
#include "Core/FileMappings.hpp"
#include "Util/Options.hpp"
#include "Camera.hpp"

#include <filesystem>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>


namespace Scene
{

    struct RenderParams
    {
        uint32_t m_maxRayDepth;
        uint32_t m_maxSamples;
        float    m_maxVariance;
        uint32_t m_Height;
        uint32_t m_Width;

        glm::vec3* m_Pixels;
        uint32_t*  m_SampleCount;
        glm::vec3* m_variance;
    };

    struct Light
    {
        glm::mat4x4 m_transform;
        glm::mat4x4 m_inverse_transform;
        Core::Acceleration_Structures::LowerLevelBVH* m_geometry;
    };

    struct Sun
    {
        Core::ImageCube* m_sky_box;
        bool m_use_sun;
        glm::vec3 m_sun_direction;
        glm::vec3 m_sun_colour;
    };

    class Scene
    {
    public:

        Scene(ThreadPool&, const std::filesystem::path& sceneFile);
        Scene(ThreadPool&, const std::filesystem::path& working_dir, const aiScene *scene, const Util::Options& options);
        ~Scene() = default;

        void render_scene_to_memory(const Camera&, const RenderParams&, const bool* should_quit);
        void render_scene_to_file(const Camera&, RenderParams&, const char*);

        Camera* get_camera(const std::string& name)
        {
            if(auto camera = mCamera.find(name); camera != mCamera.end())
            {
                return &camera->second;
            }

            return nullptr;
        }

    private:

        // Scene loading functions
        void add_mesh(const std::string& name, const Json::Value& entry);
        void add_mesh_instance(const std::string& name, const Json::Value& entry);
        void add_material(const std::string& name, const Json::Value& entry);
        void add_camera(const std::string& name, const Json::Value& entry);
        void process_globals(const std::string& name, const Json::Value& entry);

        void parse_node(const aiScene* scene,
                              const aiNode* node,
                              const aiMatrix4x4& parentTransofrmation,
                              std::vector<std::future<void>>& tasks);
        void add_material(const aiMaterial*);

        std::filesystem::path mWorkingDir;
        std::shared_mutex m_SceneLoadingMutex;
        std::unordered_map<std::string, Camera>  mCamera;
        std::unordered_map<std::string, uint32_t> mInstanceIDs;
        std::unordered_map<std::string, uint32_t> mMaterials;
        std::vector<std::unique_ptr<Core::Acceleration_Structures::LowerLevelBVH>> m_lowerLevelBVhs;

        Core::Acceleration_Structures::UpperLevelBVH m_bvh;
        Core::MaterialManager m_material_manager;
        std::vector<Light> m_lights;

        ThreadPool& m_threadPool;

        std::unique_ptr<Core::File_System_Mappings> m_file_mapper;

        std::unique_ptr<Core::ImageCube> mSkybox;

        Sun m_sky_desc;
    };

}

#endif
