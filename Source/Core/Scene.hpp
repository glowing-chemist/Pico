#ifndef RAY_TRACING_SCENE_HPP
#define RAY_TRACING_SCENE_HPP

#define NANORT_USE_CPP11_FEATURE 1
#include "ThirdParty/nanort/nanort.h"
#include "json/json.h"
#include "assimp/Importer.hpp"
#include "assimp/scene.h"

#include "Image.hpp"
#include "Core/ThreadPool.hpp"
#include "Core/LowerLevelBVH.hpp"
#include "Core/UpperLevelBVH.hpp"
#include "Core/MaterialManager.hpp"
#include "Camera.hpp"

#include <filesystem>
#include <memory>
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
        uint32_t m_sample;
        uint32_t m_Height;
        uint32_t m_Width;

        glm::vec4* m_Pixels;
        uint32_t*  m_SampleCount;
        float*     m_variance;
    };

    struct Light
    {
        glm::mat4x4 m_transform;
        glm::mat4x4 m_inverse_transform;
        std::shared_ptr<Core::BVH::LowerLevelBVH> m_geometry;
    };

    class Scene
    {
    public:

        Scene(ThreadPool&, const std::filesystem::path& sceneFile);
        Scene(ThreadPool&, const std::filesystem::path& working_dir, const aiScene *scene);
        ~Scene() = default;

        void render_scene_to_memory(const Camera&, const RenderParams&);
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
                              const aiMatrix4x4& parentTransofrmation);
        void add_material(const aiMaterial*);

        std::filesystem::path mWorkingDir;
        std::unordered_map<std::string, Camera>  mCamera;
        std::unordered_map<std::string, uint32_t> mInstanceIDs;
        std::unordered_map<std::string, uint32_t> mMaterials;
        std::vector<std::shared_ptr<Core::BVH::LowerLevelBVH>> m_lowerLevelBVhs;

        Core::BVH::UpperLevelBVH m_bvh;
        Core::MaterialManager m_material_manager;
        std::vector<Light> m_lights;

        ThreadPool& m_threadPool;

        std::shared_ptr<Core::ImageCube> mSkybox;
    };

}

#endif
