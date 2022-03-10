#ifndef RAY_TRACING_SCENE_HPP
#define RAY_TRACING_SCENE_HPP

#define NANORT_USE_CPP11_FEATURE 1
#include "ThirdParty/nanort/nanort.h"

#include "Image.hpp"
#include "Core/ThreadPool.hpp"
#include "Core/LowerLevelBVH.hpp"
#include "Core/UpperLevelBVH.hpp"
#include "Core/MaterialManager.hpp"
#include "Render/Sampler.hpp"

#include <memory>
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

        uint32_t* m_Pixels;
        uint32_t* m_SampleCount;
        float*    m_variance;
    };

    class Camera;

    class Scene
    {
    public:

        Scene(ThreadPool&, const std::string& sceneFile);
        ~Scene() = default;

        void render_scene_to_memory(const Camera&, const RenderParams&);
        void render_scene_to_file(const Camera&, RenderParams&, const char*);

    private:

        Core::BVH::UpperLevelBVH m_bvh;
        Core::MaterialManager m_material_manager;

        ThreadPool& m_threadPool;

        std::shared_ptr<Core::ImageCube> mSkybox;
    };

}

#endif
