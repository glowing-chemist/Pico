#ifndef RAY_TRACING_SCENE_HPP
#define RAY_TRACING_SCENE_HPP

#define NANORT_USE_CPP11_FEATURE 1
#include "ThirdParty/nanort/nanort.h"

#include "Image.hpp"
#include "Core/ThreadPool.hpp"
#include "Core/LowerLevelBVH.hpp"
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

        void renderSceneToMemory(const Camera&, const RenderParams&) const;
        void renderSceneToFile(const Camera&, RenderParams&, const char*) const;

    private:

        glm::vec4 traceDiffuseRays(const Core::BVH::InterpolatedVertex& frag, const glm::vec4 &origin, const uint32_t sampleCount, const uint32_t depth) const;

        glm::vec4 traceSpecularRays(const Core::BVH::InterpolatedVertex& frag, const glm::vec4 &origin, const uint32_t sampleCount, const uint32_t depth) const;

        glm::vec4 shadePoint(const Core::BVH::InterpolatedVertex& frag, const glm::vec4 &origin, const uint32_t sampleCount, const uint32_t depth) const;

        ThreadPool& m_threadPool;

        std::unique_ptr<Core::ImageCube> mSkybox;
    };

}

#endif
