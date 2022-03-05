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

        struct MaterialInfo
        {
            uint64_t instanceID;
            uint32_t materialIndex;
            uint32_t materialFlags;
        };

        struct Material
        {
            glm::vec4 diffuse;
            glm::vec4 specularRoughness; // xyz specular w roughness
            glm::vec3 normal;
            glm::vec4 emissiveOcclusion; // xyz emisive w ambient occlusion.
        };
        Material calculateMaterial(const Core::BVH::InterpolatedVertex&, const MaterialInfo&) const;

        bool isVisibleFrom(const glm::vec3& dst, const glm::vec3& src) const;

        bool traceRayNonAlphaTested(const nanort::Ray<float>& ray, Core::BVH::InterpolatedVertex* result) const;

        bool intersectsMesh(const nanort::Ray<float>& ray, uint64_t* instanceID);

        void updateCPUAccelerationStructure(const Scene* scene);

    private:

        glm::vec4 traceDiffuseRays(const Core::BVH::InterpolatedVertex& frag, const glm::vec4 &origin, const uint32_t sampleCount, const uint32_t depth) const;

        glm::vec4 traceSpecularRays(const Core::BVH::InterpolatedVertex& frag, const glm::vec4 &origin, const uint32_t sampleCount, const uint32_t depth) const;

        glm::vec4 shadePoint(const Core::BVH::InterpolatedVertex& frag, const glm::vec4 &origin, const uint32_t sampleCount, const uint32_t depth) const;

        ThreadPool& m_threadPool;

        std::vector<glm::vec3> mPositions;
        std::vector<glm::vec2> mUVs;
        std::vector<glm::vec4> mNormals;
        std::vector<glm::vec4> mVertexColours;
        std::vector<uint32_t>  mIndexBuffer;

        std::unique_ptr<nanort::TriangleMesh<float>> mMeshes;
        std::unique_ptr<nanort::TriangleSAHPred<float>> mPred;

        nanort::BVHAccel<float> mAccelerationStructure;

        std::vector<MaterialInfo> mPrimitiveMaterialID; // maps prim ID to material ID.

        std::unique_ptr<Core::Image> mSkybox;
        std::vector<Core::Image> mMaterials;
    };

}

#endif
