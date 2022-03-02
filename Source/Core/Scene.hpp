#ifndef RAY_TRACING_SCENE_HPP
#define RAY_TRACING_SCENE_HPP

#define NANORT_USE_CPP11_FEATURE 1
#include "ThirdParty/nanort/nanort.h"

#include "Image.hpp"
#include "Core/Sampler.hpp"
#include "Core/ThreadPool.hpp"

#include <memory>
#include <vector>


namespace Scene
{

    class Camera;

    class Scene
    {
    public:

        Scene();
        ~Scene() = default;

        void renderSceneToMemory(const Camera&, const uint32_t x, const uint32_t y, uint8_t *, ThreadPool&) const;
        void renderSceneToFile(const Camera&, const uint32_t x, const uint32_t y, const char*, ThreadPool&) const;

        struct InterpolatedVertex
        {
            glm::vec4 mPosition;
            glm::vec2 mUV;
            glm::vec3 mNormal;
            glm::vec4 mVertexColour;
            uint32_t  mPrimID;
        };
        InterpolatedVertex interpolateFragment(const uint32_t primID, const float u, const float v) const;

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
        Material calculateMaterial(const InterpolatedVertex&, const MaterialInfo&) const;

        bool isVisibleFrom(const glm::vec3& dst, const glm::vec3& src) const;

        bool traceRayNonAlphaTested(const nanort::Ray<float>& ray, InterpolatedVertex* result) const;

        bool intersectsMesh(const nanort::Ray<float>& ray, uint64_t* instanceID);

        void updateCPUAccelerationStructure(const Scene* scene);

    private:

        bool traceRay(const nanort::Ray<float>& ray, InterpolatedVertex* result) const;

        glm::vec4 traceDiffuseRays(const InterpolatedVertex& frag, const glm::vec4 &origin, const uint32_t sampleCount, const uint32_t depth) const;

        glm::vec4 traceSpecularRays(const InterpolatedVertex& frag, const glm::vec4 &origin, const uint32_t sampleCount, const uint32_t depth) const;

        glm::vec4 shadePoint(const InterpolatedVertex& frag, const glm::vec4 &origin, const uint32_t sampleCount, const uint32_t depth) const;

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
