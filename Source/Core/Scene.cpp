#include "Core/Scene.hpp"
#include "Core/Camera.hpp"
#include "Core/vectorUtils.hpp"
#include "Render/Sampler.hpp"
#include "Render/PBR.hpp"

#include <algorithm>
#include <thread>

#include "stbi_image_write.h"
#include "glm/ext.hpp"
#include "glm/gtx/compatibility.hpp"

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
        m_threadPool(pool),
        mPrimitiveMaterialID{}
    {
        //updateCPUAccelerationStructure(scene);

        //nanort::BVHBuildStatistics stats = mAccelerationStructure.GetStatistics();
    }

    void Scene::renderSceneToMemory(const Camera& camera, const RenderParams& params) const
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

            nanort::Ray<float> ray;
            ray.dir[0] = dir.x;
            ray.dir[1] = dir.y;
            ray.dir[2] = dir.z;

            ray.org[0] = origin.x;
            ray.org[1] = origin.y;
            ray.org[2] = origin.z;

            ray.min_t = 0.0f;
            ray.max_t = farPlane;
            //ray.type = nanort::RAY_TYPE_PRIMARY;

            InterpolatedVertex frag;
            const bool hit = traceRay(ray, &frag);
            if(hit)
            {
                return shadePoint(frag, glm::vec4(origin, 1.0f), params.m_maxSamples, params.m_maxRayDepth);
            }
            else
            {
                return mSkybox->sample4(dir);
            }
        };

        auto trace_rays = [&](const uint32_t start, const uint32_t stepSize)
        {
            uint32_t location = start;
            while(location < (params.m_Height * params.m_Width))
            {
                const uint32_t pix = location % params.m_Height;
                const uint32_t piy = location / params.m_Height;
                const uint32_t colour = Core::pack_colour(trace_ray(pix, piy));
                memcpy(&params.m_Pixels[(pix + (piy * params.m_Height)) * 4], &colour, sizeof(uint32_t));

                location += stepSize;
            }
        };

        const uint32_t processor_count = m_threadPool.getWorkerCount(); // use this many threads for tracing rays.
        std::vector<std::future<void>> handles{};
        for(uint32_t i = 1; i < processor_count; ++i)
        {
            handles.push_back(m_threadPool.addTask(trace_rays, i, processor_count));
        }

        trace_rays(0, processor_count);

        for(auto& thread : handles)
            thread.wait();
    }


    void Scene::renderSceneToFile(const Camera& camera, RenderParams& params, const char* path) const
    {
        renderSceneToMemory(camera, params);

        stbi_write_jpg(path, params.m_Width, params.m_Height, 4, params.m_Pixels, 100);
    }


    bool Scene::traceRay(const nanort::Ray<float>& ray, InterpolatedVertex *result) const
    {
        nanort::TriangleIntersector triangle_intersecter(reinterpret_cast<const float*>(mPositions.data()), mIndexBuffer.data(), sizeof(glm::vec3));
        nanort::TriangleIntersection intersection;
        bool hit = mAccelerationStructure.Traverse(ray, triangle_intersecter, &intersection);

        if(hit) // check for alpha tested geometry.
        {
            *result= interpolateFragment(intersection.prim_id, intersection.u, intersection.v);

            const MaterialInfo& matInfo = mPrimitiveMaterialID[result->mPrimID];

            if(matInfo.materialFlags & MaterialType::Albedo || matInfo.materialFlags & MaterialType::Diffuse)
            {

                const Core::Image& diffuseTexture = mMaterials[matInfo.materialIndex];
                const glm::vec4 colour = diffuseTexture.sample4(result->mUV);

                if(colour.a == 0.0f) // trace another ray.
                {
                    nanort::Ray<float> newRay{};
                    newRay.org[0] = result->mPosition.x;
                    newRay.org[1] = result->mPosition.y;
                    newRay.org[2] = result->mPosition.z;
                    newRay.dir[0] = ray.dir[0];
                    newRay.dir[1] = ray.dir[1];
                    newRay.dir[2] = ray.dir[2];
                    newRay.min_t = 0.01f;
                    newRay.max_t = 2000.0f;

                    hit = traceRay(newRay, result);
                }
            }
        }

        return hit;
    }


    bool Scene::traceRayNonAlphaTested(const nanort::Ray<float> &ray, InterpolatedVertex *result) const
    {
        nanort::TriangleIntersector triangle_intersecter(reinterpret_cast<const float*>(mPositions.data()), mIndexBuffer.data(), sizeof(glm::vec3));
        nanort::TriangleIntersection intersection;
        const bool hit = mAccelerationStructure.Traverse(ray, triangle_intersecter, &intersection);

        if(hit)
        {
            *result = interpolateFragment(intersection.prim_id, intersection.u, intersection.v);
        }

        return hit;
    }


    bool Scene::intersectsMesh(const nanort::Ray<float>& ray, uint64_t *instanceID)
    {
        InterpolatedVertex vertex;
        if(traceRay(ray, &vertex))
        {
            const MaterialInfo& matInfo = mPrimitiveMaterialID[vertex.mPrimID];
            *instanceID = matInfo.instanceID;

            return true;
        }

        return false;
    }


    glm::vec4 Scene::traceDiffuseRays(const InterpolatedVertex& frag, const glm::vec4& origin, const uint32_t sampleCount, const uint32_t depth) const
    {
        // interpolate uvs.
        const MaterialInfo& matInfo = mPrimitiveMaterialID[frag.mPrimID];
        Material mat = calculateMaterial(frag, matInfo);
        glm::vec4 diffuse = mat.diffuse;
        const glm::vec3 V = glm::normalize(glm::vec3(origin - frag.mPosition));

        Render::DiffuseSampler sampler(sampleCount * depth);

        glm::vec4 result = glm::vec4{0.0f, 0.0f, 0.0f, 0.0f};
        float weight = 0.0f;
        for(uint32_t i = 0; i < sampleCount; ++i)
        {
            Render::Sample sample = sampler.generateSample(frag.mNormal);
            weight += sample.P;

            const float NdotV = glm::clamp(glm::dot(glm::vec3(mat.normal), V), 0.0f, 1.0f);
            const float NdotL = glm::clamp(glm::dot(glm::vec3(mat.normal), sample.L), 0.0f, 1.0f);
            const glm::vec3 H = glm::normalize(V + sample.L);
            const float LdotH  = glm::clamp(glm::dot(sample.L, H), 0.0f, 1.0f);

            const float diffuseFactor = Render::disney_diffuse(NdotV, NdotL, LdotH, mat.specularRoughness.w);

            nanort::Ray<float> newRay{};
            newRay.org[0] = frag.mPosition.x;
            newRay.org[1] = frag.mPosition.y;
            newRay.org[2] = frag.mPosition.z;
            newRay.dir[0] = sample.L.x;
            newRay.dir[1] = sample.L.y;
            newRay.dir[2] = sample.L.z;
            newRay.min_t = 0.01f;
            newRay.max_t = 2000.0f;

            InterpolatedVertex intersection;
            const bool hit = traceRay(newRay, &intersection);
            if(hit)
            {
                result += sample.P * diffuseFactor * shadePoint(intersection, frag.mPosition, sampleCount, depth - 1);
            }
            else
            {
                result += sample.P * diffuseFactor * mSkybox->sample4(sample.L); // miss so sample skybox.
            }
        }

        diffuse *= result / weight;

        return diffuse;// + glm::vec4(mat.emissiveOcclusion.x, mat.emissiveOcclusion.y, mat.emissiveOcclusion.z, 1.0f);
    }


    glm::vec4 Scene::traceSpecularRays(const InterpolatedVertex& frag, const glm::vec4 &origin, const uint32_t sampleCount, const uint32_t depth) const
    {
        // interpolate uvs.
        const MaterialInfo& matInfo = mPrimitiveMaterialID[frag.mPrimID];
        Material mat = calculateMaterial(frag, matInfo);
        glm::vec4 specular = glm::vec4(mat.specularRoughness.x, mat.specularRoughness.y, mat.specularRoughness.z, 1.0f);

        Render::SpecularSampler sampler(sampleCount * depth);

        const glm::vec3 V = glm::normalize(glm::vec3(origin - frag.mPosition));

        glm::vec4 result = glm::vec4{0.0f, 0.0f, 0.0f, 0.0f};
        float weight = 0.0f;
        for(uint32_t i = 0; i < sampleCount; ++i)
        {
            Render::Sample sample = sampler.generateSample(frag.mNormal, V, mat.specularRoughness.w);
            weight += sample.P;

            nanort::Ray<float> newRay{};
            newRay.org[0] = frag.mPosition.x;
            newRay.org[1] = frag.mPosition.y;
            newRay.org[2] = frag.mPosition.z;
            newRay.dir[0] = sample.L.x;
            newRay.dir[1] = sample.L.y;
            newRay.dir[2] = sample.L.z;
            newRay.min_t = 0.01f;
            newRay.max_t = 2000.0f;

            const float specularFactor = Render::specular_GGX(mat.normal, V, sample.L, mat.specularRoughness.w, glm::vec3(mat.specularRoughness.x, mat.specularRoughness.y, mat.specularRoughness.z));

            InterpolatedVertex intersection;
            const bool hit = traceRay(newRay, &intersection);
            if(hit)
            {
                result += specularFactor * sample.P * shadePoint(intersection, frag.mPosition, sampleCount, depth - 1);
            }
            else
            {
                result += specularFactor * sample.P * mSkybox->sample4(sample.L); // miss so sample skybox.
            }
        }

        specular *= result / weight;

        return glm::all(glm::isnan(specular)) ? glm::vec4(0.0f, 0.0f, 0.0f, 1.0f) : specular;// + glm::vec4(mat.emissiveOcclusion.x, mat.emissiveOcclusion.y, mat.emissiveOcclusion.z, 1.0f);
    }


    Scene::Material Scene::calculateMaterial(const InterpolatedVertex& frag, const MaterialInfo& info) const
    {
        Material mat;
        mat.diffuse = frag.mVertexColour;
        mat.normal = glm::normalize(frag.mNormal);
        mat.specularRoughness = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
        mat.emissiveOcclusion = glm::vec4(0.0f, 0.0f, 0.0f, 1.0);

        auto fresnelSchlickRoughness = [](const float cosTheta, const glm::vec3 F0, const float roughness)
        {
            return F0 + (glm::max(glm::vec3(1.0f - roughness), F0) - F0) * std::pow(1.0f - cosTheta, 5.0f);
        };

        uint32_t nextMaterialSlot = 0;

        if(info.materialFlags & MaterialType::Diffuse)
        {
            mat.diffuse = mMaterials[info.materialIndex].sample4(frag.mUV);
            ++nextMaterialSlot;
        }
        else if(info.materialFlags & MaterialType::Albedo)
            ++nextMaterialSlot;

        if(info.materialFlags & MaterialType::Normals)
        {
            ++nextMaterialSlot; // TODO work out how to do this without derivitives!!.
        }

        if(info.materialFlags & MaterialType::Roughness)
        {
            mat.specularRoughness.w = mMaterials[info.materialIndex + nextMaterialSlot].sample(frag.mUV);
            ++nextMaterialSlot;
        }
        else if(info.materialFlags & MaterialType::Gloss)
        {
            mat.specularRoughness.w = 1.0f - mMaterials[info.materialIndex + nextMaterialSlot].sample(frag.mUV);
            ++nextMaterialSlot;
        }

        float metalness = 0.0f;
        if(info.materialFlags & MaterialType::Specular)
        {
            const glm::vec3 spec = mMaterials[info.materialIndex + nextMaterialSlot].sample4(frag.mUV);
            mat.specularRoughness.x = spec.x;
            mat.specularRoughness.y = spec.y;
            mat.specularRoughness.z = spec.z;
            ++nextMaterialSlot;
        }
        else if(info.materialFlags & MaterialType::CombinedSpecularGloss)
        {
            mat.specularRoughness = mMaterials[info.materialIndex + nextMaterialSlot].sample4(frag.mUV);
            mat.specularRoughness.w = 1.0f - mat.specularRoughness.w;
            ++nextMaterialSlot;
        }
        else if(info.materialFlags & MaterialType::Metalness)
        {
            metalness = mMaterials[info.materialIndex + nextMaterialSlot].sample(frag.mUV);
            ++nextMaterialSlot;
        }
        else if(info.materialFlags & MaterialType::CombinedMetalnessRoughness)
        {
            const glm::vec4 metalnessRoughness = mMaterials[info.materialIndex + nextMaterialSlot].sample4(frag.mUV);
            metalness = metalnessRoughness.z;
            mat.specularRoughness.w = metalnessRoughness.y;
            ++nextMaterialSlot;
        }

        if(info.materialFlags & MaterialType::Albedo)
        {
            const glm::vec4 albedo = mMaterials[info.materialIndex].sample4(frag.mUV);
            mat.diffuse = albedo * (1.0f - 0.04f) * (1.0f - metalness);
            mat.diffuse.w = albedo.w;// Preserve the alpha chanle.

            const glm::vec3 F0 = glm::lerp(glm::vec3(0.04f, 0.04f, 0.04f), glm::vec3(albedo), metalness);
            mat.specularRoughness.x = F0.x;
            mat.specularRoughness.y = F0.y;
            mat.specularRoughness.z = F0.z;
        }

        if(info.materialFlags & MaterialType::Emisive)
        {
            const glm::vec3 emissive = mMaterials[info.materialIndex + nextMaterialSlot].sample4(frag.mUV);
            mat.emissiveOcclusion.x = emissive.x;
            mat.emissiveOcclusion.y = emissive.y;
            mat.emissiveOcclusion.z = emissive.z;
            ++nextMaterialSlot;
        }

        if(info.materialFlags & MaterialType::AmbientOcclusion)
        {
            mat.emissiveOcclusion.w = mMaterials[info.materialIndex + nextMaterialSlot].sample(frag.mUV);
        }

        return mat;
    }


    bool Scene::isVisibleFrom(const glm::vec3& dst, const glm::vec3& src) const
    {
        const glm::vec3 direction = dst - src;
        const glm::vec3 normalizedDir = glm::normalize(direction);

        nanort::Ray ray{};
        ray.dir[0] = normalizedDir.x;
        ray.dir[1] = normalizedDir.y;
        ray.dir[2] = normalizedDir.z;
        ray.org[0] = src.x;
        ray.org[1] = src.y;
        ray.org[2] = src.z;
        ray.min_t = 0.001f;
        ray.max_t = 200.0f;

        InterpolatedVertex frag;
        const bool hit = traceRay(ray, &frag);
        if(hit)
        {
            const float dist = glm::length(direction);
            const float distToHit = glm::length(glm::vec3(frag.mPosition) - src);

            return distToHit >= dist;
        }

        return true;
    }


    glm::vec4 Scene::shadePoint(const InterpolatedVertex& frag, const glm::vec4 &origin, const uint32_t sampleCount, const uint32_t depth) const
    {
        if(depth == 0)
            return glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

        const glm::vec4 diffuse = traceDiffuseRays(frag, origin, sampleCount, depth);
        const glm::vec4 specular = traceSpecularRays(frag, origin, sampleCount, depth);

        return diffuse + specular;// * (shadowed ? 0.15f : 1.0f);
    }


    void Scene::updateCPUAccelerationStructure(const Scene* scene)
    {

    }

}
