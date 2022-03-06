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
    {
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



    glm::vec4 Scene::shadePoint(const InterpolatedVertex& frag, const glm::vec4 &origin, const uint32_t sampleCount, const uint32_t depth) const
    {
        if(depth == 0)
            return glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

        const glm::vec4 diffuse = traceDiffuseRays(frag, origin, sampleCount, depth);
        const glm::vec4 specular = traceSpecularRays(frag, origin, sampleCount, depth);

        return diffuse + specular;// * (shadowed ? 0.15f : 1.0f);
    }

}
