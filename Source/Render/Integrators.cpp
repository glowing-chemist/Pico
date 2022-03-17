#include "Integrators.hpp"
#include "Render/PBR.hpp"
#include "Core/Image.hpp"

namespace Render
{

    Integrator::Integrator(const Core::BVH::UpperLevelBVH& bvh, Core::MaterialManager& material_manager) :
        m_bvh{bvh},
        m_material_manager{material_manager}
    {
    }


    Monte_Carlo_Integrator::Monte_Carlo_Integrator(const Core::BVH::UpperLevelBVH& bvh,  Core::MaterialManager& material_manager, std::shared_ptr<Core::ImageCube>& skybox,
                           std::unique_ptr<Diffuse_Sampler>& diffuseSampler, std::unique_ptr<Specular_Sampler>& specSampler, const uint64_t seed) :
        Integrator(bvh, material_manager),
        mGenerator{seed},
        mDistribution(0.0f, 1.0f),
        m_diffuse_sampler(std::move(diffuseSampler)),
        m_specular_sampler(std::move(specSampler)),
        m_skybox{skybox}
    {
    }


    glm::vec4 Monte_Carlo_Integrator::integrate_ray(const Core::Ray& ray, const uint32_t maxDepth, const uint32_t rayCount)
    {
        Core::BVH::InterpolatedVertex vertex;
        if(m_bvh.get_closest_intersection(ray, &vertex))
        {
            glm::vec4 diffuse{0.0f, 0.0f, 0.0f, 0.0f};
            glm::vec4 specular{0.0f, 0.0f, 0.0f, 0.0f};
            for(uint32_t i_ray = 0; i_ray < rayCount; ++i_ray)
            {
                Core::Ray diffuse_ray = ray;
                diffuse_ray.m_payload = glm::vec4(1.0f);
                diffuse_ray.m_weight = 0.0f;
                trace_diffuse_ray(vertex, diffuse_ray, maxDepth);

                Core::Ray specular_ray = ray;
                specular_ray.m_payload = glm::vec4(1.0f);
                specular_ray.m_weight = 0.0f;
                trace_specular_ray(vertex, specular_ray, maxDepth);

                diffuse += diffuse_ray.m_payload / diffuse_ray.m_weight;
                specular += specular_ray.m_payload / specular_ray.m_weight;
            }

            diffuse /= rayCount;
            specular /= rayCount;

            if(glm::any(glm::isinf(specular)) || glm::any(glm::isnan(specular)))
                specular = glm::vec4(0.0f);

            if(glm::any(glm::isinf(diffuse)) || glm::any(glm::isnan(diffuse)))
                diffuse = glm::vec4(0.0f);


            return diffuse + specular;
        }
        else
        {
            return m_skybox->sample4(ray.mDirection);
        }
}


    void Monte_Carlo_Integrator::trace_diffuse_ray(const Core::BVH::InterpolatedVertex& frag, Core::Ray& ray, const uint32_t depth)
    {
        if(depth == 0)
        {
            ray.m_payload = glm::vec4(0.0f, 0.f, 0.0f, 1.0f);
            ray.m_weight = 1.0f;
            return;
        }

        const auto material = m_material_manager.evaluate_material(frag.mMaterialID, frag.mUV);

        if(material.emits_light())
        {
            ray.m_payload *= glm::vec4(material.emissive, 1.0f);
            ray.m_weight = ray.m_weight == 0.0f ? 1.0f : ray.m_weight;
            return;
        }

        glm::vec4 diffuse = material.diffuse;
        const glm::vec3 V = glm::normalize(glm::vec3(ray.mOrigin - frag.mPosition));

        Render::Sample sample = m_diffuse_sampler->generate_sample(frag.mNormal);
        ray.m_weight += sample.P;

        const float NdotV = glm::clamp(glm::dot(glm::vec3(frag.mNormal), V), 0.0f, 1.0f);
        const float NdotL = glm::clamp(glm::dot(glm::vec3(frag.mNormal), sample.L), 0.0f, 1.0f);
        const glm::vec3 H = glm::normalize(V + sample.L);
        const float LdotH  = glm::clamp(glm::dot(sample.L, H), 0.0f, 1.0f);

        const float diffuse_factor = Render::disney_diffuse(NdotV, NdotL, LdotH, material.specularRoughness.w);

        ray.mOrigin = frag.mPosition + glm::vec4(0.01f * sample.L, 0.0f);
        ray.mDirection = sample.L;

        Core::BVH::InterpolatedVertex intersection;
        if(m_bvh.get_closest_intersection(ray, &intersection))
        {
            ray.m_payload *= diffuse * diffuse_factor * sample.P;

            if(weighted_random_ray_type(material))
                trace_specular_ray(intersection, ray, depth - 1);
            else
                trace_diffuse_ray(intersection, ray, depth - 1);
        }
        else
        {
            ray.m_payload *= diffuse * diffuse_factor * sample.P * m_skybox->sample4(sample.L);
        }
    }

    void Monte_Carlo_Integrator::trace_specular_ray(const Core::BVH::InterpolatedVertex& frag, Core::Ray& ray, const uint32_t depth)
    {
        if(depth == 0)
        {
            ray.m_payload = glm::vec4(0.0f, 0.f, 0.0f, 1.0f);
            ray.m_weight = 1.0f;
            return;
        }

        const auto material = m_material_manager.evaluate_material(frag.mMaterialID, frag.mUV);

        if(material.emissive.x > 0.0 || material.emissive.y > 0.0 || material.emissive.z > 0.0)
        {
            ray.m_payload *= glm::vec4(material.emissive, 1.0f);
            ray.m_weight = ray.m_weight == 0.0f ? 1.0f : ray.m_weight;
            return;
        }

        glm::vec4 specular = glm::vec4(material.specularRoughness.x, material.specularRoughness.y, material.specularRoughness.z, 1.0f);

        const glm::vec3 V = glm::normalize(glm::vec3(ray.mOrigin - frag.mPosition));

        Render::Sample sample = m_specular_sampler->generate_sample(frag.mNormal, V, material.specularRoughness.w);
        ray.m_weight += sample.P;

        const glm::vec3 specular_factor = Render::specular_GGX(frag.mNormal, V, sample.L, material.specularRoughness.w,
                                                           glm::vec3(material.specularRoughness.x, material.specularRoughness.y, material.specularRoughness.z));

        ray.mOrigin = frag.mPosition + glm::vec4(0.01f * sample.L, 0.0f);
        ray.mDirection = sample.L;

        Core::BVH::InterpolatedVertex intersection;
        if(m_bvh.get_closest_intersection(ray, &intersection))
        {
            ray.m_payload *= specular * glm::vec4(specular_factor, 1.0f) * sample.P;

            if(weighted_random_ray_type(material))
                trace_specular_ray(intersection, ray, depth - 1);
            else
                trace_diffuse_ray(intersection, ray, depth - 1);
        }
        else
        {
             ray.m_payload *= specular * glm::vec4(specular_factor, 1.0f) * sample.P * m_skybox->sample4(sample.L);
        }
    }


    bool Monte_Carlo_Integrator::weighted_random_ray_type(const Core::EvaluatedMaterial& mat)
    {
        const float diffuse_weight = mat.diffuse_magnitude();
        const float specular_weight = mat.specular_magnitude();

        const float rand = mDistribution(mGenerator);

        return rand <= (specular_weight / (specular_weight + diffuse_weight));
    }
}
