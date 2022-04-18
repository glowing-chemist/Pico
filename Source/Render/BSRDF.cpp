#include "BSRDF.hpp"

#include "Core/AABB.hpp"
#include "Core/vectorUtils.hpp"
#include "Core/Asserts.hpp"
#include "Core/LowerLevelBVH.hpp"


namespace Render
{

    Sample Diffuse_BRDF::sample(Core::Rand::Hammersley_Generator& rand, const Core::BVH::InterpolatedVertex &position, Core::Ray& ray)
    {
        const glm::vec3 V = -ray.mDirection;

        PICO_ASSERT_VALID(position.mNormal);
        PICO_ASSERT_VALID(V);

        const glm::mat3x3 world_to_tangent_transform = Core::TangentSpace::construct_world_to_tangent_transform(V, position.mNormal);
        const glm::mat3x3 tangent_to_world_transform = glm::inverse(world_to_tangent_transform);

        const glm::vec3 view_tangent = glm::normalize(world_to_tangent_transform * V);

        Core::EvaluatedMaterial material = m_material_manager.evaluate_material(m_mat_id, position.mUV);

        const glm::vec2 xi = rand.next();
        const glm::vec3 H = m_distribution->sample(xi, view_tangent, material.roughness);
        PICO_ASSERT_VALID(H);
        const float pdf = m_distribution->pdf(view_tangent, H, material.roughness);
        PICO_ASSERT(!std::isinf(pdf) && !std::isnan(pdf));

        const glm::vec3 world_space_L = glm::normalize(tangent_to_world_transform * H);
        PICO_ASSERT_VALID(world_space_L);

        Sample samp{};
        samp.L = world_space_L;
        samp.P = pdf;
        const glm::vec3 distribution_energy = m_distribution->energy(view_tangent, H, material.roughness);
        samp.energy = distribution_energy * material.diffuse;

        return samp;
    }


    Sample Specular_BRDF::sample(Core::Rand::Hammersley_Generator& rand, const Core::BVH::InterpolatedVertex &position, Core::Ray &ray)
    {
        const glm::vec3 V = -ray.mDirection;

        PICO_ASSERT_VALID(position.mNormal);
        PICO_ASSERT_VALID(V);

        const glm::mat3x3 world_to_tangent_transform = Core::TangentSpace::construct_world_to_tangent_transform(V, position.mNormal);
        const glm::mat3x3 tangent_to_world_transform = glm::inverse(world_to_tangent_transform);

        const glm::vec3 view_tangent = glm::normalize(world_to_tangent_transform * V);

        Core::EvaluatedMaterial material = m_material_manager.evaluate_material(m_mat_id, position.mUV);

        // Sample microfacet direction
        const glm::vec2 Xi = rand.next();
        const glm::vec3 H = m_distribution->sample(Xi, view_tangent, material.roughness);
        PICO_ASSERT_VALID(H);
        const glm::vec3 L = glm::normalize(glm::reflect(-view_tangent, H));
        PICO_ASSERT_VALID(L);
        const float pdf = m_distribution->pdf(view_tangent, H, material.roughness);
        PICO_ASSERT(!std::isinf(pdf) && !std::isnan(pdf));
        //PICO_ASSERT(pdf > 0.0f && pdf < 1.0f);

        // Bring the sample vector back in to world space from tangent.
        const glm::vec3 world_space_L = glm::normalize(tangent_to_world_transform * L);
        PICO_ASSERT_VALID(world_space_L);

        Sample samp{};
        samp.L = world_space_L;
        samp.P = pdf / (2.0f * glm::dot(view_tangent, H));
        samp.energy = m_distribution->energy(view_tangent, H, material.roughness) * material.specular;

        return samp;
    }

    Sample Light_BRDF::sample(Core::Rand::Hammersley_Generator&, const Core::BVH::InterpolatedVertex &position, Core::Ray&)
    {
        Core::EvaluatedMaterial material = m_material_manager.evaluate_material(m_mat_id, position.mUV);

        Sample samp{};
        samp.energy = material.emissive;
        samp.L = glm::vec3(0.0f);
        samp.P = 1.0f;

        return samp;
    }

    Sample Specular_Delta_BRDF::sample(Core::Rand::Hammersley_Generator&, const Core::BVH::InterpolatedVertex& position, Core::Ray &ray)
    {
        const glm::vec3 V = -ray.mDirection;

        Core::EvaluatedMaterial material = m_material_manager.evaluate_material(m_mat_id, position.mUV);

        Sample samp{};
        samp.energy = material.specular;
        samp.L = glm::reflect(-V, position.mNormal);
        samp.P = 1.0f;

        return samp;
    }

    Sample Transparent_BTDF::sample(Core::Rand::Hammersley_Generator& rand, const Core::BVH::InterpolatedVertex &position, Core::Ray& ray)
    {
        const glm::vec3 V = -ray.mDirection;

        PICO_ASSERT_VALID(position.mNormal);
        PICO_ASSERT_VALID(V);

        const glm::mat3x3 world_to_tangent_transform = Core::TangentSpace::construct_world_to_tangent_transform(V, position.mNormal);
        const glm::mat3x3 tangent_to_world_transform = glm::inverse(world_to_tangent_transform);

        const glm::vec3 view_tangent = glm::normalize(world_to_tangent_transform * V);
        const bool enter_object = Core::TangentSpace::cos_theta(view_tangent) >= 0.0f;

        PICO_ASSERT(glm::dot(V, position.mNormal) >= 0.0f || !enter_object);

        Core::EvaluatedMaterial material = m_material_manager.evaluate_material(m_mat_id, position.mUV);

        // Sample microfacet direction
        const glm::vec2 Xi = rand.next();
        glm::vec3 H = m_distribution->sample(Xi, view_tangent, material.roughness);
        if(!enter_object)
            H = -H;
        PICO_ASSERT_VALID(H);

        float eta;
        if(enter_object)
        {
            const float current_IoR = ray.get_current_index_of_refraction();
            const float etaI = current_IoR;
            const float etaT = m_index_of_refraction;
            eta = etaI / etaT;

            ray.push_index_of_refraction(m_index_of_refraction);
        }
        else
        {
            ray.pop_index_of_refraction();

            const float current_IoR = ray.get_current_index_of_refraction();
            const float etaI = m_index_of_refraction;
            const float etaT = current_IoR;
            eta = etaI / etaT;
        }

        glm::vec3 L;
        if(refract(view_tangent, H * (enter_object ? 1.0f : -1.0f), eta, L))
        {
            PICO_ASSERT_VALID(L);

            PICO_ASSERT(glm::dot(L, glm::vec3(0, 0, enter_object ? -1.0f : 1.0f)) >= 0.0f);

            // Bring the sample vector back in to world space from tangent.
            const glm::vec3 world_space_L = glm::normalize(tangent_to_world_transform * L);
            PICO_ASSERT_VALID(world_space_L);

            // Calculate pdf
            const float pdf = m_distribution->pdf(view_tangent, H, material.roughness);
            PICO_ASSERT(!std::isinf(pdf) && !std::isnan(pdf));
            glm::vec3 wh = glm::normalize(view_tangent + L * eta);
            float sqrtDenom = glm::dot(view_tangent, wh) + eta * glm::dot(L, wh);
            float dwh_dwi = std::abs((eta * eta * glm::dot(L, wh)) / (sqrtDenom * sqrtDenom));

            Sample samp{};
            samp.L = world_space_L;
            samp.P = pdf * dwh_dwi;
            samp.energy = material.diffuse;

            return samp;
        }
        else
        {
            Sample samp{};
            samp.L = glm::vec3(0.0f);
            samp.P = 0.0f;
            samp.energy = material.diffuse;

            return samp;
        }
    }

    bool Transparent_BTDF::refract(const glm::vec3& wi, const glm::vec3& n, const float eta, glm::vec3& wt)
    {
#if 1
        const float cosThetaI = glm::dot(n, wi);
        const float sin2ThetaI = std::max(0.0f, 1.0f - cosThetaI * cosThetaI);
        const float sin2ThetaT = eta * eta * sin2ThetaI;
        if (sin2ThetaT >= 1.0f) return false;

        const float cosThetaT = std::sqrt(1.0f - sin2ThetaT);

        wt = glm::normalize(eta * -wi + (eta * cosThetaI - cosThetaT) * glm::vec3(n));
        PICO_ASSERT(glm::dot(wt, -n) >= 0.0f);
        return true;
#else

        wt = glm::refract(-wi, n, eta);
        return true;

#endif
    }

    Sample Fresnel_BTDF::sample(Core::Rand::Hammersley_Generator& rand, const Core::BVH::InterpolatedVertex& position, Core::Ray& ray)
    {
        const glm::vec3 V = -ray.mDirection;

        PICO_ASSERT_VALID(position.mNormal);
        PICO_ASSERT_VALID(V);

        const glm::mat3x3 world_to_tangent_transform = Core::TangentSpace::construct_world_to_tangent_transform(V, position.mNormal);
        const glm::vec3 view_tangent = glm::normalize(world_to_tangent_transform * V);

        const glm::vec2 Xi = rand.next();
        const float fresnel_term = fresnel_factor(Core::TangentSpace::cos_theta(view_tangent), m_transparent_bsrdf->get_index_of_refraction(), ray.get_current_index_of_refraction());
        Sample samp;
        if(Xi.x < fresnel_term)
        {
            samp =  m_specular_bsrdf->sample(rand, position, ray);
            samp.P *= fresnel_term;
        }
        else
        {
            samp = m_transparent_bsrdf->sample(rand, position, ray);
            samp.P *= 1.0f - fresnel_term;
        }

        return samp;
    }

    float Fresnel_BTDF::fresnel_factor(float cosThetaI, float etaI, float etaT)
    {
        cosThetaI = std::clamp(cosThetaI, -1.0f, 1.0f);
        bool entering = cosThetaI > 0.f;
        if (!entering)
        {
            std::swap(etaI, etaT);
            cosThetaI = std::abs(cosThetaI);
        }

        const float sinThetaI = std::sqrt(std::max(0.f,
                                                    1.0f - cosThetaI * cosThetaI));
        const float sinThetaT = etaI / etaT * sinThetaI;
        const float cosThetaT = std::sqrt(std::max(0.0f,
                                                    1.0f - sinThetaT * sinThetaT));

        const float Rparl = ((etaT * cosThetaI) - (etaI * cosThetaT)) /
                          ((etaT * cosThetaI) + (etaI * cosThetaT));
        const float Rperp = ((etaI * cosThetaI) - (etaT * cosThetaT)) /
                          ((etaI * cosThetaI) + (etaT * cosThetaT));
       return (Rparl * Rparl + Rperp * Rperp) / 2;
    }

}
