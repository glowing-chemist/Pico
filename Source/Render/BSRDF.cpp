#include "BSRDF.hpp"

#include "Core/vectorUtils.hpp"
#include "Core/Asserts.hpp"
#include "Core/LowerLevelBVH.hpp"


namespace Render
{

    Sample Diffuse_BRDF::sample(Core::Rand::Hammersley_Generator& rand, const Core::BVH::InterpolatedVertex &position, const glm::vec3& V)
    {
        PICO_ASSERT_VALID(position.mNormal);
        PICO_ASSERT_VALID(V);

        const glm::mat3x3 world_to_tangent_transform = Core::TangentSpace::construct_world_to_tangent_transform(V, position.mNormal);
        const glm::mat3x3 tangent_to_world_transform = glm::inverse(world_to_tangent_transform);

        const glm::vec3 view_tangent = glm::normalize(world_to_tangent_transform * V);

        Core::EvaluatedMaterial material = m_material_manager.evaluate_material(m_mat_id, position.mUV);

        const glm::vec2 xi = rand.next();
        const glm::vec3 H = m_distribution->sample(xi, view_tangent, material.roughness);
        const float pdf = m_distribution->pdf(view_tangent, H, material.roughness);
        PICO_ASSERT_VALID(H);

        const glm::vec3 world_space_L = glm::normalize(tangent_to_world_transform * H);
        PICO_ASSERT_VALID(world_space_L);

        Sample samp{};
        samp.L = world_space_L;
        samp.P = pdf;
        samp.energy = m_distribution->energy(view_tangent, H, material.roughness) * material.diffuse;

        return samp;
    }


    Sample Specular_BRDF::sample(Core::Rand::Hammersley_Generator& rand, const Core::BVH::InterpolatedVertex &position, const glm::vec3& V)
    {
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
        const float pdf = m_distribution->pdf(L, view_tangent, material.roughness);
        PICO_ASSERT_VALID(L);

        // Bring the sample vector back in to world space from tangent.
        const glm::vec3 world_space_L = glm::normalize(tangent_to_world_transform * L);
        PICO_ASSERT_VALID(world_space_L);

        Sample samp{};
        samp.L = world_space_L;
        samp.P = pdf / (2.0f * glm::dot(view_tangent, H));
        samp.energy = m_distribution->energy(view_tangent, H, material.roughness) * material.specular;

        return samp;
    }

    Sample Light_BRDF::sample(Core::Rand::Hammersley_Generator&, const Core::BVH::InterpolatedVertex &position, const glm::vec3&)
    {
        Sample samp{};

        Core::EvaluatedMaterial material = m_material_manager.evaluate_material(m_mat_id, position.mUV);
        samp.energy = material.emissive;
        samp.L = glm::vec3(0.0f);
        samp.P = 1.0f;

        return samp;
    }

}
