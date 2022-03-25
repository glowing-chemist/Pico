#include "Sampler.hpp"
#include "Core/vectorUtils.hpp"
#include "Core/Asserts.hpp"
#include "Render/PBR.hpp"

#include "glm/ext.hpp"

#include <math.h>

namespace Render
{

    Sample Diffuse_Sampler::generate_sample(const glm::vec3& N, const glm::vec3 &V, const float R)
    {
        PICO_ASSERT_VALID(N);
        PICO_ASSERT_VALID(V);

        const glm::mat3x3 world_to_tangent_transform = Core::TangentSpace::construct_world_to_tangent_transform(V, N);
        const glm::mat3x3 tangent_to_world_transform = glm::inverse(world_to_tangent_transform);

        const glm::vec3 view_tangent = glm::normalize(world_to_tangent_transform * V);

        const glm::vec2 xi = hammersley(m_random_sample_distribution(m_generator), m_max_samples);
        const glm::vec3 H = m_distribution->sample(xi, view_tangent, R);
        const float pdf = m_distribution->pdf(view_tangent, H, R);
        PICO_ASSERT_VALID(H);

        const glm::vec3 world_space_L = glm::normalize(tangent_to_world_transform * H);
        PICO_ASSERT_VALID(world_space_L);

        Sample samp{};
        samp.L = world_space_L;
        samp.P = pdf;
        samp.energy = m_distribution->energy(view_tangent, H, R);

        return samp;
    }


    Sample Specular_Sampler::generate_sample(const glm::vec3& N, const glm::vec3& V, const float R)
    {
        PICO_ASSERT_VALID(N);
        PICO_ASSERT_VALID(V);

        const glm::mat3x3 world_to_tangent_transform = Core::TangentSpace::construct_world_to_tangent_transform(V, N);
        const glm::mat3x3 tangent_to_world_transform = glm::inverse(world_to_tangent_transform);

        const glm::vec3 view_tangent = glm::normalize(world_to_tangent_transform * V);

        // Sample microfacet direction
        const glm::vec2 Xi = hammersley(m_random_sample_distribution(m_generator), m_max_samples);
        const glm::vec3 H = m_distribution->sample(Xi, view_tangent, R);
        PICO_ASSERT_VALID(H);
        const glm::vec3 L = glm::normalize(glm::reflect(-view_tangent, H));
        const float pdf = m_distribution->pdf(L, view_tangent, R);
        PICO_ASSERT_VALID(L);

        // Bring the sample vector back in to world space from tangent.
        const glm::vec3 world_space_L = glm::normalize(tangent_to_world_transform * L);
        PICO_ASSERT_VALID(world_space_L);

        Sample samp{};
        samp.L = world_space_L;
        samp.P = pdf / (2.0f * glm::dot(view_tangent, H));
        samp.energy = m_distribution->energy(view_tangent, H, R);

        return samp;
    }

}
