#include "Distributions.hpp"
#include "Core/vectorUtils.hpp"
#include "Core/Asserts.hpp"
#include "Render/PBR.hpp"

#include <math.h>

#include "glm/glm.hpp"

namespace Render
{

    glm::vec3 Cos_Weighted_Hemisphere_Distribution::sample(const glm::vec2& Xi, const glm::vec3&, const float)
    {
        const float  u1 = Xi.x;
        const float  u2 = Xi.y;

        const float r = std::sqrt(u1);
        const float  phi = u2 * M_PI * 2.0f;

        return glm::normalize(glm::vec3(r * cos(phi), r*sin(phi), sqrt(std::max(0.0f,1.0f-u1))));
    }

    float Cos_Weighted_Hemisphere_Distribution::pdf(const glm::vec3&, const glm::vec3& H, const float)
    {
        const float cos_theta = Core::TangentSpace::cos_theta(H);
        return std::max(0.0f, cos_theta / float(M_PI));
    }

    glm::vec3 Cos_Weighted_Hemisphere_Distribution::energy(const glm::vec3& wo, const glm::vec3& wi, const float R)
    {
        const glm::vec3 H = glm::normalize(wo + wi);
        PICO_ASSERT_VALID(H);
        const float NdotV = Core::TangentSpace::cos_theta(wo);
        const float NdotL = Core::TangentSpace::cos_theta(wi);
        const float LdotH = glm::dot(wi, H);

        return glm::vec3(Render::disney_diffuse(NdotV, NdotL, LdotH, R));
    }


    glm::vec3 Beckmann_All_Microfacet_Distribution::sample(const glm::vec2& Xi, const glm::vec3& V, const float R)
    {
        const float alpha = R * R;
        float logSample = std::log(1.0f - Xi.x);
        if (std::isinf(logSample)) logSample = 0;
        const float tan2Theta = -alpha * alpha * logSample;
        const float phi = Xi.y * 2.0f * M_PI;

        const float cosTheta = 1.0f / std::sqrt(1.0f + tan2Theta);
        const float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
        glm::vec3 H = Core::spherical_direction(sinTheta, cosTheta, phi);
        if (!Core::TangentSpace::same_hemisphere(V, H))
             H = -H;

        H = glm::normalize(H);
        PICO_ASSERT_VALID(H);
        return H;
    }

    float Beckmann_All_Microfacet_Distribution::pdf(const glm::vec3&, const glm::vec3& H, const float R)
    {
        PICO_ASSERT_VALID(H);
        float d = D(H, R);
        d = std::clamp(d, 0.0f, 1.0f);
        return d * Core::TangentSpace::abs_cos_theta(H);
    }

    glm::vec3 Beckmann_All_Microfacet_Distribution::energy(const glm::vec3& wo, const glm::vec3& wi, const float R)
    {
        return Render::specular_GGX(wi, wo, R, glm::vec3(1.0f));
    }

    float Beckmann_All_Microfacet_Distribution::roughness_to_alpha(float roughness) const
    {
        roughness = std::max(roughness, float(1e-3));
        float x = std::log(roughness);
        return 1.62142f + 0.819955f * x + 0.1734f * x * x +
               0.0171201f * x * x * x + 0.000640711f * x * x * x * x;
    }

    float Beckmann_All_Microfacet_Distribution::D(const glm::vec3 &wh, const float R) const
    {
        const float alphax =  roughness_to_alpha(R);
        const float alphay = alphax;

        float tan2Theta = Core::TangentSpace::tan2_theta(wh);
        if (std::isinf(tan2Theta))
            return 0.0f;
        const float cos4Theta = Core::TangentSpace::cos2_theta(wh) * Core::TangentSpace::cos2_theta(wh);
        return std::exp(-tan2Theta * (Core::TangentSpace::cos2_phi(wh) / (alphax * alphax) +
                                      Core::TangentSpace::sin2_phi(wh) / (alphay * alphay))) /
            (M_PI * alphax * alphay * cos4Theta);
    }

}
