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

    float Cos_Weighted_Hemisphere_Distribution::pdf(const glm::vec3& wo, const glm::vec3&, const float)
    {
        const float cos_theta = Core::TangentSpace::cos_theta(wo);
        return std::max(0.0f, cos_theta / float(M_PI));
    }

    glm::vec3 Cos_Weighted_Hemisphere_Distribution::energy(const glm::vec3& wo, const glm::vec3& wi, const float R)
    {
        const glm::vec3 H = glm::normalize(wo + wi);
        const float NdotV = Core::TangentSpace::cos_theta(wo);
        const float NdotL = Core::TangentSpace::cos_theta(wi);
        const float LdotH = glm::dot(wi, H);

        return glm::vec3(Render::disney_diffuse(NdotV, NdotL, LdotH, R));
    }


    glm::vec3 Beckmann_All_Microfacet_Distribution::sample(const glm::vec2& Xi, const glm::vec3& V, const float R)
    {
        const float alpha = R * R;
        float logSample = std::log(1 - Xi.x);
        if (std::isinf(logSample)) logSample = 0;
        const float tan2Theta = -alpha * alpha * logSample;
        const float phi = Xi.y * 2 * M_PI;

        const float cosTheta = 1 / std::sqrt(1 + tan2Theta);
        const float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
        glm::vec3 H = Core::spherical_direction(sinTheta, cosTheta, phi);
        if (!Core::TangentSpace::same_hemisphere(V, H))
             H = -H;

        PICO_ASSERT_VALID(H);
        return H;
    }

    float Beckmann_All_Microfacet_Distribution::pdf(const glm::vec3& wo, const glm::vec3& wi, const float R)
    {
        const glm::vec3 H = glm::normalize(wo + wi);
        return D(H, R) * Core::TangentSpace::abs_cos_theta(H);
    }

    glm::vec3 Beckmann_All_Microfacet_Distribution::energy(const glm::vec3& wo, const glm::vec3& wi, const float R)
    {
        return glm::vec3(1.0f);
    }

    float Beckmann_All_Microfacet_Distribution::D(const glm::vec3 &wh, const float a)
    {
        const float alphax = std::sqrt(2.0f * a);
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
