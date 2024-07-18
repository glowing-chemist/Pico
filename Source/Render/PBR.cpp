#include "PBR.hpp"
#include "Core/vectorUtils.hpp"
#include "Core/Asserts.hpp"

#include "glm/ext.hpp"

#ifndef M_PI
    #define M_PI           3.14159265358979323846
#endif

namespace Render
{
    glm::vec3 F_Schlick(const glm::vec3 f0, const float f90, const float u)
    {
        return  f0 + (f90 - f0) * std::pow (1.0f - u, 5.0f);
    }

    float smith_GGX_masking_shadowing(glm::vec3 wi, glm::vec3 wo, float a2)
    {
        float dotNL = Core::TangentSpace::cos_theta(wi);
        float dotNV = Core::TangentSpace::cos_theta(wo);

        float denomA = dotNV * std::sqrt(a2 + (1.0f - a2) * dotNL * dotNL);
        float denomB = dotNL * std::sqrt(a2 + (1.0f - a2) * dotNV * dotNV);

        return 2.0f * dotNL * dotNV / (denomA + denomB);
    }

    float disney_diffuse(float  NdotV , float  NdotL , float  LdotH ,float linearRoughness)
    {
        float  energyBias = std::lerp(0.0f, 0.5f,  linearRoughness);
        float  energyFactor = std::lerp (1.0f, 1.0f / 1.51f,  linearRoughness);
        float  fd90 = energyBias + 2.0f * LdotH*LdotH * linearRoughness;
        glm::vec3  f0 = glm::vec3 (1.0f, 1.0f, 1.0f);
        float  lightScatter   = F_Schlick(f0, fd90 , NdotL).r;
        float  viewScatter    = F_Schlick(f0, fd90 , NdotV).r;

        return  lightScatter * viewScatter * energyFactor;
    }

    glm::vec3 specular_GGX(const glm::vec3& wi, const glm::vec3& wo, const float roughness, const glm::vec3& F0)
    {
        const glm::vec3 H = glm::normalize(wi + wo);

        float a2 = roughness * roughness;
        if(Core::TangentSpace::cos_theta(wi) > 0.0f && glm::dot(wi, H) > 0.0f)
        {

            const float dotWiWm = glm::dot(wi, H);

            float  energyBias = std::lerp(0.0f, 0.5f,  roughness);
            float  fd90 = energyBias + 2.0f * dotWiWm*dotWiWm * roughness;

            const glm::vec3 F = F_Schlick(F0, fd90, dotWiWm);
            float G = smith_GGX_masking_shadowing(wi, wo, a2);
            float weight = std::abs(glm::dot(wo, H))
                         / (Core::TangentSpace::cos_theta(wo) * Core::TangentSpace::cos_theta(H));

            return F * G * weight;
        }
        else
            return glm::vec3(0.0f);
    }

}
