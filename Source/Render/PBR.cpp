#include "PBR.hpp"

#include "glm/ext.hpp"

#ifndef M_PI
    #define M_PI           3.14159265358979323846
#endif

namespace Render
{

    void  importance_sample_CosDir(const glm::vec2& u, const glm::vec3& N, glm::vec3& L, float& NdotL)
    {
        //  Local  referencial
        glm::vec3  upVector = abs(N.z) < 0.999 ? glm::vec3 (0,0,1) : glm::vec3 (1,0,0);
        glm::vec3  tangentX = normalize( cross( upVector , N ) );
        glm::vec3  tangentY = cross( N, tangentX );

        float  u1 = u.x;
        float  u2 = u.y;

        float r = sqrt(u1);
        float  phi = u2 * M_PI * 2;

        L = glm::vec3(r*cos(phi), r*sin(phi), sqrt(std::max(0.0f,1.0f-u1)));
        L = normalize(tangentX * L.y + tangentY * L.x + N * L.z);

        NdotL = dot(L,N);
    }

    glm::vec3 importance_sample_GGX(const glm::vec2& Xi, const float roughness, const glm::vec3& N)
    {
        float a = roughness*roughness;

        float phi = 2.0 * M_PI * Xi.x;
        float cosTheta = sqrt((1.0 - Xi.y) / (1.0 + (a*a - 1.0) * Xi.y));
        float sinTheta = sqrt(1.0 - cosTheta*cosTheta);

        // from spherical coordinates to cartesian coordinates
        glm::vec3 H;
        H.x = cos(phi) * sinTheta;
        H.y = sin(phi) * sinTheta;
        H.z = cosTheta;

        // from tangent-space vector to world-space sample vector
        glm::vec3 up        = abs(N.z) < 0.999 ? glm::vec3(0.0, 0.0, 1.0) : glm::vec3(1.0, 0.0, 0.0);
        glm::vec3 tangent   = normalize(cross(up, N));
        glm::vec3 bitangent = cross(N, tangent);

        glm::vec3 sampleVec = tangent * H.x + bitangent * H.y + N * H.z;
        return normalize(sampleVec);
    }

    float radical_inverse_VdC(uint32_t bits)
    {
        bits = (bits << 16u) | (bits >> 16u);
        bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
        bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
        bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
        bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
        return float(bits) * 2.3283064365386963e-10; // / 0x100000000
    }


    glm::vec3 hemisphere_sample_uniform(float u, float v)
    {
        float phi = v * 2.0 * M_PI;
        float cosTheta = 1.0 - u;
        float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
        return glm::vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
    }


    glm::vec3 hemisphere_sample_cos(float u, float v)
    {
        float phi = v * 2.0 * M_PI;
        float cosTheta = sqrt(1.0 - u);
        float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
        return glm::vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
    }


    glm::vec2 hammersley(uint32_t i, uint32_t N)
    {
        return glm::vec2(float(i)/float(N), radical_inverse_VdC(i));
    }


    glm::vec3 hamersley_uniform(uint32_t i, uint32_t N)
    {
        glm::vec2 ham = hammersley(i, N);
        return hemisphere_sample_uniform(ham.x, ham.y);
    }


    glm::vec3 hamersley_cosine(uint32_t i, uint32_t N)
    {
        glm::vec2 ham = hammersley(i, N);
        return hemisphere_sample_cos(ham.x, ham.y);
    }

    glm::vec3 F_Schlick(const glm::vec3 f0, const float f90, const float u)
    {
        return  f0 + (f90 - f0) * std::pow (1.0f - u, 5.0f);
    }

    float smith_GGX_masking_shadowing(glm::vec3 N, glm::vec3 wi, glm::vec3 wo, float a2)
    {
        float dotNL = glm::dot(N, wi);
        float dotNV = glm::dot(N, wo);

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

    glm::vec3 specular_GGX(const glm::vec3& N, const glm::vec3& wi, const glm::vec3& wo, const float roughness, const glm::vec3& F0)
    {
        const glm::vec3 H = glm::normalize(wi + wo);

        float a2 = roughness * roughness;
        if(glm::dot(N, wi) > 0.0f && glm::dot(wi, H) > 0.0f)
        {

            float dotWiWm = glm::dot(wi, H);

            const glm::vec3 F = F_Schlick(F0, 1.0f, dotWiWm);
            float G = smith_GGX_masking_shadowing(N, wi, wo, a2);
            float weight = std::abs(glm::dot(wo, H))
                         / (glm::dot(N, wo) * glm::dot(N, H));

            return F * G * weight;
        }
        else
            return glm::vec3(0.0f);
    }

}
