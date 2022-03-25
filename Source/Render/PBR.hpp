#ifndef PBR_HPP
#define PBR_HPP

#include "glm/common.hpp"

namespace Render
{
    float radical_inverse_VdC(uint32_t bits);

    glm::vec3 hemisphere_sample_uniform(float u, float v);


    glm::vec3 hemisphere_sample_cos(float u, float v);


    glm::vec2 hammersley(uint32_t i, uint32_t N);


    glm::vec3 hamersley_uniform(uint32_t i, uint32_t N);


    glm::vec3 hamersley_cosine(uint32_t i, uint32_t N);

    glm::vec3 F_Schlick(const glm::vec3 f0, const float f90, const float u);

    float beckmann_D(const glm::vec3& h, const float a);

    float disney_diffuse(float  NdotV , float  NdotL , float  LdotH ,float linearRoughness);

    glm::vec3 specular_GGX(const glm::vec3& N, const glm::vec3& V, const glm::vec3& L, const float roughness, const glm::vec3& F0);

}

#endif
