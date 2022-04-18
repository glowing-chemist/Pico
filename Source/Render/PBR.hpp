#ifndef PBR_HPP
#define PBR_HPP

#include "glm/common.hpp"

namespace Render
{
    glm::vec3 F_Schlick(const glm::vec3 f0, const float f90, const float u);

    float disney_diffuse(float  NdotV , float  NdotL , float  LdotH ,float linearRoughness);

    glm::vec3 specular_GGX(const glm::vec3& wi, const glm::vec3& wo, const float roughness, const glm::vec3& F0);

}

#endif
