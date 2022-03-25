#ifndef VECTOR_UTIL_HPP
#define VECTOR_UTIL_HPP

#include "glm/vec3.hpp"
#include "glm/vec4.hpp"
#include "glm/mat3x3.hpp"
#include <cstdint>


namespace Core
{
    inline uint32_t pack_colour(const glm::vec4& colour)
    {
        return uint32_t(colour.r * 255.0f) | (uint32_t(colour.g * 255.0f) << 8) | (uint32_t(colour.b * 255.0f) << 16) | (uint32_t(colour.a * 255.0f) << 24);
    }

    inline glm::vec4 unpack_colour(const uint32_t colour)
    {
        return glm::vec4{(colour & 0xFF) / 255.0f, ((colour & 0xFF00) >> 8) / 255.0f, ((colour & 0xFF0000) >> 16) / 255.0f, ((colour & 0xFF000000) >> 24) / 255.0f};
    }

    inline glm::vec3 spherical_direction(float sinTheta,
            float cosTheta, float phi)
    {
        return glm::vec3(sinTheta * std::cos(phi),
                        sinTheta * std::sin(phi),
                        cosTheta);
    }

    namespace TangentSpace
    {
        glm::mat3x3 construct_world_to_tangent_transform(const glm::vec3& V, const glm::vec3& N);

        inline float cos_theta(const glm::vec3& w) { return w.z; }

        inline float cos2_theta(const glm::vec3& w) { return w.z * w.z; }

        inline float abs_cos_theta(const glm::vec3& w) { return std::abs(w.z); }

        inline float sin2_theta(const glm::vec3 &w) { return std::max((float)0, (float)1 - cos2_theta(w)); }

        inline float sin_theta(const glm::vec3 &w) { return std::sqrt(sin2_theta(w)); }

        inline float tan_theta(const glm::vec3& w) { return sin_theta(w) / cos_theta(w); }

        inline float tan2_theta(const glm::vec3& w) { return sin2_theta(w) / cos2_theta(w); }

        inline float cos_phi(const glm::vec3& w)
        {
            float sinTheta = sin_theta(w);
            return (sinTheta == 0) ? 1 : glm::clamp(w.x / sinTheta, -1.0f, 1.0f);
        }

        inline float sin_phi(const glm::vec3& w)
        {
            float sinTheta = sin_theta(w);
            return (sinTheta == 0) ? 0 : glm::clamp(w.y / sinTheta, -1.0f, 1.0f);
        }

        inline float cos2_phi(const glm::vec3& w)
        {
            return cos_phi(w) * cos_phi(w);
        }

        inline float sin2_phi(const glm::vec3& w)
        {
            return sin_phi(w) * sin_phi(w);
        }

        inline bool same_hemisphere(const glm::vec3& w, const glm::vec3& wp)
        {
            return w.z * wp.z > 0;
        }
    }
}

#endif
