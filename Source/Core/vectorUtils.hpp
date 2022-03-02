#ifndef VECTOR_UTIL_HPP
#define VECTOR_UTIL_HPP

#include "glm/vec4.hpp"
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
}

#endif
