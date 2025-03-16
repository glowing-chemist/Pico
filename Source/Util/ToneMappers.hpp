#ifndef PICO_TONE_MAPPERS_HPP
#define PICO_TONE_MAPPERS_HPP

#include <glm/vec3.hpp>


namespace Util
{
    void reinhard_tone_mapping(glm::vec3* pixels, const uint32_t pixel_count);
}

#endif
