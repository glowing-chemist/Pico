#ifndef PICO_TONE_MAPPERS_HPP
#define PICO_TONE_MAPPERS_HPP

#include <glm/vec3.hpp>


namespace Util
{
    class Tiler;

    float get_luminance(const glm::vec3&);

    void reinhard_tone_mapping(glm::vec3* pixels, const glm::uvec2& resolution, Tiler& tiler);
}

#endif
