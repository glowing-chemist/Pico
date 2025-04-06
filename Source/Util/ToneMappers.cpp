#include "ToneMappers.hpp"
#include "Core/AABB.hpp"

namespace Util
{
    float get_luminance(const glm::vec3& colour)
    {
        return glm::dot(colour, glm::vec3(0.2126f, 0.587f, 0.114f));    
    }

    void reinhard_tone_mapping(glm::vec3* pixels, const uint32_t pixel_count)
    {
        float white_point = 0.0f;
        for(uint32_t i = 0; i < pixel_count; ++i)
        {
            white_point = std::max(white_point, get_luminance(pixels[i]));
        }

        for(uint32_t i = 0; i < pixel_count; ++i)
        {
            const glm::vec3 numerator = pixels[i] * (1.0f + (pixels[i] / (white_point * white_point)));
            pixels[i] = numerator / (1.0f + pixels[i]);
        }
    }
}
