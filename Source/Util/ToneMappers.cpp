#include "ToneMappers.hpp"
#include "Core/AABB.hpp"


namespace Util
{
    void reinhard_tone_mapping(glm::vec3* pixels, const uint32_t pixel_count)
    {
        glm::vec3 white_point = glm::vec3(0.0f);
        for(uint32_t i = 0; i < pixel_count; ++i)
        {
            white_point = Core::component_wise_max(white_point, pixels[i]);
        }

        for(uint32_t i = 0; i < pixel_count; ++i)
        {
            const glm::vec3 numerator = pixels[i] * (1.0f + (pixels[i] / (white_point * white_point)));
            pixels[i] = numerator / (1.0f + pixels[i]);
        }
    }
}
