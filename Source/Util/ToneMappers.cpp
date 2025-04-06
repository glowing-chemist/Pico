#include "ToneMappers.hpp"
#include "Core/AABB.hpp"
#include "Tiler.hpp"

namespace Util
{
    float get_luminance(const glm::vec3& colour)
    {
        return glm::dot(colour, glm::vec3(0.2126f, 0.587f, 0.114f));    
    }

    void reinhard_tone_mapping(glm::vec3* pixels, const glm::uvec2& resolution, Tiler& tiler)
    {
        const uint32_t pixel_count = resolution.x * resolution.y;
        float white_point = 0.0f;
        for(uint32_t i = 0; i < pixel_count; ++i)
        {
            white_point = std::max(white_point, get_luminance(pixels[i]));
        }

        const auto apply_tonemap = [](  const glm::uvec2 start, 
                                        const glm::uvec2& tile_size, 
                                        const glm::uvec2& resolution,
                                        const uint32_t,
                                        glm::vec3* pixels,
                                        const float white_point)
        {
            for(uint32_t x = start.x; x < start.x + tile_size.x; ++x)
            {
                for(uint32_t y = start.y; y < start.y + tile_size.y; ++y)
                {
                    const uint32_t flat_location = (y * resolution.x) + x;

                    const glm::vec3 numerator = pixels[flat_location] * (1.0f + (pixels[flat_location] / (white_point * white_point)));
                    pixels[flat_location] = numerator / (1.0f + pixels[flat_location]);
                }

            }

            return true;
        };

        tiler.execute_over_surface(apply_tonemap, pixels, white_point);
    }
}
