#include "Denoisers.hpp"

#include "glm/common.hpp"
#include "glm/ext.hpp"
#include "glm/geometric.hpp"

#include <cmath>
#include <glm/integer.hpp>

namespace
{
    bool blend_pixel(   const glm::uvec2 start, 
                        const glm::uvec2& tile_size, 
                        const glm::uvec2& resolution, 
                        const uint32_t,
                        const uint32_t atrous_level,  
                        const glm::vec3* pixels, 
                        const glm::vec3* normals,
                        const glm::vec3* position, 
                        const glm::vec3* diffuse,
                        glm::vec3* output)
    {
        // 5x5 gaussian kernel
        const float weights[25] = {0.00292, 0.01306, 0.02154, 0.01306, 0.00292,
                                   0.01306, 0.05855, 0.09653, 0.05855, 0.01306,
                                   0.02154, 0.09653, 0.15915, 0.09653, 0.02154,
                                   0.01306, 0.05855, 0.09563, 0.05855, 0.01306,
                                   0.00292, 0.01306, 0.02154, 0.01306, 0.00292};

        const glm::ivec2 offsets[25] = {{-2, -2}, {-1, -2}, {0, -2}, {1, -2}, {2, -2},
                                        {-2, -1}, {-1, -1}, {0, -1}, {1, -1}, {2, -1},
                                        {-2, -0}, {-1, 0},  {0, 0},  {1, 0},  {2, 0},
                                        {-2, 1},  {-1, 1},  {0, 1},  {1, 1},  {2, 1},
                                        {-2, 2},  {-1, 2},  {0, 2},  {1, 2},  {2, 2}};

        const int atrous_scale = std::pow(2, atrous_level);
       

        for(uint32_t x = start.x; x < start.x + tile_size.x; ++x)
        {
            for(uint32_t y = start.y; y < start.y + tile_size.y; ++y)
            {
                const glm::uvec2 pixel{x, y};
                const uint32_t center_flat_index = (pixel.y * resolution.x) + pixel.x;

                glm::vec3 result = glm::vec3(0.0f, 0.0f, 0.0f);
                float weight = 0.0f;
                for(uint32_t u = 0; u < 25; ++u)
                {
                    const glm::uvec2 index = glm::clamp(glm::ivec2(pixel) + (offsets[u] * atrous_scale), glm::ivec2(0, 0), glm::ivec2(resolution) - glm::ivec2(1, 1));
                    const size_t flat_index = (index.y * resolution.x) + index.x; 

                    const float v = 0.3f; 

                    glm::vec3 t = normals[center_flat_index] - normals[flat_index];
                    float dist = glm::dot(t, t);
                    const float normal_weight = std::min(std::exp(- dist / v), 1.0f);

                    t = position[center_flat_index] - position[flat_index];
                    dist = std::max(glm::dot(t, t) / (float(atrous_scale) * float(atrous_scale)), 0.0f);
                    const float position_weight = std::min(std::exp(-dist / v), 1.0f);

                    t = diffuse[center_flat_index] - diffuse[flat_index];
                    dist = glm::dot(t, t);
                    const float diffuse_weight = std::min(std::exp(-dist / v), 1.0f);

                    const float tap_weight = weights[u] * normal_weight * position_weight * diffuse_weight;

                    result += pixels[flat_index] * tap_weight;
                    weight += tap_weight;
                }

                output[center_flat_index] = result / weight;
            }
        }
        return true;
    }
}


namespace Util
{
    glm::vec3* atrous_denoise(const glm::vec3* pixels, const glm::vec3* normals, const glm::vec3* position, const glm::vec3* diffuse, Tiler& tiler)
    {
        const glm::uvec2& resolution = tiler.get_resolution();
        // Ping - Pong between the buffers for the a-trous levels.
        glm::vec3* result[2] = {new glm::vec3[resolution.x * resolution.y], new glm::vec3[resolution.x * resolution.y]}; 
        std::memcpy(result[0], pixels, sizeof(glm::vec3) * resolution.x * resolution.y);

        // 5 levels for a-trous denoising seems the standard approach.
        for(uint32_t level = 0; level < 5; ++level)
        {
            const uint32_t src_index = level % 2;
            const uint32_t dst_index = (level + 1) % 2;

            tiler.execute_over_surface(blend_pixel, level, result[src_index], normals, position, diffuse, result[dst_index]);
        }

        // final dest is buffer 1
        delete[] result[0];
        return result[1];
    }

}
