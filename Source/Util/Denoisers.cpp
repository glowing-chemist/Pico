#include "Denoisers.hpp"

#include "glm/common.hpp"
#include "glm/ext.hpp"
#include "glm/geometric.hpp"

#include <cmath>
#include <complex>
#include <glm/integer.hpp>

namespace
{
    glm::vec3 blend_pixel(const glm::uvec2& pixel, 
                          const uint32_t atrous_level, 
                          const glm::uvec2& dims, 
                          const glm::vec3* pixels, 
                          const glm::vec3* normals,
                          const glm::vec3* position, 
                          const glm::vec3* diffuse)
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
        const uint32_t center_flat_index = (pixel.y * dims.x) + pixel.x;

        glm::vec3 result = glm::vec3(0.0f, 0.0f, 0.0f);
        float weight = 0.0f;
        for(uint32_t u = 0; u < 25; ++u)
        {
            const glm::uvec2 index = glm::clamp(glm::ivec2(pixel) + (offsets[u] * atrous_scale), glm::ivec2(0, 0), glm::ivec2(dims) - glm::ivec2(1, 1));
            const size_t flat_index = (index.y * dims.x) + index.x; 

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

        return result / weight;
    }
}


namespace Util
{
    glm::vec3* atrous_denoise(const glm::vec3* pixels, const glm::vec3* normals, const glm::vec3* position, const glm::vec3* diffuse, const glm::uvec2& dims)
    {
        // Ping - Pong between the buffers for the a-trous levels.
        glm::vec3* result[2] = {new glm::vec3[dims.x * dims.y], new glm::vec3[dims.x * dims.y]}; 
        std::memcpy(result[0], pixels, sizeof(glm::vec3) * dims.x * dims.y);

        // 5 levels for a-trous denoising seems the standard approach.
        for(uint32_t level = 0; level < 5; ++level)
        {
            const uint32_t src_index = level % 2;
            const uint32_t dst_index = (level + 1) % 2;
            for(uint32_t x = 0; x < dims.x; ++x)
            {
                for(uint32_t y = 0; y < dims.y; ++y)
                {
                    const uint32_t flat_index = (y * dims.x) + x;
                    result[dst_index][flat_index] = blend_pixel(glm::uvec2(x, y), level, dims, result[src_index], normals, position, diffuse);
                }
            }
        }

        // final dest is buffer 1
        delete[] result[0];
        return result[1];
    }

}
