#ifndef PICO_DENOISERS_HPP
#define PICO_DENOISERS_HPP

#include <glm/vec3.hpp>

namespace Util
{
    glm::vec3* atrous_denoise(const glm::vec3* pixels, const glm::vec3* normals, const glm::vec3* position, const glm::vec3* diffuse, const glm::uvec2& dims);
}
#endif
