#include "RandUtils.hpp"

namespace Core
{

    namespace Rand
    {


        float radical_inverse_VdC(uint32_t bits)
        {
            bits = (bits << 16u) | (bits >> 16u);
            bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
            bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
            bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
            bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
            return float(bits) * 2.3283064365386963e-10; // / 0x100000000
        }

        glm::vec2 hammersley(uint32_t i, uint32_t N)
        {
            return glm::vec2(float(i)/float(N), radical_inverse_VdC(i));

        }

        glm::vec2 uniform_sample_triangle(const glm::vec2& Xi)
        {
            float su0 = std::sqrt(Xi.x);
            return glm::vec2(1 - su0, Xi.y * su0);
        }

        glm::vec3 uniform_sample_hemisphere(const glm::vec2& Xi)
        {
            const float z = Xi.x;
            const float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
            const float phi = 2 * M_PI * Xi.y;
            return glm::vec3(r * std::cos(phi), r * std::sin(phi), z);
        }

        glm::vec3 uniform_sample_sphere(const glm::vec2& Xi)
        {
            const float z = 1 - 2 * Xi.x;
            const float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
            const float phi = 2 * M_PI * Xi.y;
            return glm::vec3(r * std::cos(phi), r * std::sin(phi), z);
        }

        Hammersley_Generator::Hammersley_Generator(const size_t seed) :
            m_generator(seed),
            m_random_sample_distribution(0, kMaxSamples)
        {}

        glm::vec2 Hammersley_Generator::next()
        {
            return hammersley(m_random_sample_distribution(m_generator), kMaxSamples);
        }

    }

}
