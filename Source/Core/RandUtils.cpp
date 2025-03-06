#define _USE_MATH_DEFINES

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

        // Written in 2018 by David Blackman and Sebastiano Vigna (vigna@acm.org)
        uint32_t xorshift_random::rotl(const uint32_t x, int k)
        {
            return (x << k) | (x >> (32 - k));
        }

        uint32_t xorshift_random::next()
        {
            const uint32_t result = rotl(m_state[0] + m_state[3], 7) + m_state[0];;

            const uint32_t t = m_state[1] << 9;

            m_state[2] ^= m_state[0];
            m_state[3] ^= m_state[1];
            m_state[1] ^= m_state[2];
            m_state[0] ^= m_state[3];

            m_state[2] ^= t;

            m_state[3] = rotl(m_state[3], 11);

            return result;
        }

        Hammersley_Generator::Hammersley_Generator(const uint32_t seed) :
            m_random_sample_distribution(seed)
        {}

        glm::vec2 Hammersley_Generator::next()
        {
            return hammersley(m_random_sample_distribution.next(), kMaxSamples);
        }

    }

}
