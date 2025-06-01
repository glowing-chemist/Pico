#ifndef RAND_UTILS_HPP
#define RAND_UTILS_HPP

#include <limits>
#include <random>

#include "glm/glm.hpp"

namespace Core
{

    namespace Rand
    {

        float radical_inverse_VdC(uint32_t bits);

        glm::vec2 hammersley(uint32_t i, uint32_t N);

        glm::vec2 uniform_sample_triangle(const glm::vec2& Xi);

        glm::vec3 uniform_sample_hemisphere(const glm::vec2& Xi);

        glm::vec3 uniform_sample_sphere(const glm::vec2& Xi);

        template<typename T>
        inline uint32_t choose(const float r, const T probs, const float norm = 1.0f)
        {
            float running_total = 0.0f;
            for(uint32_t i = 0; i < probs.size(); ++i)
            {
                running_total += probs[i] / norm;

                if(r <= running_total)
                    return i;

            }

            return UINT_MAX;
        }

        class xorshift_random
        {
            public:

                xorshift_random(const uint32_t seed1);

                uint32_t next();

                // Operator to match the std random number generators.
                uint32_t operator()() 
                {
                    return next();
                }
    
                using result_type = uint32_t;

                static constexpr result_type min() { return 0; }
                static constexpr result_type max() { return std::numeric_limits<uint32_t>::max(); }

                result_type g() { return next(); }

            private:

                uint32_t rotl(const uint32_t x, int k);

                uint32_t m_state[4];
        };

        class Hammersley_Generator
        {
        public:
            Hammersley_Generator(const uint32_t seed);

            glm::vec2 next();

            xorshift_random& get_xor_random_generator() { return m_random_sample_distribution; }

        private:

            constexpr static size_t kMaxSamples = std::numeric_limits<uint32_t>::max();

            xorshift_random m_random_sample_distribution;

        };

    }

}

#endif
