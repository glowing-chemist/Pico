#ifndef RT_SAMPLERS_HPP
#define RT_SAMPLERS_HPP

#include "Distributions.hpp"

#include "glm/common.hpp"
#include <memory>
#include <random>

namespace Render
{

    struct Sample
    {
        glm::vec3 L;
        float P;
        glm::vec3 energy;
    };

    class Diffuse_Sampler
    {
    public:

        Diffuse_Sampler(const uint32_t maxSamples, const uint64_t seed, std::unique_ptr<Distribution>& dist) :
            m_distribution(std::move(dist)),
            m_max_samples(maxSamples),
            m_generator(seed),
            m_random_sample_distribution(0, m_max_samples) {}

        Sample generate_sample(const glm::vec3& N, const glm::vec3& V, const float R);

    private:

        std::unique_ptr<Distribution> m_distribution;

        uint32_t m_max_samples;

        std::mt19937 m_generator;
        std::uniform_int_distribution<> m_random_sample_distribution;

    };

    class Specular_Sampler
    {
    public:

        Specular_Sampler(const uint32_t maxSamples, const uint64_t seed, std::unique_ptr<Distribution>& dist) :
            m_distribution(std::move(dist)),
            m_max_samples(maxSamples),
            m_generator(seed),
            m_random_sample_distribution(0, m_max_samples) {}

        Sample generate_sample(const glm::vec3& N, const glm::vec3& V, const float R);

    private:

        std::unique_ptr<Distribution> m_distribution;

        uint32_t m_max_samples;

        std::mt19937 m_generator;
        std::uniform_int_distribution<> m_random_sample_distribution;

    };

}
#endif
