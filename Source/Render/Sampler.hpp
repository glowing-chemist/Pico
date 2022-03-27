#ifndef RT_SAMPLERS_HPP
#define RT_SAMPLERS_HPP

#include "Distributions.hpp"
#include "Core/RandUtils.hpp"

#include "glm/common.hpp"
#include <memory>

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

        Diffuse_Sampler(const uint64_t seed, std::unique_ptr<Distribution>& dist) :
            m_distribution(std::move(dist)),
            m_generator(seed) {}

        Sample generate_sample(const glm::vec3& N, const glm::vec3& V, const float R);

    private:

        std::unique_ptr<Distribution> m_distribution;

        Core::Rand::Hammersley_Generator m_generator;

    };

    class Specular_Sampler
    {
    public:

        Specular_Sampler(const uint64_t seed, std::unique_ptr<Distribution>& dist) :
            m_distribution(std::move(dist)),
            m_generator(seed) {}

        Sample generate_sample(const glm::vec3& N, const glm::vec3& V, const float R);

    private:

        std::unique_ptr<Distribution> m_distribution;

        Core::Rand::Hammersley_Generator m_generator;

    };

}
#endif
