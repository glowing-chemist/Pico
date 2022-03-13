#ifndef RT_SAMPLERS_HPP
#define RT_SAMPLERS_HPP

#include "glm/common.hpp"
#include <random>

namespace Render
{

    struct Sample
    {
        glm::vec3 L;
        float P;
    };

    class Diffuse_Sampler
    {
    public:

        Diffuse_Sampler() = default;

        virtual Sample generate_sample(const glm::vec3 &normal) = 0;

    };

    class Specular_Sampler
    {
    public:

        Specular_Sampler() = default;

        virtual Sample generate_sample(const glm::vec3& N, const glm::vec3& V, const float R) = 0;

    };

    class Hammersley_GGX_Diffuse_Sampler : public Diffuse_Sampler
    {
    public:
        Hammersley_GGX_Diffuse_Sampler(const uint32_t maxSamples, const uint64_t seed) :
            mMaxSamples(maxSamples),
            mGenerator{seed},
            mDistribution(0, maxSamples) {}

        virtual Sample generate_sample(const glm::vec3 &normal) final;

    private:

        uint32_t mMaxSamples;

        std::mt19937 mGenerator;
        std::uniform_int_distribution<> mDistribution;
    };


    class Hammersley_GGX_Specular_Sampler : public Specular_Sampler
    {
    public:
        Hammersley_GGX_Specular_Sampler(const uint32_t maxSamples, const uint64_t seed) :
            mMaxSamples(maxSamples),
            mGenerator{seed},
            mDistribution(0, maxSamples) {}

        Sample generate_sample(const glm::vec3& N, const glm::vec3& V, const float R);

    private:
        uint32_t mMaxSamples;

        std::mt19937 mGenerator;
        std::uniform_int_distribution<> mDistribution;
    };

}
#endif
