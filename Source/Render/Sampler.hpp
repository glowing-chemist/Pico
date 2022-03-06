#ifndef RT_SAMPLERS_HPP
#define RT_SAMPLERS_HPP

#include "glm/common.hpp"

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
        Hammersley_GGX_Diffuse_Sampler(const uint32_t maxSamples) :
        mMaxSamples(maxSamples),
        mGeneratedSamples(0) {}

        virtual Sample generate_sample(const glm::vec3 &normal) final;

    private:

        uint32_t mMaxSamples;
        uint32_t mGeneratedSamples;
    };


    class Hammersley_GGX_Specular_Sampler : public Specular_Sampler
    {
    public:
        Hammersley_GGX_Specular_Sampler(const uint32_t maxSamples) :
        mMaxSamples(maxSamples),
        mGeneratedSamples(0) {}

        Sample generate_sample(const glm::vec3& N, const glm::vec3& V, const float R);

    private:
        uint32_t mMaxSamples;
        uint32_t mGeneratedSamples;
    };

}
#endif
