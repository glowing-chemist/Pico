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

    class DiffuseSampler
    {
    public:
        DiffuseSampler(const uint32_t maxSamples) :
        mMaxSamples(maxSamples),
        mGeneratedSamples(0) {}

        Sample generateSample(const glm::vec3 &normal);

    private:

        uint32_t mMaxSamples;
        uint32_t mGeneratedSamples;
    };


    class SpecularSampler
    {
    public:
        SpecularSampler(const uint32_t maxSamples) :
        mMaxSamples(maxSamples),
        mGeneratedSamples(0) {}

        Sample generateSample(const glm::vec3& N, const glm::vec3& V, const float R);

    private:
        uint32_t mMaxSamples;
        uint32_t mGeneratedSamples;
    };

}
#endif
