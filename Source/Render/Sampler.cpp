#include "Sampler.hpp"
#include "Render/PBR.hpp"

#include "glm/ext.hpp"

namespace Render
{

    Sample Hammersley_GGX_Diffuse_Sampler::generate_sample(const glm::vec3& normal)
    {
        const glm::vec2 xi = hammersley(mDistribution(mGenerator), mMaxSamples);
        glm::vec3 L;
        float NdotL;
        importance_sample_CosDir(xi, normal, L, NdotL);

        return {L, 1.0f};
    }


    Sample Hammersley_GGX_Specular_Sampler::generate_sample(const glm::vec3& N, const glm::vec3& V, const float R)
    {
        glm::vec2 Xi = hammersley(mDistribution(mGenerator), mMaxSamples);
        // Sample microfacet direction
        glm::vec3 GGX_H = importance_sample_GGX(Xi, R, N);

        Sample samp{};
        // Get the light direction
        samp.L = glm::reflect(-V, GGX_H);
        const float NdotL = glm::dot(N, samp.L);
        samp.P = std::max(NdotL, 0.0f);

        return samp;
    }

}
