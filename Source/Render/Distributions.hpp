#ifndef DISTRIBUTIONS_HPP
#define DISTRIBUTIONS_HPP

#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

namespace Render
{

    class Distribution
    {
    public:

        Distribution() = default;
        virtual ~Distribution() = default;

        virtual glm::vec3 sample(const glm::vec2& Xi, const glm::vec3& V, const float R) = 0;

        virtual float pdf(const glm::vec3& wo, const glm::vec3& H, const float R) = 0;

        virtual glm::vec3 energy(const glm::vec3& wo, const glm::vec3& wi, const float R) = 0;

    };


    class Cos_Weighted_Hemisphere_Distribution : public Distribution
    {
    public:

        Cos_Weighted_Hemisphere_Distribution() = default;

        virtual glm::vec3 sample(const glm::vec2& Xi, const glm::vec3& V, const float R) final;

        virtual float pdf(const glm::vec3& wo, const glm::vec3& H, const float R) final;

        virtual glm::vec3 energy(const glm::vec3& wo, const glm::vec3& wi, const float R) final;
    };

    class Beckmann_All_Microfacet_Distribution : public Distribution
    {
    public:

        Beckmann_All_Microfacet_Distribution() = default;

        virtual glm::vec3 sample(const glm::vec2& Xi, const glm::vec3& V, const float R) final;

        virtual float pdf(const glm::vec3&, const glm::vec3& H, const float R) final;

        virtual glm::vec3 energy(const glm::vec3& wo, const glm::vec3& wi, const float R) final;

    private:

        float roughness_to_alpha(float) const;

        float D(const glm::vec3 &wh, const float R) const;
    };
}

#endif
