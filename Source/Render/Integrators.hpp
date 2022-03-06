#ifndef INTEGRATORS_HPP
#define INTEGRATORS_HPP

#include "Core/UpperLevelBVH.hpp"
#include "Sampler.hpp"

#include <memory>

namespace Render
{

    class Integrator
    {
    public:

        Integrator(const Core::BVH::UpperLevelBVH&);


        virtual glm::vec4 integrate_ray(const Core::Ray ray, const uint32_t maxDepth, const uint32_t rayCount) = 0;

    protected:

        Core::BVH::UpperLevelBVH m_bvh;

    };


    class Monte_Carlo_Integrator : public Integrator
    {
    public:

        Monte_Carlo_Integrator(const Core::BVH::UpperLevelBVH&, std::unique_ptr<Diffuse_Sampler>& diffuseSampler, std::unique_ptr<Specular_Sampler>& specSampler);

        virtual glm::vec4 integrate_ray(const Core::Ray ray, const uint32_t maxDepth, const uint32_t rayCount) final;

    private:

        std::unique_ptr<Diffuse_Sampler> m_diffuse_sampler;
        std::unique_ptr<Specular_Sampler> m_specular_sampler;
    };
}

#endif
