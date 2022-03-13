#ifndef INTEGRATORS_HPP
#define INTEGRATORS_HPP

#include "Core/UpperLevelBVH.hpp"
#include "Core/MaterialManager.hpp"
#include "Sampler.hpp"

#include <memory>

namespace Core
{
    class ImageCube;
}

namespace Render
{

    class Integrator
    {
    public:

        Integrator(const Core::BVH::UpperLevelBVH&, Core::MaterialManager&);


        virtual glm::vec4 integrate_ray(const Core::Ray& ray, const uint32_t maxDepth, const uint32_t rayCount) = 0;

    protected:

        const Core::BVH::UpperLevelBVH& m_bvh;
        Core::MaterialManager& m_material_manager;
    };


    class Monte_Carlo_Integrator : public Integrator
    {
    public:

        Monte_Carlo_Integrator(const Core::BVH::UpperLevelBVH&,  Core::MaterialManager&, std::shared_ptr<Core::ImageCube>& skybox,
                               std::unique_ptr<Diffuse_Sampler>& diffuseSampler, std::unique_ptr<Specular_Sampler>& specSampler);

        virtual glm::vec4 integrate_ray(const Core::Ray& ray, const uint32_t maxDepth, const uint32_t rayCount) final;

    private:

        void trace_diffuse_ray(const Core::BVH::InterpolatedVertex& frag, Core::Ray& ray, const uint32_t depth);
        void trace_specular_ray(const Core::BVH::InterpolatedVertex& frag, Core::Ray& ray, const uint32_t depth);

        std::unique_ptr<Diffuse_Sampler> m_diffuse_sampler;
        std::unique_ptr<Specular_Sampler> m_specular_sampler;

        std::shared_ptr<Core::ImageCube> m_skybox;
    };
}

#endif
