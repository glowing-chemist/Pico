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

        Integrator(const Core::BVH::UpperLevelBVH&, Core::MaterialManager&, const std::vector<Core::AABB>& light_bounds);


        virtual glm::vec4 integrate_ray(const Core::Ray& ray, const uint32_t maxDepth, const uint32_t rayCount) = 0;

    protected:

        const Core::BVH::UpperLevelBVH& m_bvh;
        Core::MaterialManager& m_material_manager;
        std::vector<Core::AABB> m_light_bounds;
    };


    class Monte_Carlo_Integrator : public Integrator
    {
    public:

        Monte_Carlo_Integrator(const Core::BVH::UpperLevelBVH&,  Core::MaterialManager&, const std::vector<Core::AABB> &light_bounds, std::shared_ptr<Core::ImageCube>& skybox,
                               std::unique_ptr<Diffuse_Sampler>& diffuseSampler, std::unique_ptr<Specular_Sampler>& specSampler, const uint64_t seed);

        virtual glm::vec4 integrate_ray(const Core::Ray& ray, const uint32_t maxDepth, const uint32_t rayCount) final;

    private:

        Render::Sample generate_next_diffuse_event(const glm::vec3& pos, const glm::vec3& N, const glm::vec3& V, const float R);
        Render::Sample generate_next_specular_event(const glm::vec3& pos, const glm::vec3& N, const glm::vec3& V, const float R);

        void trace_diffuse_ray(const Core::BVH::InterpolatedVertex& frag, Core::Ray& ray, const uint32_t depth);
        void trace_specular_ray(const Core::BVH::InterpolatedVertex& frag, Core::Ray& ray, const uint32_t depth);

        bool weighted_random_ray_type(const Core::EvaluatedMaterial& mat);

        std::mt19937 mGenerator;
        std::uniform_real_distribution<float> mDistribution;

        std::unique_ptr<Diffuse_Sampler> m_diffuse_sampler;
        std::unique_ptr<Specular_Sampler> m_specular_sampler;

        std::shared_ptr<Core::ImageCube> m_skybox;
    };
}

#endif
