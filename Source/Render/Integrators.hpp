#ifndef INTEGRATORS_HPP
#define INTEGRATORS_HPP

#include "Core/UpperLevelBVH.hpp"
#include "Core/MaterialManager.hpp"
#include "Core/Scene.hpp"

namespace Core
{
    class ImageCube;
}

namespace Render
{

    class Integrator
    {
    public:

        Integrator(const Core::Acceleration_Structures::UpperLevelBVH&, Core::MaterialManager&, const std::vector<Scene::Light>& light_bounds);


        virtual glm::vec3 integrate_ray(const Scene::Camera& camera, const glm::uvec2& pixel, const uint32_t maxDepth) = 0;

    protected:

        const Core::Acceleration_Structures::UpperLevelBVH& m_bvh;
        Core::MaterialManager& m_material_manager;
        const std::vector<Scene::Light>& m_lights;
    };


    class Monte_Carlo_Integrator : public Integrator
    {
    public:

        Monte_Carlo_Integrator(const Core::Acceleration_Structures::UpperLevelBVH&,  Core::MaterialManager&, const std::vector<Scene::Light> &light_bounds, const Scene::Sun& sun, const uint64_t seed);

        virtual glm::vec3 integrate_ray(const Scene::Camera& camera, const glm::uvec2& pixel, const uint32_t maxDepth) final;

    private:

        bool sample_direct_lighting(const Core::Acceleration_Structures::InterpolatedVertex& frag, glm::vec3& radiance, float& pdf);

        void trace_ray(const Core::Acceleration_Structures::InterpolatedVertex& frag, Core::Ray& ray, const uint32_t depth);

        bool weighted_random_ray_type(const Core::EvaluatedMaterial& mat);

        std::mt19937_64 mGenerator;
        std::uniform_real_distribution<float> mDistribution;

        Core::Rand::Hammersley_Generator m_hammersley_generator;

        uint32_t m_max_depth;

        Scene::Sun m_sky_desc;
    };
}

#endif
