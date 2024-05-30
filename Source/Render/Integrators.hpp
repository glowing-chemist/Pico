#ifndef INTEGRATORS_HPP
#define INTEGRATORS_HPP

#include "Core/UpperLevelBVH.hpp"
#include "Core/MaterialManager.hpp"
#include "Core/Scene.hpp"

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

        Integrator(const Core::Acceleration_Structures::UpperLevelBVH&, Core::MaterialManager&, const std::vector<Scene::Light>& light_bounds);


        virtual glm::vec4 integrate_ray(const Core::Ray& ray, const uint32_t maxDepth, const uint32_t rayCount) = 0;

    protected:

        const Core::Acceleration_Structures::UpperLevelBVH& m_bvh;
        Core::MaterialManager& m_material_manager;
        std::vector<Scene::Light> m_lights;
    };


    class Monte_Carlo_Integrator : public Integrator
    {
    public:

        Monte_Carlo_Integrator(const Core::Acceleration_Structures::UpperLevelBVH&,  Core::MaterialManager&, const std::vector<Scene::Light> &light_bounds, std::shared_ptr<Core::ImageCube>& skybox,const uint64_t seed);

        virtual glm::vec4 integrate_ray(const Core::Ray& ray, const uint32_t maxDepth, const uint32_t rayCount) final;

    private:

        Render::Sample generate_next_event(const Core::Acceleration_Structures::InterpolatedVertex& frag, Core::Ray& ray);

        void trace_ray(const Core::Acceleration_Structures::InterpolatedVertex& frag, Core::Ray& ray, const uint32_t depth);

        bool weighted_random_ray_type(const Core::EvaluatedMaterial& mat);

        std::mt19937 mGenerator;
        std::uniform_real_distribution<float> mDistribution;

        Core::Rand::Hammersley_Generator m_hammersley_generator;

        std::shared_ptr<Core::ImageCube> m_skybox;
    };
}

#endif
