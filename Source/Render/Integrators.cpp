#include "Integrators.hpp"
#include "Core/LowerLevelBVH.hpp"
#include "Core/MaterialManager.hpp"
#include "Render/SolidAngle.hpp"
#include "Core/vectorUtils.hpp"
#include "Core/Image.hpp"
#include "Core/Asserts.hpp"
#include "glm/ext.hpp"
#include "glm/geometric.hpp"

namespace 
{
    float direct_lighting_pdf(const Core::Acceleration_Structures::InterpolatedVertex& frag, const glm::vec3& wi, const glm::vec3& wo, const Core::EvaluatedMaterial& mat)
    {
        const glm::mat3x3 tangent_transform = Core::TangentSpace::construct_world_to_tangent_transform(wi, frag.mNormal);
    
        const glm::vec3 tangent_wo = tangent_transform * wo;
        glm::vec3 H = glm::normalize(wi + wo);
        H = tangent_transform * H;

        return frag.m_bsrdf->pdf(tangent_wo, H, mat.roughness, mat.get_reflectance());
    }
}

namespace Render
{

    Integrator::Integrator(const Core::Acceleration_Structures::UpperLevelBVH& bvh, Core::MaterialManager& material_manager, const std::vector<Scene::Light>& lights) :
        m_bvh{bvh},
        m_material_manager{material_manager},
        m_lights(lights)
    {
    }


    Monte_Carlo_Integrator::Monte_Carlo_Integrator(const Core::Acceleration_Structures::UpperLevelBVH& bvh,  Core::MaterialManager& material_manager, const std::vector<Scene::Light>& lights,
                                                   const Scene::Sun &sun, const uint64_t seed) :
        Integrator(bvh, material_manager, lights),
        mGenerator{seed},
        mDistribution(0.0f, 1.0f),
        m_hammersley_generator(seed),
        m_sky_desc{sun}
    {
    }

    bool Monte_Carlo_Integrator::sample_direct_lighting(const Core::Acceleration_Structures::InterpolatedVertex& frag, const glm::vec3& wi, glm::vec3& radiance, float& pdf)
    {
        const auto bsrdf_type = frag.m_bsrdf->get_type();
        if(bsrdf_type != Render::BSRDF_Type::kBTDF && bsrdf_type != Render::BSRDF_Type::kLight)
        {

            const Core::EvaluatedMaterial mat = m_material_manager.evaluate_material(frag.m_bsrdf->get_material_id(), frag.mUV);

            // account for a special cased sunlight
            uint32_t light_count = m_lights.size() + (m_sky_desc.m_use_sun ? 1 : 0);
            const uint32_t light_index = mDistribution(mGenerator) * light_count;
            if(light_index == light_count)
                light_count--;

            // handle sunlight contribution
            if(light_index >= (light_count - 1) && m_sky_desc.m_use_sun)
            {
                Core::Ray direct_lighting_ray{};
                direct_lighting_ray.mDirection = -m_sky_desc.m_sun_direction;
                direct_lighting_ray.mOrigin = frag.mPosition + glm::vec4((0.01f * direct_lighting_ray.mDirection), 0.0f);
                direct_lighting_ray.mLenght = 10000.0f;

                Core::Acceleration_Structures::InterpolatedVertex point_hit;
                if(!m_bvh.get_closest_intersection(direct_lighting_ray, &point_hit))
                {
                    pdf = direct_lighting_pdf(frag, wi, -m_sky_desc.m_sun_direction, mat);
                    radiance = m_sky_desc.m_sun_colour;
                    return true;
                }

                return false;
            }

            glm::vec3 sample_position;
            float selected_pdf;

            const auto& geometrty = m_lights[light_index].m_geometry;

            const bool found_sample = geometrty->sample_geometry(m_hammersley_generator, sample_position, selected_pdf);
            if(found_sample)
            {
                sample_position = m_lights[light_index].m_transform * glm::vec4(sample_position, 1.0f);
            }

            if(!found_sample)
            {
                return false;
            }
            else // Try to find a light.
            {
                glm::vec3 to_light = glm::normalize(sample_position - glm::vec3(frag.mPosition));

                if(glm::dot(to_light, frag.mNormal) < 0.0f)
                    return false;

                Core::Ray direct_lighting_ray{};
                direct_lighting_ray.mDirection = to_light;
                direct_lighting_ray.mOrigin = frag.mPosition + glm::vec4((0.01f * to_light), 0.0f);
                direct_lighting_ray.mLenght = 10000.0f;

                Core::Acceleration_Structures::InterpolatedVertex point_hit;
                if(m_bvh.get_closest_intersection(direct_lighting_ray, &point_hit))
                {
                    if(point_hit.m_bsrdf->get_type() == Render::BSRDF_Type::kLight)
                    {
                        const Core::EvaluatedMaterial light_material = m_material_manager.evaluate_material(point_hit.m_bsrdf->get_material_id(), point_hit.mUV);

                        pdf =  direct_lighting_pdf(frag, wi, to_light, mat);
                        radiance = light_material.emissive;

                        return true;
                    }
                }

                return false;
            }

        }
        else
        {
            return false;
        }
    }

    glm::vec3 Monte_Carlo_Integrator::integrate_ray(const Scene::Camera& camera, const glm::uvec2& pixel, const uint32_t maxDepth)
    {
        m_max_depth = maxDepth;
        Core::Ray ray = camera.generate_ray(m_hammersley_generator.next(), pixel);

        Core::Acceleration_Structures::InterpolatedVertex vertex;
        if(m_bvh.get_closest_intersection(ray, &vertex))
        {
            //return glm::vec4(vertex.mNormal * 0.5f + 0.5f, 1.0f);
            //return glm::vec4(m_material_manager.evaluate_material(vertex.m_bsrdf->get_material_id(), vertex.mUV).diffuse, 1.0f);

            trace_ray(vertex, ray, 0);

            glm::vec3 result = ray.m_payload;

            if(glm::any(glm::isinf(result)) || glm::any(glm::isnan(result)))
                result = glm::vec3(1.0f, 0.4, 0.7);

            return result;
        }
        else
        {
            return m_sky_desc.m_sky_box->sample4(ray.mDirection);
        }
}


    void Monte_Carlo_Integrator::trace_ray(const Core::Acceleration_Structures::InterpolatedVertex& frag, Core::Ray& ray, const uint32_t depth)
    {
        if(depth == m_max_depth)
        {
            return;
        }

        Sample sample = frag.m_bsrdf->sample(m_hammersley_generator, frag, ray);
        if(frag.m_bsrdf->get_type() == BSRDF_Type::kLight)
        {
            ray.m_payload += ray.m_throughput * sample.energy;
        }

        // Add direct lighting contribution(s)
        glm::vec3 direct_radiance;
        float direct_pdf;
        if(sample_direct_lighting(frag, -ray.mDirection, direct_radiance, direct_pdf))
        {
            ray.m_payload += ray.m_throughput * direct_pdf * direct_radiance * sample.energy;
        }

        // Sample does not contribute, so early out.
        if(sample.P == 0.0f)
        {
            return;
        }

        // kill off random rays here for russian roulette sampling.
        {
            const float inverse_kill_rate = std::max(std::max(ray.m_throughput.x, ray.m_throughput.y), ray.m_throughput.z);
            if(mDistribution(mGenerator) > inverse_kill_rate)
            {
                return;
            }

            ray.m_throughput *= 1.0f / inverse_kill_rate;
        }

        PICO_ASSERT_VALID(sample.L);
        PICO_ASSERT_NORMALISED(sample.L);
        ray.m_throughput *= sample.P * sample.energy;

        ray.mOrigin = frag.mPosition + glm::vec4(0.01f * (ray.inside_geometry() ? -frag.mNormal : frag.mNormal), 0.0f);
        ray.mDirection = sample.L;

        Core::Acceleration_Structures::InterpolatedVertex intersection;
        if(m_bvh.get_closest_intersection(ray, &intersection))
        {
            trace_ray(intersection, ray, depth + 1);
        }
        else
            ray.m_payload += ray.m_throughput * glm::vec3(m_sky_desc.m_sky_box->sample4(sample.L));
    }
}
