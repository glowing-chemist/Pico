#include "Integrators.hpp"
#include "Core/LowerLevelBVH.hpp"
#include "Core/MaterialManager.hpp"
#include "Core/Image.hpp"
#include "Core/Asserts.hpp"
#include "glm/ext.hpp"
#include "glm/geometric.hpp"

#include <algorithm>
#include <cmath>

namespace 
{
    glm::vec3 diffuse_response(const Core::Acceleration_Structures::InterpolatedVertex& frag, const glm::vec3& wo, const Core::EvaluatedMaterial& mat)
    {
        const float cosine = std::max(0.0f, glm::dot(frag.mNormal, wo));
        return mat.diffuse * (cosine / float(M_PI));
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

    bool Monte_Carlo_Integrator::sample_direct_lighting(const Core::Acceleration_Structures::InterpolatedVertex& frag, glm::vec3& contribution)
    {
        const auto bsrdf_type = frag.m_bsrdf->get_type();
        if(bsrdf_type == Render::BSRDF_Type::kDiffuse_BRDF ||
           bsrdf_type == Render::BSRDF_Type::kDielectric_BRDF)
        {
            const Core::EvaluatedMaterial mat = m_material_manager.evaluate_material(frag.m_bsrdf->get_material_id(), frag.mUV);

            const uint32_t light_count = static_cast<uint32_t>(m_lights.size()) + (m_sky_desc.m_use_sun ? 1u : 0u);
            if(light_count == 0)
                return false;

            const float light_selection_pdf = 1.0f / static_cast<float>(light_count);
            const uint32_t light_index = mDistribution(mGenerator) * light_count;

            if(m_sky_desc.m_use_sun && light_index == static_cast<uint32_t>(m_lights.size()))
            {
                Core::Ray direct_lighting_ray{};
                direct_lighting_ray.mDirection = -m_sky_desc.m_sun_direction;
                direct_lighting_ray.mOrigin = frag.mPosition + glm::vec4((0.01f * direct_lighting_ray.mDirection), 0.0f);
                direct_lighting_ray.mLenght = 10000.0f;

                Core::Acceleration_Structures::InterpolatedVertex point_hit;
                if(!m_bvh.get_closest_intersection(direct_lighting_ray, &point_hit))
                {
                    contribution = diffuse_response(frag, -m_sky_desc.m_sun_direction, mat) *
                                   m_sky_desc.m_sun_colour / light_selection_pdf;
                    return true;
                }

                return false;
            }

            glm::vec3 sample_position;
            float area_pdf;

            const auto& light = m_lights[light_index];

            const bool found_sample = light.m_geometry->sample_geometry(m_hammersley_generator, sample_position, area_pdf);
            if(found_sample)
            {
                sample_position = light.m_transform * glm::vec4(sample_position, 1.0f);
            }

            if(!found_sample)
            {
                return false;
            }
            else // Try to find a light.
            {
                const glm::vec3 offset = sample_position - glm::vec3(frag.mPosition);
                const float distance_squared = glm::dot(offset, offset);
                if(distance_squared <= 1e-8f)
                    return false;

                const float distance = std::sqrt(distance_squared);
                const glm::vec3 to_light = offset / distance;

                if(glm::dot(to_light, frag.mNormal) < 0.0f)
                    return false;

                Core::Ray direct_lighting_ray{};
                direct_lighting_ray.mDirection = to_light;
                direct_lighting_ray.mOrigin = frag.mPosition + glm::vec4((0.01f * to_light), 0.0f);
                direct_lighting_ray.mLenght = distance;

                Core::Acceleration_Structures::InterpolatedVertex point_hit;
                if(m_bvh.get_closest_intersection(direct_lighting_ray, &point_hit))
                {
                    const float position_error = std::max(0.02f, distance * 1e-3f);
                    if(point_hit.m_bsrdf->get_type() == Render::BSRDF_Type::kLight &&
                       glm::distance(glm::vec3(point_hit.mPosition), sample_position) <= position_error)
                    {
                        const Core::EvaluatedMaterial light_material = m_material_manager.evaluate_material(point_hit.m_bsrdf->get_material_id(), point_hit.mUV);

                        const float light_cosine = glm::dot(point_hit.mNormal, -to_light);
                        if(light_cosine <= 0.0f || area_pdf <= 0.0f)
                            return false;

                        const glm::mat3 light_transform(light.m_transform);
                        const float determinant = std::abs(glm::determinant(light_transform));
                        const float area_scale = determinant /
                            glm::length(glm::transpose(light_transform) * point_hit.mNormal);
                        if(area_scale <= 0.0f || !std::isfinite(area_scale))
                            return false;

                        const float area_pdf_world = area_pdf / area_scale;
                        const float solid_angle_pdf = light_selection_pdf * area_pdf_world *
                                                      distance_squared / light_cosine;
                        if(solid_angle_pdf <= 0.0f || !std::isfinite(solid_angle_pdf))
                            return false;

                        contribution = diffuse_response(frag, to_light, mat) * light_material.emissive /
                                       solid_angle_pdf;

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


    void Monte_Carlo_Integrator::trace_ray(const Core::Acceleration_Structures::InterpolatedVertex& frag, Core::Ray& ray,
                                           const uint32_t depth, const bool previous_bounce_was_diffuse)
    {
        if(frag.m_bsrdf->get_type() == BSRDF_Type::kLight)
        {
            // Diffuse paths to emitters are already accounted for by direct-light sampling.
            if(!previous_bounce_was_diffuse)
            {
                const Core::EvaluatedMaterial material =
                    m_material_manager.evaluate_material(frag.m_bsrdf->get_material_id(), frag.mUV);
                ray.m_payload += ray.m_throughput * material.emissive;
            }
            return;
        }

        if(depth >= m_max_depth)
            return;

        Sample sample = frag.m_bsrdf->sample(m_hammersley_generator, frag, ray);

        // Add direct lighting contribution(s)
        glm::vec3 direct_contribution{};
        if(sample_direct_lighting(frag, direct_contribution))
        {
            ray.m_payload += ray.m_throughput * direct_contribution;
        }

        // Sample does not contribute, so early out.
        if(sample.P == 0.0f)
        {
            return;
        }

        // kill off random rays here for russian roulette sampling.
        {
            const float survival_probability = std::clamp(
                std::max(std::max(ray.m_throughput.x, ray.m_throughput.y), ray.m_throughput.z), 0.0f, 1.0f);
            if(survival_probability <= 0.0f || mDistribution(mGenerator) >= survival_probability)
            {
                return;
            }

            ray.m_throughput *= 1.0f / survival_probability;
        }

        PICO_ASSERT_VALID(sample.L);
        PICO_ASSERT_NORMALISED(sample.L);
        ray.m_throughput *= sample.energy;

        ray.mOrigin = frag.mPosition + glm::vec4(0.01f * (ray.inside_geometry() ? -frag.mNormal : frag.mNormal), 0.0f);
        ray.mDirection = sample.L;

        Core::Acceleration_Structures::InterpolatedVertex intersection;
        if(m_bvh.get_closest_intersection(ray, &intersection))
        {
            trace_ray(intersection, ray, depth + 1, sample.diffuse);
        }
        else
            ray.m_payload += ray.m_throughput * glm::vec3(m_sky_desc.m_sky_box->sample4(sample.L));
    }
}
