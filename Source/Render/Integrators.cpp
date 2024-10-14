#include "Integrators.hpp"
#include "Render/SolidAngle.hpp"
#include "Core/vectorUtils.hpp"
#include "Core/Image.hpp"
#include "Core/Asserts.hpp"
#include "glm/ext.hpp"

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

    bool Monte_Carlo_Integrator::sample_direct_lighting(const Core::Acceleration_Structures::InterpolatedVertex& frag, glm::vec3& radiance, float& pdf)
    {
        const auto bsrdf_type = frag.m_bsrdf->get_type();
        if(bsrdf_type != Render::BSRDF_Type::kBTDF && bsrdf_type != Render::BSRDF_Type::kLight)
        {
            // account for a special cased sunlight
            const uint32_t light_count = m_lights.size() + (m_sky_desc.m_use_sun ? 1 : 0);
            uint32_t light_index = mDistribution(mGenerator) * light_count;

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
                    const float cos_theta = glm::saturate<float, glm::packed_highp>(glm::dot(frag.mNormal, -m_sky_desc.m_sun_direction));

                    pdf = (1.0f / light_count) * cos_theta;
                    radiance = m_sky_desc.m_sun_colour;
                    return true;
                }

                return false;
            }

            glm::vec3 sample_position;
            float selected_solid_angle;

            const glm::vec3 light_space_pos = m_lights[light_index].m_inverse_transform * frag.mPosition;
            const glm::vec3 light_space_normal = glm::normalize(glm::mat3x3(m_lights[light_index].m_inverse_transform) * frag.mNormal);
            auto& geometrty = m_lights[light_index].m_geometry;

            const bool found_sample = geometrty->sample_geometry(m_hammersley_generator, light_space_pos, light_space_normal, sample_position, selected_solid_angle);
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

                        const float cos_theta = glm::dot(frag.mNormal, to_light);

                        pdf = (1.0f / light_count) * cos_theta;
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

    glm::vec3 Monte_Carlo_Integrator::integrate_ray(const Scene::Camera& camera, const glm::uvec2& pixel, const uint32_t maxDepth, const uint32_t rayCount)
    {
        m_max_depth = maxDepth;
        Core::Ray ray = camera.generate_ray(glm::vec2(0.5f, 0.5f), pixel);

        Core::Acceleration_Structures::InterpolatedVertex vertex;
        if(m_bvh.get_closest_intersection(ray, &vertex))
        {
            //return glm::vec4(vertex.mNormal * 0.5f + 0.5f, 1.0f);
            //return glm::vec4(m_material_manager.evaluate_material(vertex.m_bsrdf->get_material_id(), vertex.mUV).diffuse, 1.0f);

            glm::vec3 result{0.0f, 0.0f, 0.0f};
            for(uint32_t i_ray = 0; i_ray < rayCount; ++i_ray)
            {
                ray = camera.generate_ray(m_hammersley_generator.next(), pixel);
                trace_ray(vertex, ray, 0);

                result += ray.m_payload;
            }

            result /= rayCount;

            if(glm::any(glm::isinf(result)) || glm::any(glm::isnan(result)))
                result = glm::vec4(0.0f);

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
        if(sample_direct_lighting(frag, direct_radiance, direct_pdf))
        {
            ray.m_payload += ray.m_throughput * direct_pdf * direct_radiance * sample.energy;
        }

        // Sample does not contribute, so early out.
        if(sample.P == 0.0f)
        {
            return;
        }
        PICO_ASSERT_VALID(sample.L);
        PICO_ASSERT_NORMALISED(sample.L);
        ray.m_throughput *= sample.P * sample.energy;

        // kill off random rays here for russian roulette sampling.
        if(depth > 2)
        {
            const float inverse_kill_rate = std::max(std::max(ray.m_throughput.x, ray.m_throughput.y), ray.m_throughput.y);
            if(mDistribution(mGenerator) > inverse_kill_rate)
            {
                return;
            }

            ray.m_throughput *= 1.0f / inverse_kill_rate;
        }

        ray.mOrigin = frag.mPosition + glm::vec4(0.01f * sample.L, 0.0f);
        ray.mDirection = sample.L;

        Core::Acceleration_Structures::InterpolatedVertex intersection;
        if(m_bvh.get_closest_intersection(ray, &intersection))
        {
            trace_ray(intersection, ray, depth + 1);
        }
        else
            ray.m_payload += ray.m_throughput * glm::vec3(m_sky_desc.m_sky_box->sample4(sample.L));
    }

    bool Monte_Carlo_Integrator::weighted_random_ray_type(const Core::EvaluatedMaterial& mat)
    {
        const float diffuse_weight = mat.diffuse_magnitude();
        const float specular_weight = mat.specular_magnitude();

        const float rand = mDistribution(mGenerator);

        return rand <= (specular_weight / (specular_weight + diffuse_weight));
    }

}
