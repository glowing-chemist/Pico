#include "Integrators.hpp"
#include "Render/SolidAngle.hpp"
#include "Core/vectorUtils.hpp"
#include "Core/Image.hpp"
#include "Core/Asserts.hpp"

namespace Render
{

    Integrator::Integrator(const Core::Acceleration_Structures::UpperLevelBVH& bvh, Core::MaterialManager& material_manager, const std::vector<Scene::Light>& lights) :
        m_bvh{bvh},
        m_material_manager{material_manager},
        m_lights(lights)
    {
    }


    Monte_Carlo_Integrator::Monte_Carlo_Integrator(const Core::Acceleration_Structures::UpperLevelBVH& bvh,  Core::MaterialManager& material_manager, const std::vector<Scene::Light>& lights,
                                                   Core::ImageCube *skybox, const uint64_t seed) :
        Integrator(bvh, material_manager, lights),
        mGenerator{seed},
        mDistribution(0.0f, 1.0f),
        m_hammersley_generator(seed),
        m_skybox{skybox}
    {
    }

    Render::Sample Monte_Carlo_Integrator::generate_next_event(const Core::Acceleration_Structures::InterpolatedVertex& frag, Core::Ray& ray)
    {
        const auto bsrdf_type = frag.m_bsrdf->get_type();
        if(bsrdf_type == Render::BSRDF_Type::kDiffuse_BRDF || bsrdf_type == Render::BSRDF_Type::kDielectric_BRDF)
        {
            constexpr float random_sample_rate = 0.5f;
            if(mDistribution(mGenerator) <= random_sample_rate)
            {
                const glm::vec3 V = -ray.mDirection;
                PICO_ASSERT_NORMALISED(V);
                const glm::mat3x3 world_to_tangent_transform = Core::TangentSpace::construct_world_to_tangent_transform(V, frag.mNormal);
                const glm::vec3 view_tangent = glm::normalize(world_to_tangent_transform * V);

                // calculate aproximate solid angles of all lights above the points hemisphere
                // and pick one at random weighted by it's solid angle.
                float total_solid_angle = 0.0f;

                for(uint32_t i_light = 0; i_light < m_lights.size(); ++i_light)
                {
                    total_solid_angle += Render::solid_angle_from_bounds(m_lights[i_light].m_geometry->get_bounds(), frag.mPosition);
                }

                glm::vec3 sample_position;
                float selected_solid_angle;

                const float rand = mDistribution(mGenerator) * total_solid_angle;

                float accumilated_solid_angle = 0.0f;
                bool found_light = false;
                for(uint32_t i_light = 0; i_light < m_lights.size(); ++i_light)
                {
                    accumilated_solid_angle += Render::solid_angle_from_bounds(m_lights[i_light].m_geometry->get_bounds(), frag.mPosition);

                    if(rand <= accumilated_solid_angle)
                    {
                        const glm::vec3 light_space_pos = m_lights[i_light].m_inverse_transform * frag.mPosition;
                        const glm::vec3 light_space_normal = glm::normalize(glm::mat3x3(m_lights[i_light].m_inverse_transform) * frag.mNormal);
                        auto& geometrty = m_lights[i_light].m_geometry;
                        const bool found_sample = geometrty->sample_geometry(m_hammersley_generator, light_space_pos, light_space_normal, sample_position, selected_solid_angle);
                        if(found_sample)
                        {
                            sample_position = m_lights[i_light].m_transform * glm::vec4(sample_position, 1.0f);
                            found_light = true;
                            break;
                        }
                    }
                }

                // if we fail to find any lights generate a "random" diffuse sample.
                if(!found_light)
                {
                    Render::Sample samp = frag.m_bsrdf->sample(m_hammersley_generator, frag, ray);
                    samp.P *= random_sample_rate;

                    return samp;
                }
                else // Try to find a light.
                {
                    glm::vec3 to_light = glm::normalize(sample_position - glm::vec3(frag.mPosition));
                    glm::vec3 H = glm::normalize(to_light + V);
                    if(glm::any(glm::isnan(H)))
                        H = V;

                    const auto mat_id = frag.m_bsrdf->get_material_id();
                    const Core::EvaluatedMaterial material = m_material_manager.evaluate_material(mat_id, frag.mUV);

                    const glm::vec3 H_tangent = glm::normalize(world_to_tangent_transform * H);
                    PICO_ASSERT_VALID(H_tangent);
                    const float pdf = frag.m_bsrdf->pdf(view_tangent, H_tangent, material.roughness, material.get_reflectance());
                    const glm::vec3 energy = frag.m_bsrdf->energy(frag, view_tangent, H_tangent);

                    // TODO what is the correrct way to combine these pdfs here.
                    return Render::Sample{to_light, ((selected_solid_angle / total_solid_angle) * pdf) * random_sample_rate, energy};
                }
            }
            else
            {
                Render::Sample samp = frag.m_bsrdf->sample(m_hammersley_generator, frag, ray);
                samp.P *= (1.0f - random_sample_rate);

                return samp;
            }
        }
        else
        {
            return frag.m_bsrdf->sample(m_hammersley_generator, frag, ray);
        }
    }

    glm::vec3 Monte_Carlo_Integrator::integrate_ray(const Scene::Camera& camera, const glm::uvec2& pixel, const uint32_t maxDepth, const uint32_t rayCount)
    {
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
                trace_ray(vertex, ray, maxDepth);

                result += ray.m_payload;
            }

            result /= rayCount;

            if(glm::any(glm::isinf(result)) || glm::any(glm::isnan(result)))
                result = glm::vec4(0.0f);

            return result;
        }
        else
        {
            return m_skybox->sample4(ray.mDirection);
        }
}


    void Monte_Carlo_Integrator::trace_ray(const Core::Acceleration_Structures::InterpolatedVertex& frag, Core::Ray& ray, const uint32_t depth)
    {
        if(depth == 0)
        {
            return;
        }

        if(frag.m_bsrdf->get_type() == BSRDF_Type::kLight)
        {
            Sample samp = frag.m_bsrdf->sample(m_hammersley_generator, frag, ray);
            ray.m_payload += ray.m_throughput * samp.energy;
        }

        Render::Sample sample = generate_next_event(frag, ray);
        // Sample does not contribute, so early out.
        if(sample.P == 0.0f)
        {
            return;
        }
        PICO_ASSERT_VALID(sample.L);
        PICO_ASSERT_NORMALISED(sample.L);
        ray.m_throughput *= sample.P * sample.energy;

        ray.mOrigin = frag.mPosition + glm::vec4(0.01f * sample.L, 0.0f);
        ray.mDirection = sample.L;

        Core::Acceleration_Structures::InterpolatedVertex intersection;
        if(m_bvh.get_closest_intersection(ray, &intersection))
        {
            trace_ray(intersection, ray, depth - 1);
        }
        else
            ray.m_payload += ray.m_throughput * glm::vec3(m_skybox->sample4(sample.L));
    }

    bool Monte_Carlo_Integrator::weighted_random_ray_type(const Core::EvaluatedMaterial& mat)
    {
        const float diffuse_weight = mat.diffuse_magnitude();
        const float specular_weight = mat.specular_magnitude();

        const float rand = mDistribution(mGenerator);

        return rand <= (specular_weight / (specular_weight + diffuse_weight));
    }

}
