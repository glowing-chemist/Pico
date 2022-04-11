#include "Integrators.hpp"
#include "Render/PBR.hpp"
#include "Core/Image.hpp"
#include "Core/Asserts.hpp"

namespace Render
{

    Integrator::Integrator(const Core::BVH::UpperLevelBVH& bvh, Core::MaterialManager& material_manager, const std::vector<Scene::Light>& lights) :
        m_bvh{bvh},
        m_material_manager{material_manager},
        m_lights(lights)
    {
    }


    Monte_Carlo_Integrator::Monte_Carlo_Integrator(const Core::BVH::UpperLevelBVH& bvh,  Core::MaterialManager& material_manager, const std::vector<Scene::Light>& lights,
                                                   std::shared_ptr<Core::ImageCube>& skybox, const uint64_t seed) :
        Integrator(bvh, material_manager, lights),
        mGenerator{seed},
        mDistribution(0.0f, 1.0f),
        m_hammersley_generator(seed),
        m_skybox{skybox}
    {
    }

    Render::Sample Monte_Carlo_Integrator::generate_next_diffuse_event(const Core::BVH::InterpolatedVertex& frag, const glm::vec3& V)
    {
        if(mDistribution(mGenerator) > 0.25f)
        {
            // calculate aproximate solid angles of all lights above the points hemisphere
            // and pick one at random weighted by it's solid angle.
            float total_solid_angle = 0.0f;

            std::vector<float> sample_solid_angle{};
            sample_solid_angle.reserve(m_lights.size());
            std::vector<glm::vec3> sample_positions{};
            sample_positions.reserve(m_lights.size());

            for(uint32_t i_light = 0; i_light < m_lights.size(); ++i_light)
            {
                float solid_angle;
                glm::vec3 sample_pos;
                const glm::vec3 light_space_pos = m_lights[i_light].m_inverse_transform * frag.mPosition;
                const glm::vec3 light_space_normal = glm::normalize(glm::mat3x3(m_lights[i_light].m_inverse_transform) * frag.mNormal);
                auto& geometrty = m_lights[i_light].m_geometry;
                const bool found_sample = geometrty->sample_geometry(m_hammersley_generator, light_space_pos, light_space_normal, sample_pos, solid_angle);
                sample_pos = m_lights[i_light].m_transform * glm::vec4(sample_pos, 1.0f);

                if(found_sample)
                {
                    total_solid_angle += solid_angle;
                    sample_solid_angle.push_back(solid_angle);
                    sample_positions.push_back(sample_pos);
                }
            }

            const uint32_t selected_light_index = Core::Rand::choose(mDistribution(mGenerator), sample_solid_angle, total_solid_angle);

            // if we fail to find any lights generate a "random" diffuse sample.
            if(selected_light_index == UINT_MAX)
                return frag.m_bsrdf->sample(m_hammersley_generator, frag, V);
            else // Try to find a light.
            {
                const glm::vec3& sample_pos = sample_positions[selected_light_index];
                glm::vec3 to_light = glm::normalize(sample_pos - glm::vec3(frag.mPosition));
                const float NdotV = std::abs(glm::dot(frag.mNormal, V));
                const float NdotL = std::abs(glm::dot(frag.mNormal, to_light));
                const glm::vec3 H = glm::normalize(to_light + V);
                const float LdotH = std::abs(glm::dot(to_light, H));

                const auto mat_id = frag.m_bsrdf->get_material_id();
                const glm::vec3 diffuse_color = m_material_manager.evaluate_material(mat_id, frag.mUV).diffuse;

                return Render::Sample{to_light, (sample_solid_angle[selected_light_index] / total_solid_angle) * 0.75f, diffuse_color * glm::vec3(Render::disney_diffuse(NdotV, NdotL, LdotH, 1.0f))};
            }
        }
        else
        {
            Render::Sample samp = frag.m_bsrdf->sample(m_hammersley_generator, frag, V);
            samp.P *= 0.25f;

            return samp;
        }
    }

    Render::Sample Monte_Carlo_Integrator::generate_next_specular_event(const Core::BVH::InterpolatedVertex& frag, const glm::vec3 &V)
    {
        return frag.m_bsrdf->sample(m_hammersley_generator, frag, V);
    }

    glm::vec4 Monte_Carlo_Integrator::integrate_ray(const Core::Ray& ray, const uint32_t maxDepth, const uint32_t rayCount)
    {
        Core::BVH::InterpolatedVertex vertex;
        if(m_bvh.get_closest_intersection(ray, &vertex))
        {
            //return m_material_manager.evaluate_material(vertex.mMaterialID, vertex.mUV).diffuse;

            glm::vec4 result{0.0f, 0.0f, 0.0f, 0.0f};
            for(uint32_t i_ray = 0; i_ray < rayCount; ++i_ray)
            {
                Core::Ray new_ray = ray;
                new_ray.m_payload = glm::vec4(1.0f);
                new_ray.m_weight = 0.0f;
                trace_ray(vertex, new_ray, maxDepth);

                result += new_ray.m_payload / new_ray.m_weight;
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


    void Monte_Carlo_Integrator::trace_ray(const Core::BVH::InterpolatedVertex& frag, Core::Ray& ray, const uint32_t depth)
    {
        if(depth == 0)
        {
            ray.m_payload = glm::vec4(0.0f, 0.f, 0.0f, 1.0f);
            ray.m_weight = 1.0f;
            return;
        }

        const glm::vec3 V = glm::normalize(glm::vec3(ray.mOrigin - frag.mPosition));

        if(frag.m_bsrdf->get_type() == BSRDF_Type::kLight)
        {
            Sample samp = frag.m_bsrdf->sample(m_hammersley_generator, frag, V);
            ray.m_payload *= glm::vec4(samp.energy, 1.0f);
            ray.m_weight = ray.m_weight == 0.0f ? 1.0f : ray.m_weight;
            return;
        }

        Render::Sample sample = generate_next_diffuse_event(frag, V);
        PICO_ASSERT_VALID(sample.L);
        ray.m_weight += sample.P;

        const glm::vec4 diffuse_factor = glm::vec4(sample.energy, 1.0f);

        ray.mOrigin = frag.mPosition + glm::vec4(0.01f * sample.L, 0.0f);
        ray.mDirection = sample.L;

        Core::BVH::InterpolatedVertex intersection;
        if(m_bvh.get_closest_intersection(ray, &intersection))
        {
            ray.m_payload *= diffuse_factor;

            trace_ray(intersection, ray, depth - 1);
        }
        else
            ray.m_payload *= diffuse_factor * m_skybox->sample4(sample.L);
    }

    bool Monte_Carlo_Integrator::weighted_random_ray_type(const Core::EvaluatedMaterial& mat)
    {
        const float diffuse_weight = mat.diffuse_magnitude();
        const float specular_weight = mat.specular_magnitude();

        const float rand = mDistribution(mGenerator);

        return rand <= (specular_weight / (specular_weight + diffuse_weight));
    }
}
