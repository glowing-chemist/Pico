#include "UpperLevelBVH.hpp"
#include "LowerLevelImplicitShapesBVH.hpp"
#include "LowerLevelMeshBVH.hpp"
#include "Core/Asserts.hpp"

#include <iterator>

namespace Core
{
    namespace Acceleration_Structures
    {
        bool UpperLevelBVH::get_closest_intersection(const Ray& ray, InterpolatedVertex* vertex) const
        {
            InterpolatedVertex intersected_vertex{};
            const bool found = m_acceleration_structure->get_first_intersection(ray, intersected_vertex);
            if(found)
                *vertex = intersected_vertex;

            return found;
        }

        void UpperLevelBVH::add_lower_level_bvh(LowerLevelBVH* bvh, const glm::mat4x4& transform, std::unique_ptr<Render::BSRDF>& bssrdf)
        {
            mLowerLevelBVHs.push_back(Entry{transform, glm::inverse(transform), bvh, std::move(bssrdf)});
        }

        void UpperLevelBVH::build()
        {
            std::vector<UPPER_ACCELERATION_STRUCTURE::BoundedValue> values;
            values.reserve(mLowerLevelBVHs.size());

            std::transform(mLowerLevelBVHs.begin(), mLowerLevelBVHs.end(), std::back_inserter(values), [](const auto& entry) -> UPPER_ACCELERATION_STRUCTURE::BoundedValue
            {
                return { entry.mBVH->get_bounds() * entry.mTransform, &entry };
            });

            glm::vec4 min(INFINITY, INFINITY, INFINITY, INFINITY);
            glm::vec4 max{-INFINITY, -INFINITY, -INFINITY, -INFINITY};
            for(const auto& entry : values)
            {
                min = Core::component_wise_min(min, entry.m_bounds.get_min());
                max = Core::component_wise_max(max, entry.m_bounds.get_max());
            }

            AABB scene_bounds(min, max);

#ifdef USE_OCTTREE
            m_acceleration_structure = OctTreeFactory<const Entry*>(scene_bounds, values)
                           .set_intersector(std::make_unique<lower_level_intersector>())
                           .generate_octTree();
#else
            m_acceleration_structure = BVHFactory<const Entry*>(scene_bounds, values)
                                           .set_intersector(std::make_unique<lower_level_intersector>())
                                           .set_parition_scheme(std::make_unique<SAH_parition_Scheme<const Entry*>>())
                                           .generate_BVH();
#endif
        }

        bool UpperLevelBVH::lower_level_intersector::intersects(const Ray& ray, const Entry* entry, float& intersect_distance, InterpolatedVertex& vertex) const
        {
            // Move the ray in to the local space of the lower level bvh.
            const Ray object_space_ray = Core::transform_ray(ray, entry->mInverseTransform);

            InterpolatedVertex found_vertex;
            if(entry->mBVH->calculate_intersection(object_space_ray, &found_vertex))
            {
                found_vertex.m_bsrdf = entry->m_material.get();

                // Bring vertex back to world space.
                found_vertex.mPosition = entry->mTransform * found_vertex.mPosition;
                found_vertex.mNormal   = glm::normalize(glm::mat3x3(entry->mTransform) * found_vertex.mNormal);

                intersect_distance = glm::length(found_vertex.mPosition - ray.mOrigin);
                vertex= found_vertex;

                return true;
            }

            return false;
        }
    }

}
