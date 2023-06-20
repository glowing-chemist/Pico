#include "UpperLevelBVH.hpp"
#include "LowerLevelImplicitShapesBVH.hpp"
#include "LowerLevelMeshBVH.hpp"
#include "Core/Asserts.hpp"

#include <iterator>

namespace Core
{
    namespace BVH
    {
        bool UpperLevelBVH::get_closest_intersection(const Ray& ray, InterpolatedVertex* vertex) const
        {
            InterpolatedVertex intersected_vertex{};
            const bool found = mOctTree.get_first_intersection(ray, intersected_vertex);
            if(found)
                *vertex = intersected_vertex;

            return found;
        }

        void UpperLevelBVH::add_lower_level_bvh(std::shared_ptr<LowerLevelBVH>& bvh, const glm::mat4x4& transform, std::shared_ptr<Render::BSRDF>& bssrdf)
        {
            mLowerLevelBVHs.push_back(Entry{transform, glm::inverse(transform), bvh, bssrdf});
        }

        void UpperLevelBVH::build()
        {
            std::vector<Core::OctTree<const Entry*>::BoundedValue> values;
            values.reserve(mLowerLevelBVHs.size());

            std::transform(mLowerLevelBVHs.begin(), mLowerLevelBVHs.end(), std::back_inserter(values), [](const auto& entry) -> Core::OctTree<const Entry*>::BoundedValue
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
            Core::OctTreeFactory<const Entry*> factory(scene_bounds, values, std::make_unique<lower_level_intersector>());

            mOctTree = factory.generate_octTree();
        }

        bool UpperLevelBVH::lower_level_intersector::intersects(const Ray& ray, const Entry* entry, float& intersect_distance, InterpolatedVertex& vertex)
        {
            // Move the ray in to the local space of the lower level bvh.
            const Ray object_space_ray = Core::transform_ray(ray, entry->mInverseTransform);

            InterpolatedVertex found_vertex;
            if(entry->mBVH->calculate_intersection(object_space_ray, &found_vertex))
            {
                found_vertex.m_bsrdf = entry->m_material;

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
