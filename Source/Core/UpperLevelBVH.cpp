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
            const auto rough_intersections = mOctTree.get_all_intersections(ray);

            float clostest_distance = INFINITY;
            InterpolatedVertex closest_intersection{};
            for(uint32_t i = 0; i < rough_intersections.size(); ++i)
            {
                const auto& intersection = rough_intersections[i];

                // Move the ray in to the local space of the lower level bvh.
                const Ray object_space_ray = Core::transform_ray(ray, intersection->mInverseTransform);

                InterpolatedVertex found_vertex;
                if(intersection->mBVH->calculate_intersection(object_space_ray, &found_vertex))
                {
                    const float intersection_distance = glm::length(found_vertex.mPosition - ray.mOrigin);

                    found_vertex.m_bsrdf = intersection->m_material;

                    // Bring vertex back to world space.
                    found_vertex.mPosition = intersection->mTransform * found_vertex.mPosition;
                    found_vertex.mNormal   = glm::normalize(glm::mat3x3(intersection->mTransform) * found_vertex.mNormal);

                    if(intersection_distance <= clostest_distance)
                    {
                        closest_intersection = found_vertex;
                        clostest_distance = intersection_distance;
                    }
                }
            }

            if(clostest_distance < INFINITY)
            {
                *vertex = closest_intersection;
                return true;
            }

            //PICO_ASSERT(rough_intersections.empty());
            return false;
        }

        void UpperLevelBVH::get_all_intersections(const Ray& ray, std::vector<InterpolatedVertex>& vertices) const
        {
            PICO_ASSERT_VALID(ray.mDirection);
            PICO_ASSERT_VALID(ray.mOrigin);

            const auto rough_intersections = mOctTree.get_all_intersections(ray);

            for(const auto& intersection : rough_intersections)
            {
                // Move the ray in to the local space of the lower level bvh.
                const Ray object_space_ray = Core::transform_ray(ray, intersection->mInverseTransform);

                InterpolatedVertex found_vertex;
                if(intersection->mBVH->calculate_intersection(object_space_ray, &found_vertex))
                {
                    found_vertex.m_bsrdf = intersection->m_material;

                    // Bring vertex back to world space.
                    found_vertex.mPosition = intersection->mTransform * found_vertex.mPosition;
                    found_vertex.mNormal   = glm::normalize(glm::mat3x3(intersection->mTransform) * found_vertex.mNormal);

                    PICO_ASSERT_VALID(found_vertex.mPosition);
                    PICO_ASSERT_VALID(found_vertex.mNormal);

                    vertices.push_back(found_vertex);
                }
            }
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
                min = Core::component_wise_min(min, entry.mBounds.get_min());
                max = Core::component_wise_max(max, entry.mBounds.get_max());
            }

            AABB scene_bounds(min, max);
            Core::OctTreeFactory<const Entry*> factory(scene_bounds, values);

            mOctTree = factory.generate_octTree();
        }
    }

}
