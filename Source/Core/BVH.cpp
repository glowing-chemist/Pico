#include "BVH.hpp"

#include <limits>
#include <algorithm>
#include <array>

#include "OctTree.hpp"

#include "Core/Asserts.hpp"
#include "LowerLevelBVH.hpp"

namespace Core
{

    namespace Acceleration_Structures
    {

        template<typename T, uint32_t C>
        bool BVH<T, C>::get_first_intersection(const Ray& ray, Acceleration_Structures::InterpolatedVertex &val) const
        {
            float intersection_distance = INFINITY;

        #ifdef BVH_PROFILE
            uint32_t tests_performed = 0;
            get_closest_intersections(ray, get_node(m_root), val, intersection_distance, tests_performed);
            PICO_LOG("intersection tests performed %d\n", tests_performed);
        #else
            get_closest_intersections(ray, get_node(m_root), val, intersection_distance);
        #endif

            return intersection_distance != INFINITY;
        }

        template<typename T, uint32_t C>
        void    BVH<T, C>::get_closest_intersections(const Ray& ray, const typename BVH<T, C>::Node& node, Core::Acceleration_Structures::InterpolatedVertex& intersection, float& intersection_distance
        #ifdef BVH_PROFILE
                                                   ,uint32_t &tests_performed
        #endif
                                                   ) const
        {
        #ifdef BVH_PROFILE
            ++tests_performed;
        #endif
            if(node.m_bounding_box.intersection_distance(ray) < intersection_distance)
            {
                for(auto& value : node.m_values)
                {
        // If we intersect the bounds then invoke the intersector to check if the contained entity is also intersected
        #ifdef BVH_PROFILE
                    ++tests_performed;
        #endif
                    if(value.m_bounds.intersection_distance(ray) < intersection_distance)
                    {
                        float fine_intersected_distance = INFINITY;
                        Acceleration_Structures::InterpolatedVertex vertex;
        #ifdef BVH_PROFILE
                        ++tests_performed;
        #endif
                        if(m_intersector->intersects(ray, value.m_value, fine_intersected_distance, vertex) && fine_intersected_distance < intersection_distance)
                        {
                            intersection = vertex;
                            intersection_distance = fine_intersected_distance;
                        }
                    }
                }

                for (const auto& childIndex : node.m_children)
                {
                    if (childIndex != kInvalidNodeIndex)
                    {
                        const Node& childNode = get_node(childIndex);
        #ifdef BVH_PROFILE
                        get_closest_intersections(ray, childNode, intersection, intersection_distance, tests_performed);
        #else
                        get_closest_intersections(ray, childNode, intersection, intersection_distance);
        #endif
                    }
                }
            }
        }

    }

}

// Explicitly instantiate
#include "Core/UpperLevelBVH.hpp"

template
    class Core::Acceleration_Structures::OctTreeFactory<const Core::Acceleration_Structures::UpperLevelBVH::Entry*>;

template
    class Core::Acceleration_Structures::BVH<const Core::Acceleration_Structures::UpperLevelBVH::Entry*, 8>;

template
    class Core::Acceleration_Structures::OctTreeFactory<uint32_t>;

template
    class Core::Acceleration_Structures::BVH<uint32_t, 8>;




