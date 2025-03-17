#include "BVH.hpp"

#include <limits>
#include <algorithm>
#include <array>

#include "Core/Asserts.hpp"
#include "LowerLevelBVH.hpp"

namespace Core
{

    namespace Acceleration_Structures
    {

        template<typename T, uint32_t C>
        bool BVH<T, C>::get_first_intersection(Ray& ray, Acceleration_Structures::InterpolatedVertex &val) const
        {
            float intersection_distance = INFINITY;
            ray.mInverseDirection = glm::vec3(1.0f, 1.0f, 1.0f) / ray.mDirection;

            get_closest_intersections(ray, get_node(m_root), val, intersection_distance);

            return intersection_distance != INFINITY;
        }

        template<typename T, uint32_t C>
        void    BVH<T, C>::get_closest_intersections(const Ray& ray, const typename BVH<T, C>::Node& node, Core::Acceleration_Structures::InterpolatedVertex& intersection, float& intersection_distance) const
        {
            if(node.m_bounding_box.intersection_distance(ray) < intersection_distance)
            {
                if (node.is_leaf())
                {
                    for (auto& value : node.m_values)
                    {
                        // If we intersect the bounds then invoke the intersector to check if the contained entity is also intersected
                        //if(value.m_bounds.intersection_distance(ray) < intersection_distance)
                        {
                            float fine_intersected_distance = INFINITY;
                            Acceleration_Structures::InterpolatedVertex vertex;

                            if (m_intersector->intersects(ray, value.m_value, fine_intersected_distance, vertex) && fine_intersected_distance < intersection_distance)
                            {
                                intersection = vertex;
                                intersection_distance = fine_intersected_distance;
                            }
                        }
                    }
                }
                else
                {
                    for (NodeIndex child_index : node.m_children)
                    {
                        if (child_index != kInvalidNodeIndex)
                        {
                            const Node& childNode = get_node(child_index);

                            get_closest_intersections(ray, childNode, intersection, intersection_distance);
                        }
                    }
                }
            }
        }

        template<typename T>
        BVH<T, 2>* BVHFactory<T>::generate_BVH()
        {
            const NodeIndex root_node = split_primitives(m_bounding_boxes.begin(), m_bounding_boxes.end(), m_max_depth);

            return new BVH<T, 2>(root_node, m_node_storage, std::move(m_intersector));
        }

        template<typename T>
        NodeIndex BVHFactory<T>::split_primitives(BVHPartitionScheme<T>::ITERATOR start, BVHPartitionScheme<T>::ITERATOR end, uint32_t depth)
        {
            if(start == end)
                return kInvalidNodeIndex;

            AABB bounds(glm::vec4(INFINITY, INFINITY, INFINITY, INFINITY), -glm::vec4(INFINITY, INFINITY, INFINITY, INFINITY));
            for(auto bounds_it = start; bounds_it != end; ++bounds_it)
            {
                bounds.union_of(bounds_it->m_bounds);
            }

            typename BVH<T, 2>::Node newNode{};
            newNode.m_bounding_box = bounds;

            const auto pivot = m_partition_scheme->partition(start, end);

            if(depth == 0 || pivot == end || std::distance(start, end) == 2 )
            {
                for(auto primitives_it = start; primitives_it != end; ++primitives_it)
                {
                    newNode.m_values.push_back(*primitives_it);
                }

                newNode.m_children[0] = kInvalidNodeIndex;
                newNode.m_children[1] = kInvalidNodeIndex;

                return add_node(newNode);
            }

            newNode.m_children[0] = split_primitives(start, pivot, depth - 1);
            newNode.m_children[1] = split_primitives(pivot, end, depth - 1);

            return add_node(newNode);
        }

        template<typename T>
        Centroid_parition_Scheme<T>::ITERATOR Centroid_parition_Scheme<T>::partition(ITERATOR start, ITERATOR end)
        {
            // compute the bounds to find the longest edge.
            AABB bounds(glm::vec4(INFINITY, INFINITY, INFINITY, INFINITY), -glm::vec4(INFINITY, INFINITY, INFINITY, INFINITY));
            for(auto bounds_it = start; bounds_it != end; ++bounds_it)
            {
                bounds.add_point(bounds_it->m_bounds.get_central_point());
            }

            const glm::vec3 side_lengths = bounds.get_side_lengths();
            const uint32_t max_component = maximum_component_index(side_lengths);
            const float central_point =  bounds.get_central_point()[max_component];

            return std::partition(start, end, [&](const auto& it){ return it.m_bounds.get_central_point()[max_component] < central_point; });
        }

        template<typename T>
        SAH_parition_Scheme<T>::ITERATOR SAH_parition_Scheme<T>::partition(ITERATOR start, ITERATOR end)
        {
            if(start == end)
                return end;

            AABB bounds(glm::vec4(INFINITY, INFINITY, INFINITY, INFINITY), -glm::vec4(INFINITY, INFINITY, INFINITY, INFINITY));
            for(auto bounds_it = start; bounds_it != end; ++bounds_it)
            {
                bounds.add_point(bounds_it->m_bounds.get_central_point());
            }

            const glm::vec3 side_lengths = bounds.get_side_lengths();
            const uint32_t max_component = maximum_component_index(side_lengths);

            constexpr uint32_t bucket_count = 12;
            Bucket buckets[bucket_count];
            // Init the buckets.
            for(uint32_t i = 0; i < bucket_count; ++i)
            {
                buckets[i] = {0, AABB{glm::vec4(INFINITY, INFINITY, INFINITY, INFINITY), -glm::vec4(INFINITY, INFINITY, INFINITY, INFINITY)}};
            }

            for(auto i = start; i < end; ++i)
            {
                uint32_t b = bucket_count * bounds.get_offset(i->m_bounds.get_central_point())[max_component];
                if (b == bucket_count) b = bucket_count - 1;
                buckets[b].mCount++;
                buckets[b].mBounds.union_of(i->m_bounds);
            }

            const float leaf_cost = std::distance(start, end);
            float cost[bucket_count - 1];
            for (uint32_t i = 0; i < bucket_count - 1; ++i)
            {
                AABB b0{glm::vec4(INFINITY, INFINITY, INFINITY, INFINITY), -glm::vec4(INFINITY, INFINITY, INFINITY, INFINITY)};
                AABB b1{glm::vec4(INFINITY, INFINITY, INFINITY, INFINITY), -glm::vec4(INFINITY, INFINITY, INFINITY, INFINITY)};
                uint32_t count0 = 0, count1 = 0;
                for (uint32_t j = 0; j <= i; ++j)
                {
                    b0.union_of(buckets[j].mBounds);
                    count0 += buckets[j].mCount;
                }
                for (uint32_t j = i+1; j < bucket_count; ++j)
                {
                    b1.union_of(buckets[j].mBounds);
                    count1 += buckets[j].mCount;
                }
                cost[i] = .125f + (count0 * b0.get_surface_area() +
                                   count1 * b1.get_surface_area()) / (leaf_cost * bounds.get_surface_area());
            }

            auto min_element = std::min_element(&cost[0], &cost[bucket_count - 1]);
            const float minCost = *min_element;
            const uint32_t minCostSplitBucket = std::distance(&cost[0], min_element);

            if (leaf_cost > 2 || minCost < leaf_cost)
            {
                return std::partition(start,
                                      end,
                                      [&](const auto pi)
                                      {
                                        uint32_t b = bucket_count * bounds.get_offset(pi.m_bounds.get_central_point())[max_component];
                                        if (b == bucket_count) b = bucket_count - 1;
                                        return b <= minCostSplitBucket;
                                      });
            }
            else
            {
                return end;
            }
        }

    }

}

// Explicitly instantiate
#include "Core/UpperLevelBVH.hpp"

template
    class Core::Acceleration_Structures::BVH<const Core::Acceleration_Structures::UpperLevelBVH::Entry*, 2>;

template
    class Core::Acceleration_Structures::BVH<uint32_t, 2>;

template
    class Core::Acceleration_Structures::BVHFactory<uint32_t>;

template
    class Core::Acceleration_Structures::BVHFactory<const Core::Acceleration_Structures::UpperLevelBVH::Entry*>;

template
    class Core::Acceleration_Structures::Centroid_parition_Scheme<uint32_t>;

template
    class Core::Acceleration_Structures::SAH_parition_Scheme<uint32_t>;

template
    class Core::Acceleration_Structures::SAH_parition_Scheme<const Core::Acceleration_Structures::UpperLevelBVH::Entry*>;




