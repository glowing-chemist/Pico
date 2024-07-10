#include <limits>
#include <algorithm>
#include <array>
#include <iterator>
#include <numeric>
#include <map>
#include <set>

#include "OctTree.hpp"

#include "LowerLevelBVH.hpp"

namespace Core
{

    namespace Acceleration_Structures
    {

        template<typename T>
        bool BVH<T>::get_first_intersection(const Ray& ray, Acceleration_Structures::InterpolatedVertex &val) const
        {
            #ifdef PICO_PROFILE
            m_test = 0;
            #endif

            float intersection_distance = INFINITY;
            get_closest_intersections(ray, get_node(m_root), val, intersection_distance);

            return intersection_distance != INFINITY;
        }

        template<typename T>
        void    BVH<T>::get_closest_intersections(const Ray& ray, const typename BVH<T>::Node& node, Core::Acceleration_Structures::InterpolatedVertex& intersection, float& intersection_distance) const
        {
    #ifdef PICO_PROFILE
            ++m_tests;
    #endif
            if(node.m_bounding_box.intersection_distance(ray) < intersection_distance)
            {
                for(auto& value : node.m_values)
                {
    #ifdef PICO_PROFILE
                    ++m_tests;
    #endif
                    // If we intersect the bounds then invoke the intersector to check if the contained entity is also intersected
                    if(auto minDistance = value.m_bounds.intersection_distance(ray); minDistance < intersection_distance)
                    {
                        float fine_intersected_distance = INFINITY;
                        Acceleration_Structures::InterpolatedVertex vertex;
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
                        get_closest_intersections(ray, childNode, intersection, intersection_distance);
                    }
                }
            }
        }

        template<typename T>
        void BVH<T>::print_debug_info() const
        {

        }

        template<typename T>
        BVH<T> BVHFactory<T>::generate_octTree()
        {
            const NodeIndex root = createSpacialSubdivisions(m_root_bounding_box, m_bounding_boxes, m_max_depth);

            return BVH<T>{root, m_node_storage, std::move(m_intersector)};
        }


        template<typename T>
        NodeIndex BVHFactory<T>::createSpacialSubdivisions(const AABB& parentBox,
                                                               const std::vector<typename BVH<T>::BoundedValue>& nodes,
                                                               uint32_t depth)
        {
            if (nodes.empty())
            {
                return kInvalidNodeIndex;
            }

            typename BVH<T>::Node newNode{};
            newNode.m_bounding_box = parentBox;

            // Max depth reached so don't subdivide any more.
            if(depth == 0u)
            {
                for (const auto& node : nodes)
                {
                    newNode.m_values.push_back(node);
                    newNode.m_child_count = 0;

                    return add_node(newNode);
                }
            }

            const glm::vec3 halfNodeSize = parentBox.get_side_lengths() / 2.0f;
            std::vector<typename BVH<T>::BoundedValue> unfittedNodes{};
            for (const auto& node : nodes)
            {
                const glm::vec3 size = node.m_bounds.get_side_lengths();
                if(size.x > halfNodeSize.x || size.y > halfNodeSize.y || size.z > halfNodeSize.z)
                    newNode.m_values.push_back(node);
                else
                    unfittedNodes.push_back(node);
            }

            const auto subSpaces = split_AABB(parentBox);

            // Check that the values fit entirely within a subspace partition, if not add them to this "parent" partition
            uint32_t childCount = 0;
            std::vector<uint32_t> unclaimedCount(unfittedNodes.size(), 0u);
            for (uint32_t i = 0; i < subSpaces.size(); ++i)
            {
                std::vector<typename BVH<T>::BoundedValue> subSpaceNodes{};
                for(uint32_t j = 0; j < unfittedNodes.size(); ++j)
                {
                    const auto& node = unfittedNodes[j];
                    if(subSpaces[i].contains(node.m_bounds) & Intersection::Contains)
                    {
                        subSpaceNodes.push_back(node);
                    }
                    else
                    {
                        uint32_t& counter = unclaimedCount[j];
                        ++counter;
                    }
                }

               const NodeIndex child = createSpacialSubdivisions(subSpaces[i], subSpaceNodes, depth - 1u);
               if(child != kInvalidNodeIndex)
                    ++childCount;
               newNode.m_children[i] = child;
            }
            newNode.m_child_count = childCount;

            for(size_t idx = 0; idx < unclaimedCount.size(); idx++)
            {
                if(unclaimedCount[idx] == 8)
                    newNode.m_values.push_back(unfittedNodes[idx]);
            }

            if(newNode.m_values.empty() && newNode.m_child_count == 0)
                return kInvalidNodeIndex;
            else
                return add_node(newNode);
        }




        template<typename T>
        std::array<AABB, 8> BVHFactory<T>::split_AABB(const AABB& aabb) const
        {
            Cube cube = aabb.get_cube();

            const glm::vec4 diagonal = cube.mLower3 - cube.mUpper1;
            const glm::vec4 centre = cube.mUpper1 + (diagonal / 2.0f);

            // Top layer
            const AABB first(cube.mUpper1, centre);
            const AABB second(cube.mUpper1 + glm::vec4(0.0f, 0.0f, diagonal.z / 2.0f, 1.0f), centre + glm::vec4(0.0f, 0.0f, diagonal.z / 2.0f, 1.0f));
            const AABB third(cube.mUpper1 + glm::vec4(diagonal.x / 2.0f, 0.0f, 0.0f, 1.0f), centre + glm::vec4(diagonal.x / 2.0f, 0.0f, 0.0f, 1.0f));
            const AABB fourth(cube.mUpper1 + glm::vec4(diagonal.x / 2.0f, 0.0f, diagonal.z / 2.0f, 1.0f), centre + glm::vec4(diagonal.x / 2.0f, 0.0f, diagonal.z / 2.0f, 1.0f));

            // Bottom layer
            const AABB fith(cube.mUpper1 + glm::vec4(0.0f, diagonal.y / 2.0f, 0.0f, 1.0f), centre + glm::vec4(0.0f, diagonal.y / 2.0f, 0.0f, 1.0f));
            const AABB sixth(cube.mUpper1 + glm::vec4(0.0f, diagonal.y / 2.0f, diagonal.z / 2.0f, 1.0f), centre + glm::vec4(0.0f, diagonal.y / 2.0f, diagonal.z / 2.0f, 1.0f));
            const AABB seventh(cube.mUpper1 + glm::vec4(diagonal.x / 2.0f, diagonal.y / 2.0f, 0.0f, 1.0f), centre + glm::vec4(diagonal.x / 2.0f, diagonal.y / 2.0f, 0.0f, 1.0f));
            const AABB eighth(centre, cube.mLower3);

            return std::array<AABB, 8>{ first, second, third, fourth,
                                        fith, sixth, seventh, eighth };
        }

    }

}

// Explicitly instantiate
#include "Core/UpperLevelBVH.hpp"

template
    class Core::Acceleration_Structures::BVHFactory<const Core::Acceleration_Structures::UpperLevelBVH::Entry*>;

template
    class Core::Acceleration_Structures::BVH<const Core::Acceleration_Structures::UpperLevelBVH::Entry*>;

template
    class Core::Acceleration_Structures::BVHFactory<uint32_t>;

template
    class Core::Acceleration_Structures::BVH<uint32_t>;




