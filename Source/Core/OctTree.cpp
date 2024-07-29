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
        template<typename T>
        OctTree<T>* OctTreeFactory<T>::generate_octTree()
        {
            const NodeIndex root = create_spacial_subdivisions(m_root_bounding_box, m_bounding_boxes, m_max_depth);

            minimise_bounds(root);

            return new OctTree<T>{root, m_node_storage, std::move(m_intersector)};
        }


        template<typename T>
        NodeIndex OctTreeFactory<T>::create_spacial_subdivisions(const AABB& parentBox,
                                                               const std::vector<BuilderNode>& nodes,
                                                               uint32_t depth)
        {
            if (nodes.empty())
            {
                return kInvalidNodeIndex;
            }

            typename OctTree<T>::Node newNode{};
            newNode.m_bounding_box = parentBox;

            // Max depth reached so don't subdivide any more.
            if(depth == 0u)
            {
                for(auto& child_slot : newNode.m_children)
                    child_slot = kInvalidNodeIndex;

                for (const auto& node : nodes)
                {
                    newNode.m_values.push_back(node);

                    return add_node(newNode);
                }
            }

            const glm::vec3 halfNodeSize = parentBox.get_side_lengths() / 2.0f;
            std::vector<typename BVH<T, 8>::BoundedValue> unfittedNodes{};
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
            for (uint32_t i = 0; i < subSpaces.size(); ++i)
            {
                std::vector<typename BVH<T, 8>::BoundedValue> subSpaceNodes{};
                for(uint32_t j = 0; j < unfittedNodes.size(); ++j)
                {
                    const auto& node = unfittedNodes[j];
                    if(auto intersection_flags = subSpaces[i].contains(node.m_bounds); (intersection_flags & Intersection::Contains) || (intersection_flags & Intersection::Partial))
                    {
                        subSpaceNodes.push_back(node);
                    }
                }

               const NodeIndex child = create_spacial_subdivisions(subSpaces[i], subSpaceNodes, depth - 1u);
               if(child != kInvalidNodeIndex)
                    ++childCount;
               newNode.m_children[i] = child;
            }

            if(newNode.m_values.empty() && childCount == 0)
                return kInvalidNodeIndex;
            else
                return add_node(newNode);
        }




        template<typename T>
        std::array<AABB, 8> OctTreeFactory<T>::split_AABB(const AABB& aabb) const
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

        template<typename T>
        AABB OctTreeFactory<T>::minimise_bounds(const uint32_t node_index)
        {
            typename BVH<T, 8>::Node& node = m_node_storage[node_index];

            glm::vec4 min(INFINITY, INFINITY, INFINITY, INFINITY);
            glm::vec4 max(-INFINITY, -INFINITY, -INFINITY, -INFINITY);
            for(uint32_t i = 0; i < node.m_values.size(); ++i)
            {
                min = Core::component_wise_min(min, node.m_values[i].m_bounds.get_min());
                max = Core::component_wise_max(max, node.m_values[i].m_bounds.get_max());
            }

            for(uint32_t i = 0; i < 8; ++i)
            {
                if(node.m_children[i] != kInvalidNodeIndex)
                {
                    const AABB child_bounds = minimise_bounds(node.m_children[i]);

                    min = Core::component_wise_min(min, child_bounds.get_min());
                    max = Core::component_wise_max(max, child_bounds.get_max());
                }
            }

            AABB new_bounds = AABB{min, max};
            node.m_bounding_box = new_bounds;

            return new_bounds;

        }

    }

}

// Explicitly instantiate
#include "Core/UpperLevelBVH.hpp"

template
    class Core::Acceleration_Structures::OctTreeFactory<const Core::Acceleration_Structures::UpperLevelBVH::Entry*>;

template
    class Core::Acceleration_Structures::OctTree<const Core::Acceleration_Structures::UpperLevelBVH::Entry*>;

template
    class Core::Acceleration_Structures::OctTreeFactory<uint32_t>;

template
    class Core::Acceleration_Structures::OctTree<uint32_t>;




