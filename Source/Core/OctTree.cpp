#include <limits>
#include <algorithm>
#include <array>
#include <iterator>
#include <numeric>
#include <map>
#include <set>

#include "OctTree.hpp"

namespace Core
{

    template<typename T>
    T OctTree<T>::get_first_intersection(const Ray& ray) const
    {
        std::vector<std::pair<float, T>> intersections{};
        get_intersections(ray, get_node(mRoot), intersections);

        auto firstIt = intersections.begin();
        std::nth_element(firstIt, firstIt, intersections.end(), [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

        return firstIt->second;
    }

    template<typename T>
    std::vector<T>  OctTree<T>::get_all_intersections(const Ray& ray) const
    {
        std::vector<std::pair<float, T>> intersections{};
        get_intersections(ray, get_node(mRoot), intersections);

        // Sort nearest to furthest
        std::sort(intersections.begin(), intersections.end(), [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

        std::vector<T> result{};
        std::transform(intersections.begin(), intersections.end(), std::back_inserter(result), [](const auto& input) { return input.second; });

        return result;
    }

    template<typename T>
    void    OctTree<T>::get_intersections(const Ray& ray, const typename OctTree<T>::Node& node, std::vector<std::pair<float, T>>& intersections) const
    {
        ++mTests;
        if(node.mBoundingBox.intersection_distances(ray).first != std::numeric_limits<float>::max())
        {
            for(auto& value : node.mValues)
            {
                ++mTests;
                if(auto minDistance = value.mBounds.intersection_distances(ray).first; minDistance != std::numeric_limits<float>::max())
                    intersections.push_back(std::make_pair(minDistance, value.mValue));
            }

            for (const auto& childIndex : node.mChildren)
            {
                if (childIndex != kInvalidNodeIndex)
                {
                    const Node childNode = get_node(childIndex);
                    get_intersections(ray, childNode, intersections);
                }
            }
        }
    }

    template<typename T>
    OctTree<T> OctTreeFactory<T>::generate_octTree()
    {
        const NodeIndex root = createSpacialSubdivisions(mRootBoundingBox, mBoundingBoxes);

        return OctTree<T>{root, mNodeStorage};
    }


    template<typename T>
    NodeIndex OctTreeFactory<T>::createSpacialSubdivisions(const AABB& parentBox,
                                                           const std::vector<typename OctTree<T>::BoundedValue>& nodes)
    {
        if (nodes.empty())
        {
            return kInvalidNodeIndex;
        }

        typename OctTree<T>::Node newNode{};
        newNode.mBoundingBox = parentBox;

        const glm::vec3 halfNodeSize = parentBox.get_side_lengths() / 2.0f;
        std::vector<typename OctTree<T>::BoundedValue> unfittedNodes{};
        for (const auto& node : nodes)
        {
            const glm::vec3 size = node.mBounds.get_side_lengths();
            if(size.x > halfNodeSize.x || size.y > halfNodeSize.y || size.z > halfNodeSize.z)
                newNode.mValues.push_back(node);
            else
                unfittedNodes.push_back(node);
        }

        const auto subSpaces = split_AABB(parentBox);

        uint32_t childCount = 0;
        std::map<uint32_t, uint32_t> unclaimedCount{};
        for (uint32_t i = 0; i < subSpaces.size(); ++i)
        {
            std::vector<typename OctTree<T>::BoundedValue> subSpaceNodes{};
            for(uint32_t j = 0; j < unfittedNodes.size(); ++j)
            {
                const auto& node = unfittedNodes[j];
                if(subSpaces[i].contains(node.mBounds) & Intersection::Contains)
                {
                    subSpaceNodes.push_back(node);
                }
                else
                {
                    uint32_t& counter = unclaimedCount[j];
                    ++counter;
                }
            }

           const NodeIndex child = createSpacialSubdivisions(subSpaces[i], subSpaceNodes);
           if(child != kInvalidNodeIndex)
                ++childCount;
           newNode.mChildren[i] = child;
        }
        newNode.mChildCount = childCount;

        for(const auto&[idx, count] : unclaimedCount)
        {
            if(count == 8)
                newNode.mValues.push_back(unfittedNodes[idx]);
        }

        if(newNode.mValues.empty() && newNode.mChildCount == 0)
            return kInvalidNodeIndex;
        else
            return add_node(newNode);
    }




    template<typename T>
    std::array<AABB, 8> OctTreeFactory<T>::split_AABB(const AABB& aabb) const
    {
        Cube cube = aabb.getCube();

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

// Explicitly instantiate
#include "Core/UpperLevelBVH.hpp"

template
class Core::OctTreeFactory<const Core::BVH::UpperLevelBVH::Entry*>;

template
class Core::OctTree<const Core::BVH::UpperLevelBVH::Entry*>;




