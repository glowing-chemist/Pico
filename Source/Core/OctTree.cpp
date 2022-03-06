#include <limits>
#include <algorithm>
#include <array>
#include <numeric>
#include <map>
#include <set>

#include "OctTree.hpp"

namespace Core
{
    template<typename T>
    std::vector<T> OctTree<T>::getIntersections(const AABB& aabb) const
    {
        mTests = 0;

        std::vector<T> intersections{};

        if(mRoot != kInvalidNodeIndex)
        {
            const Node& root = getNode(mRoot);
            const Intersection initialFlags = root.mBoundingBox.contains(aabb);
            getIntersections(aabb, root, intersections, initialFlags);
        }

        return intersections;
    }


    template<typename T>
    void OctTree<T>::getIntersections(const AABB& aabb, const typename OctTree<T>::Node& node, std::vector<T>& intersections, const Intersection nodeFlags) const
    {
        if(nodeFlags & Intersection::Partial)
        {
            for (const auto& mesh : node.mValues)
            {
                ++mTests;
                if (mesh.mBounds.contains(aabb))
                    intersections.push_back(mesh.mValue);
            }
        }
        else
        {
            return;
        }

        for (const auto& childIndex : node.mChildren)
        {
            if (childIndex != kInvalidNodeIndex)
            {
                const Node childNode = getNode(childIndex);
                ++mTests;
                const Intersection newFlags = childNode.mBoundingBox.contains(aabb);
                if (newFlags)
                    getIntersections(aabb, childNode, intersections, newFlags);
            }
        }
    }


    template<typename T>
    OctTree<T> OctTreeFactory<T>::generateOctTree()
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

        const auto subSpaces = splitAABB(parentBox);

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
            return addNode(newNode);
    }




    template<typename T>
    std::array<AABB, 8> OctTreeFactory<T>::splitAABB(const AABB& aabb) const
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


    // Explicitly instantiate
    #include "Core/UpperLevelBVH.hpp"

    template
    class OctTreeFactory<AABB*>;

    template
    class OctTree<AABB*>;

}




