#ifndef OCT_TREE_HPP
#define OCT_TREE_HPP


#include <array>
#include <memory>
#include <optional>
#include <set>
#include <tuple>
#include <vector>

#include "AABB.hpp"

namespace Core
{

    constexpr uint32_t kInvalidNodeIndex = ~0u;
    using NodeIndex = uint32_t;


    template<typename T>
    class OctTree
    {
    public:
        struct Node;

        OctTree(const NodeIndex rootIndex, std::vector<Node>& nodeStorage) : mTests{0},  mRoot{rootIndex}, mNodes{nodeStorage} {}
        OctTree() : mTests{0}, mRoot{kInvalidNodeIndex} , mNodes{} {}

        OctTree(OctTree&&) = default;
        OctTree& operator=(OctTree&&) = default;

        T               get_first_intersection(const Ray& ray) const;

        std::vector<T>  get_all_intersections(const Ray& ray) const;

        struct BoundedValue
        {
            AABB mBounds;
            T mValue;
        };

        struct Node
        {
            Node() = default;

            // Use an optional incase T doesn't have a default constructor.
           AABB mBoundingBox;
           std::vector<BoundedValue> mValues;

           uint32_t mChildCount;
           NodeIndex mChildren[8];
        };

        uint32_t get_tests_performed() const
        {
            return mTests;
        }

    private:

        const Node& get_node(const NodeIndex n) const
        {
            return mNodes[n];
        }

        void    get_intersections(const Ray& ray, const typename OctTree<T>::Node& node, std::vector<std::pair<float, T>>& intersections) const;

        mutable uint32_t mTests;
        NodeIndex mRoot;

        std::vector<Node> mNodes;
    };


    template<typename T>
    class OctTreeFactory
    {
    public:

        using BuilderNode = typename OctTree<T>::BoundedValue;

        OctTreeFactory(const AABB& rootBox, std::vector<typename OctTree<T>::BoundedValue>& data) : mRootBoundingBox{rootBox},
                                                                                mBoundingBoxes{data} {}


        OctTree<T> generate_octTree();

    private:

        NodeIndex add_node(const typename OctTree<T>::Node& n)
        {
            const NodeIndex i = mNodeStorage.size();
            mNodeStorage.push_back(n);

            return i;
        }

        std::array<AABB, 8> split_AABB(const AABB&) const;
        NodeIndex createSpacialSubdivisions(const AABB& parentBox,
                                            const std::vector<typename OctTree<T>::BoundedValue>& nodes);

        std::vector<typename OctTree<T>::Node> mNodeStorage;

        AABB mRootBoundingBox; // AABB that all others are contained within.
        std::vector<typename OctTree<T>::BoundedValue> mBoundingBoxes;
    };

}

#endif
