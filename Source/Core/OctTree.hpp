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
    class Intersector;

    namespace BVH
    {
        struct InterpolatedVertex;
    }

    template<typename T>
    class OctTree
    {
    public:
        struct Node;

        OctTree(const NodeIndex rootIndex, std::vector<Node>& nodeStorage, std::unique_ptr<Intersector<T>>&& intersector) :
            m_tests{0},
            m_root{rootIndex},
            m_nodes{nodeStorage},
            m_intersector{std::move(intersector)} {}

        OctTree() : m_tests{0}, m_root{kInvalidNodeIndex} , m_nodes{}, m_intersector{} {}

        OctTree(OctTree&&) = default;
        OctTree& operator=(OctTree&&) = default;

        bool get_first_intersection(const Ray& ray, BVH::InterpolatedVertex&) const;

        struct BoundedValue
        {
            AABB m_bounds;
            T m_value;
        };

        struct Node
        {
            Node() = default;

            // Use an optional incase T doesn't have a default constructor.
           AABB m_bounding_box;
           std::vector<BoundedValue> m_values;

           uint32_t m_child_count;
           NodeIndex m_children[8];
        };

        uint32_t get_tests_performed() const
        {
            return m_tests;
        }

    private:

        const Node& get_node(const NodeIndex n) const
        {
            return m_nodes[n];
        }

        void    get_intersections(const Ray& ray, const typename OctTree<T>::Node& node, std::vector<std::pair<float, Core::BVH::InterpolatedVertex>>& intersections) const;

        mutable uint32_t m_tests;
        NodeIndex m_root;

        std::vector<Node> m_nodes;

        std::unique_ptr<Intersector<T>> m_intersector;
    };


    template<typename T>
    class OctTreeFactory
    {
    public:

        using BuilderNode = typename OctTree<T>::BoundedValue;

        OctTreeFactory(const AABB& rootBox, std::vector<typename OctTree<T>::BoundedValue>& data, std::unique_ptr<Intersector<T>>&& intersector) :
                                                                                m_root_bounding_box{rootBox},
                                                                                m_bounding_boxes{data},
                                                                                m_intersector{std::move(intersector)} {}


        OctTree<T> generate_octTree();

    private:

        NodeIndex add_node(const typename OctTree<T>::Node& n)
        {
            const NodeIndex i = m_node_storage.size();
            m_node_storage.push_back(n);

            return i;
        }

        std::array<AABB, 8> split_AABB(const AABB&) const;
        NodeIndex createSpacialSubdivisions(const AABB& parentBox,
                                            const std::vector<typename OctTree<T>::BoundedValue>& nodes);

        std::vector<typename OctTree<T>::Node> m_node_storage;

        AABB m_root_bounding_box; // AABB that all others are contained within.
        std::vector<typename OctTree<T>::BoundedValue> m_bounding_boxes;

        std::unique_ptr<Intersector<T>> m_intersector;
    };


    template<typename T>
    class Intersector
    {
    public:

        virtual bool intersects(const Ray& ray, T, float& intersect_distance, Core::BVH::InterpolatedVertex& intersection_data) = 0;

    };
}

#endif
