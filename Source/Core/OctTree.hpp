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

    namespace Acceleration_Structures
    {
        struct InterpolatedVertex;

        template<typename T>
        class Intersector;

        template<typename T>
        class BVH
        {
        public:
            struct Node;

            BVH(const NodeIndex rootIndex, std::vector<Node>& nodeStorage, std::unique_ptr<Intersector<T>>&& intersector) :
                m_tests{0},
                m_root{rootIndex},
                m_nodes{nodeStorage},
                m_intersector{std::move(intersector)} {}

            BVH() : m_tests{0}, m_root{kInvalidNodeIndex} , m_nodes{}, m_intersector{} {}

            BVH(BVH&&) = default;
            BVH& operator=(BVH&&) = default;

            bool get_first_intersection(const Ray& ray, Acceleration_Structures::InterpolatedVertex&) const;

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

            void print_debug_info() const;

        private:

            const Node& get_node(const NodeIndex n) const
            {
                return m_nodes[n];
            }

            void    get_closest_intersections(const Ray& ray, const typename BVH<T>::Node& node, Core::Acceleration_Structures::InterpolatedVertex& intersection, float& intersection_distance) const;

            mutable uint32_t m_tests;
            NodeIndex m_root;

            std::vector<Node> m_nodes;

            std::unique_ptr<Intersector<T>> m_intersector;
        };


        template<typename T>
        class BVHFactory
        {
        public:

            using BuilderNode = typename BVH<T>::BoundedValue;

            BVHFactory(const AABB& rootBox, std::vector<typename BVH<T>::BoundedValue>& data) :
                                                                                    m_root_bounding_box{rootBox},
                                                                                    m_bounding_boxes{data},
                                                                                    m_intersector{},
                                                                                    m_max_depth{32u} {}


            BVH<T> generate_octTree();

            BVHFactory<T>& set_intersector(std::unique_ptr<Intersector<T>>&& intersector)
            {
                m_intersector = std::move(intersector);
                return *this;
            }

            BVHFactory<T>& set_max_depth(const uint32_t depth)
            {
                m_max_depth = depth;
                return *this;
            }

        private:

            NodeIndex add_node(const typename BVH<T>::Node& n)
            {
                const NodeIndex i = m_node_storage.size();
                m_node_storage.push_back(n);

                return i;
            }

            std::array<AABB, 8> split_AABB(const AABB&) const;
            NodeIndex create_spacial_subdivisions(const AABB& parentBox,
                                                const std::vector<typename BVH<T>::BoundedValue>& nodes,
                                                uint32_t depth);

            AABB minimise_bounds(const uint32_t node_index);

            std::vector<typename BVH<T>::Node> m_node_storage;

            AABB m_root_bounding_box; // AABB that all others are contained within.
            std::vector<typename BVH<T>::BoundedValue> m_bounding_boxes;

            std::unique_ptr<Intersector<T>> m_intersector;

            // Build settings
            uint32_t m_max_depth;
        };


        template<typename T>
        class Intersector
        {
        public:

            virtual bool intersects(const Ray& ray, T, float& intersect_distance, Core::Acceleration_Structures::InterpolatedVertex& intersection_data) const = 0;

        };

    }
}

#endif
