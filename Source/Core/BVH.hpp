#ifndef PICO_BVH_HPP
#define PICO_BVH_HPP

#include <memory>
#include <vector>

#include "AABB.hpp"

namespace Core
{
    namespace Acceleration_Structures
    {
        constexpr uint32_t kInvalidNodeIndex = ~0u;
        using NodeIndex = uint32_t;

        struct InterpolatedVertex;

        template<typename T>
        class Intersector
        {
        public:

            virtual bool intersects(const Ray& ray, T, float& intersect_distance, Core::Acceleration_Structures::InterpolatedVertex& intersection_data) const = 0;
        };

        template<typename T, uint32_t C>
        class BVH
        {
        public:
            struct Node;

            BVH(const NodeIndex rootIndex, std::vector<Node>& nodeStorage, std::unique_ptr<Intersector<T>>&& intersector) :
                m_root{rootIndex},
                m_nodes{nodeStorage},
                m_intersector{std::move(intersector)} {}

            BVH() : m_root{kInvalidNodeIndex} , m_nodes{}, m_intersector{} {}

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
                NodeIndex m_children[C];
            };

        protected:

            const Node& get_node(const NodeIndex n) const
            {
                return m_nodes[n];
            }

            void    get_closest_intersections(const Ray& ray, const typename BVH<T, C>::Node& node, Core::Acceleration_Structures::InterpolatedVertex& intersection, float& intersection_distance
#ifdef BVH_PROFILE
                                           , uint32_t& tests_performed
#endif
                                           ) const;

            NodeIndex m_root;

            std::vector<Node> m_nodes;

            std::unique_ptr<Intersector<T>> m_intersector;
        };
    }
}

#endif
