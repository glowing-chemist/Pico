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

            virtual ~Intersector() {}

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

            bool get_first_intersection(Ray& ray, Acceleration_Structures::InterpolatedVertex&) const;

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

                NodeIndex m_children[C];

                bool is_leaf() const
                {
                    return !m_values.empty();
                }
            };

        protected:

            const Node& get_node(const NodeIndex n) const
            {
                return m_nodes[n];
            }

            void    get_closest_intersections(const Ray& ray, const typename BVH<T, C>::Node& node, Core::Acceleration_Structures::InterpolatedVertex& intersection, float& intersection_distance) const;

            NodeIndex m_root;

            std::vector<Node> m_nodes;

            std::unique_ptr<Intersector<T>> m_intersector;
        };

        template<typename T>
        class BVHPartitionScheme
        {
        public:

            using ITERATOR = std::vector<typename BVH<T, 2>::BoundedValue>::iterator;

            virtual ITERATOR partition(ITERATOR start, ITERATOR end) = 0;
        };

        template<typename T>
        class Centroid_parition_Scheme : public BVHPartitionScheme<T>
        {
        public:
            using ITERATOR = BVHPartitionScheme<T>::ITERATOR;

            virtual ITERATOR partition(ITERATOR start, ITERATOR end) override final;
        };

        template<typename T>
        class SAH_parition_Scheme : public BVHPartitionScheme<T>
        {
        public:
            using ITERATOR = BVHPartitionScheme<T>::ITERATOR;

            struct Bucket
            {
                uint32_t mCount;
                AABB mBounds;
            };

            virtual ITERATOR partition(ITERATOR start, ITERATOR end) override final;
        };

        template<typename T>
        class BVHFactory
        {
        public:

            using BuilderNode = typename BVH<T, 2>::BoundedValue;

            BVHFactory(const AABB& rootBox, std::vector<BuilderNode>& data) :
                m_root_bounding_box{rootBox},
                m_bounding_boxes{data},
                m_intersector{},
                m_max_depth{32u} {}


            BVH<T, 2>* generate_BVH();

            BVHFactory<T>& set_intersector(std::unique_ptr<Intersector<T>>&& intersector)
            {
                m_intersector = std::move(intersector);
                return *this;
            }

            BVHFactory<T>& set_parition_scheme(std::unique_ptr<BVHPartitionScheme<T>>&& partition_scheme)
            {
                m_partition_scheme = std::move(partition_scheme);
                return *this;
            }

            BVHFactory<T>& set_max_depth(const uint32_t depth)
            {
                m_max_depth = depth;
                return *this;
            }


        private:

            NodeIndex split_primitives(BVHPartitionScheme<T>::ITERATOR start, BVHPartitionScheme<T>::ITERATOR end,uint32_t depth);

            NodeIndex add_node(const typename BVH<T, 2>::Node& n)
            {
                const NodeIndex i = m_node_storage.size();
                m_node_storage.push_back(n);

                return i;
            }


            std::vector<typename BVH<T, 2>::Node> m_node_storage;

            AABB m_root_bounding_box; // AABB that all others are contained within.
            std::vector<BuilderNode> m_bounding_boxes;

            std::unique_ptr<Intersector<T>> m_intersector;
            std::unique_ptr<BVHPartitionScheme<T>> m_partition_scheme;

            // Build settings
            uint32_t m_max_depth;
        };


    }
}

#endif
