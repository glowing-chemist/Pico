#ifndef OCT_TREE_HPP
#define OCT_TREE_HPP


#include <array>
#include <memory>
#include <optional>
#include <set>
#include <tuple>
#include <vector>

#include "AABB.hpp"
#include "BVH.hpp"

//#define BVH_PROFILE

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
        class OctTree : public BVH<T, 8>
        {
        public:

            using Node = BVH<T, 8>::Node;
            using BoundedValue = BVH<T, 8>::BoundedValue;

            OctTree(const NodeIndex rootIndex, std::vector<Node>& nodeStorage, std::unique_ptr<Intersector<T>>&& intersector) : BVH<T, 8>(rootIndex, nodeStorage, std::move(intersector)) {}

            OctTree(OctTree&&) = default;
            OctTree& operator=(OctTree&&) = default;
        };


        template<typename T>
        class OctTreeFactory
        {
        public:

            using BuilderNode = typename BVH<T, 8>::BoundedValue;

            OctTreeFactory(const AABB& rootBox, std::vector<BuilderNode>& data) :
                                                            m_root_bounding_box{rootBox},
                                                            m_bounding_boxes{data},
                                                            m_intersector{},
                                                            m_max_depth{32u} {}


            OctTree<T>* generate_octTree();

            OctTreeFactory<T>& set_intersector(std::unique_ptr<Intersector<T>>&& intersector)
            {
                m_intersector = std::move(intersector);
                return *this;
            }

            OctTreeFactory<T>& set_max_depth(const uint32_t depth)
            {
                m_max_depth = depth;
                return *this;
            }

        private:

            NodeIndex add_node(const typename OctTree<T>::Node& n)
            {
                const NodeIndex i = m_node_storage.size();
                m_node_storage.push_back(n);

                return i;
            }

            std::array<AABB, 8> split_AABB(const AABB&) const;
            NodeIndex create_spacial_subdivisions(const AABB& parentBox,
                                                const std::vector<BuilderNode>& nodes,
                                                uint32_t depth);

            AABB minimise_bounds(const uint32_t node_index);

            std::vector<typename BVH<T, 8>::Node> m_node_storage;

            AABB m_root_bounding_box; // AABB that all others are contained within.
            std::vector<BuilderNode> m_bounding_boxes;

            std::unique_ptr<Intersector<T>> m_intersector;

            // Build settings
            uint32_t m_max_depth;
        };
    }
}

#endif
