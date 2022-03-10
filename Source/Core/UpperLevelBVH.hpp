#ifndef UPPER_LEVEL_BVH_HPP
#define UPPER_LEVEL_BVH_HPP

#include "AABB.hpp"
#include "LowerLevelBVH.hpp"
#include "OctTree.hpp"

#include "glm/mat4x4.hpp"

#include <memory>
#include <vector>

namespace Core
{

    namespace BVH
    {

        class UpperLevelBVH
        {
        public:

            struct Entry
            {
                glm::mat4x4                     mTransform;
                glm::mat4x4                     mInverseTransform;
                std::shared_ptr<LowerLevelBVH>  mBVH;
                uint32_t                        mMaterialIndex;
            };

            UpperLevelBVH() = default;

            bool get_closest_intersection(const Ray&, InterpolatedVertex* vertex) const;

            void get_all_intersections(const Ray&, std::vector<InterpolatedVertex>& vertex) const;

            void add_lower_level_bvh(std::shared_ptr<LowerLevelBVH>& bvh, const glm::mat4x4& transform, uint32_t materialIndex);

            void build();

        private:

            std::vector<Entry> mLowerLevelBVHs;
            OctTree<const Entry*> mOctTree;

        };

    }

}

#endif
