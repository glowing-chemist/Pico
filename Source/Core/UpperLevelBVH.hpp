#ifndef UPPER_LEVEL_BVH_HPP
#define UPPER_LEVEL_BVH_HPP

#include "AABB.hpp"
#include "LowerLevelBVH.hpp"
#include "OctTree.hpp"
#include "Render/BSRDF.hpp"

#include "glm/mat4x4.hpp"

#include <memory>
#include <vector>

namespace Core
{

    namespace Acceleration_Structures
    {

        class UpperLevelBVH
        {
        public:

            struct Entry
            {
                glm::mat4x4                    mTransform;
                glm::mat4x4                    mInverseTransform;
                std::shared_ptr<LowerLevelBVH> mBVH;
                std::shared_ptr<Render::BSRDF> m_material;
            };

            UpperLevelBVH() = default;

            bool get_closest_intersection(const Ray&, InterpolatedVertex* vertex) const;

            void get_all_intersections(const Ray&, std::vector<InterpolatedVertex>& vertex) const;

            void add_lower_level_bvh(std::shared_ptr<Acceleration_Structures::LowerLevelBVH>& bvh, const glm::mat4x4& transform, std::shared_ptr<Render::BSRDF>& bsrdf);

            void build();

        private:

            class lower_level_intersector : public Core::Acceleration_Structures::Intersector<const Entry*>
            {
            public:
                virtual bool intersects(const Core::Ray& ray, const Entry*, float& intersect_distance, InterpolatedVertex&) const override;
            };

            std::vector<Entry> mLowerLevelBVHs;
            BVH<const Entry*> mOctTree;

        };

    }

}

#endif
