#ifndef LOWER_LEVEL_IMPLICIT_SHAPES_BVH_HPP
#define LOWER_LEVEL_IMPLICIT_SHAPES_BVH_HPP

#include "LowerLevelBVH.hpp"

namespace Core
{

    namespace BVH
    {

        class LowerLevelSphereBVH : public LowerLevelBVH
        {
        public:

            LowerLevelSphereBVH(const float radius);


            virtual bool calculate_intersection(const Ray&, InterpolatedVertex* result) const final;

            virtual AABB get_bounds() const final;

        private:

            float mRadius;
        };


    }

}


#endif
