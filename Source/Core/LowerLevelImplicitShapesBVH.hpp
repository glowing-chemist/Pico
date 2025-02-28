#ifndef LOWER_LEVEL_IMPLICIT_SHAPES_BVH_HPP
#define LOWER_LEVEL_IMPLICIT_SHAPES_BVH_HPP

#include "LowerLevelBVH.hpp"

namespace Core
{

    namespace Acceleration_Structures
    {

        class LowerLevelSphereBVH : public LowerLevelBVH
        {
        public:

            LowerLevelSphereBVH(const float radius);


            virtual bool calculate_intersection(Ray&, InterpolatedVertex* result) const final;

            virtual AABB get_bounds() const final;

            virtual void generate_sampling_data() final {}

            virtual bool sample_geometry(Core::Rand::Hammersley_Generator&, const glm::vec3&, const glm::vec3&, glm::vec3&, float&) final;

        private:

            float mRadius;
        };


    }

}


#endif
