#ifndef LOWER_LEVEL_BVH_HPP
#define LOWER_LEVEL_BVH_HPP

#include "glm/common.hpp"
#include "AABB.hpp"

namespace Core
{

    namespace BVH
    {

        struct InterpolatedVertex
        {
            glm::vec4 mPosition;
            glm::vec2 mUV;
            glm::vec3 mNormal;
            glm::vec4 mVertexColour;
            uint32_t  mMaterialID;
        };

        class LowerLevelBVH
        {
        public:

            LowerLevelBVH() = default;
            virtual ~LowerLevelBVH() = default;

            virtual bool calculate_intersection(const Ray&, InterpolatedVertex* result) const = 0;

            virtual AABB get_bounds() const = 0;

        };

    }

}

#endif
