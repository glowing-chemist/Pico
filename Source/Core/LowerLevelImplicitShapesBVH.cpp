#include "LowerLevelImplicitShapesBVH.hpp"


#ifndef M_PI
#define M_PI 3.14159265359
#endif

namespace Core
{

    namespace BVH
    {

    LowerLevelSphereBVH::LowerLevelSphereBVH(const float radius) :
        LowerLevelBVH(),
        mRadius{radius}
    {
    }


    bool LowerLevelSphereBVH::calculate_intersection(const Ray& ray, InterpolatedVertex* result) const
    {
        // sphere center is always at (0, 0).
        const glm::vec3 m = ray.mOrigin;

        const float b = glm::dot(m, ray.mDirection);
        const float c = glm::dot(m, m) - mRadius * mRadius;

        if(c > 0.0f && b > 0.0f)
            return false;
        const float discr = b * b - c;

        if(discr < 0.0f)
            return false;

        float t = -b - sqrt(discr);

        // ray is inside sphere
        if(t < 0.0f)
            t = 0.0f;

        result->mPosition = ray.mOrigin + t * glm::vec4(ray.mDirection, 1.0f);
        result->mPosition.w = 1.0f;
        result->mVertexColour = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
        result->mNormal = glm::normalize(glm::vec3(result->mPosition));
        result->mUV = glm::vec2(atan2(result->mNormal.x, result->mNormal.z) / (2.0f * M_PI) + 0.5f, result->mNormal.y * 0.5f + 0.5f);

        return true;
    }

    AABB LowerLevelSphereBVH::get_bounds() const
    {
        return AABB{{-mRadius, -mRadius, -mRadius, 1.0f}, {mRadius, mRadius, mRadius, 1.0f}};
    }


    }

}
