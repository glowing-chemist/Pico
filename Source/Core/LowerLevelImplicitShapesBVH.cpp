#include "LowerLevelImplicitShapesBVH.hpp"
#include "Render/SolidAngle.hpp"
#include "Core/Asserts.hpp"

#ifndef M_PI
#define M_PI 3.14159265359
#endif

namespace Core
{

    namespace Acceleration_Structures
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

            //PICO_ASSERT(t > 0.0f);
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

        bool LowerLevelSphereBVH::sample_geometry(Core::Rand::Hammersley_Generator& rand, const glm::vec3& point, const glm::vec3& N, glm::vec3& sample_point, float& pdf)
        {
            const glm::vec3 unit_sphere_point = Core::Rand::uniform_sample_sphere(rand.next());
            const glm::vec3 sphere_point = mRadius * unit_sphere_point;

            const glm::vec3 to_point = glm::normalize(sphere_point - point);
            if(glm::dot(to_point, N) >= 0.0f)
                sample_point = sphere_point;
            else
                sample_point = -sphere_point;

            pdf = 1.0f;

            return true;
        }


    }

}
