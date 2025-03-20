#include "LowerLevelImplicitShapesBVH.hpp"
#include "Core/AABB.hpp"
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


        bool LowerLevelSphereBVH::calculate_intersection(Ray& ray, InterpolatedVertex* result) const
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

        bool LowerLevelSphereBVH::sample_geometry(Core::Rand::Hammersley_Generator& rand, glm::vec3& sample_point, float& pdf)
        {
            const glm::vec3 unit_sphere_point = Core::Rand::uniform_sample_sphere(rand.next());
            const glm::vec3 sphere_point = mRadius * unit_sphere_point;

            sample_point = sphere_point;
            pdf = 1.0f;

            return true;
        }

        LowerLevelCube::LowerLevelCube() :
            m_box(glm::vec4(-0.5f, -0.5f, -0.5f, 1.0f), glm::vec4(0.5f, 0.5f, 0.5f, 1.0f))
        {}

        bool LowerLevelCube::calculate_intersection(Ray& ray, InterpolatedVertex* result) const
        {
            const float t = m_box.intersection_distance(ray);
            
            if(t == INFINITY)
                return false;

            result->mPosition = ray.mOrigin + glm::vec4((ray.mDirection * t), 1.0f);
            result->mPosition.w = 1.0f;
            result->mUV = glm::vec2(0.0f, 0.0f); // Don't support textured implicit cubes.
            const uint32_t max_index =  Core::maximum_component_index(result->mPosition);
            glm:: vec3 normal = glm::vec3(0.0f, 0.0f, 0.0f);
            normal[max_index] = 1.0f * glm::sign(result->mPosition[max_index]);
            result->mNormal = normal;
            result->mVertexColour = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);

            return true;
        }

        bool LowerLevelCube::sample_geometry(Core::Rand::Hammersley_Generator& rand, glm::vec3& sample_point, float& pdf)
        {
            const glm::vec2 Xi = rand.next();
            const glm::vec2 Xj = rand.next();
            const glm::vec2 Xk = rand.next();

            const glm::vec3 corner = glm::sign(glm::vec3(Xi.x, Xi.y, Xj.x) - 0.5f);

            glm::vec3 face_offset{};
            u_int32_t zero_axis = static_cast<uint32_t>(Xj.y * 3.0f);
            if(zero_axis == 3) zero_axis = 2;

            face_offset[zero_axis] = 0.0f;
            face_offset[(zero_axis + 1) % 3] = Xk.x;
            face_offset[(zero_axis + 1) % 3] = Xk.y;

            // Incorrect, TODO fix this
            sample_point = (corner * 0.5f)  + (-corner * face_offset);
            pdf = 1.0f;

            return true;
        }

    }

}
