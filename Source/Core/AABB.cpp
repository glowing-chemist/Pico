#include "AABB.hpp"

namespace Core
{

    // Return a vector constructed from the minimum of each component.
    inline glm::vec3 component_wise_min(const glm::vec3& lhs, const glm::vec3& rhs)
    {
        return glm::vec3{std::min(lhs.x, rhs.x),
                      std::min(lhs.y, rhs.y),
                      std::min(lhs.z, rhs.z)};
    }


    inline glm::vec4 component_wise_min(const glm::vec4& lhs, const glm::vec4& rhs)
    {
        return glm::vec4{std::min(lhs.x, rhs.x),
                      std::min(lhs.y, rhs.y),
                      std::min(lhs.z, rhs.z),
                      std::min(lhs.w, rhs.w)};
    }

    inline glm::vec3 component_wise_max(const glm::vec3& lhs, const glm::vec3& rhs)
    {
        return glm::vec3{std::max(lhs.x, rhs.x),
                      std::max(lhs.y, rhs.y),
                      std::max(lhs.z, rhs.z)};
    }


    inline glm::vec4 component_wise_max(const glm::vec4& lhs, const glm::vec4& rhs)
    {
        return glm::vec4{std::max(lhs.x, rhs.x),
                      std::max(lhs.y, rhs.y),
                      std::max(lhs.z, rhs.z),
                      std::max(lhs.w, rhs.w)};
    }

    Ray transform_ray(const Ray& ray, const glm::mat4x4& transform)
    {
        assert(ray.mOrigin.w == 1.0f);
        Ray newRay{};
        newRay.mOrigin = transform * ray.mOrigin;
        newRay.mDirection = glm::mat3x3(transform) * ray.mDirection;
        newRay.mLenght = ray.mLenght;

        return newRay;
    }

    std::array<glm::vec4, 8> AABB::get_cube_as_vertex_array() const
    {
        glm::vec4 upper1{mMinimum};
        glm::vec4 upper2{mMinimum.x, mMinimum.y, mMaximum.z, 1.0f};
        glm::vec4 upper3{mMaximum.x, mMinimum.y, mMaximum.z, 1.0f };
        glm::vec4 upper4{mMaximum.x, mMinimum.y, mMinimum.z, 1.0f };

        glm::vec4 lower1{mMinimum.x, mMaximum.y, mMinimum.z, 1.0f };
        glm::vec4 lower2{mMinimum.x, mMaximum.y, mMaximum.z, 1.0f };
        glm::vec4 lower3{mMaximum};
        glm::vec4 lower4{mMaximum.x, mMaximum.y, mMinimum.z, 1.0f };

        return {upper1, upper2, upper3, upper4,
                lower1, lower2, lower3, lower4};
    }


    Cube AABB::getCube() const
    {

        auto verticies = get_cube_as_vertex_array();

        return Cube{verticies[0], verticies[1], verticies[2], verticies[3],
                    verticies[4], verticies[5], verticies[6], verticies[7]};
    }


    std::pair<float, float> AABB::intersection_distances(const Ray& ray) const
    {
        glm::vec3 rayDirection = ray.mDirection;
        glm::vec3 rayOrigin = ray.mOrigin;
        glm::vec3 inverseDirection{1.0f / rayDirection.x, 1.0f / rayDirection.y, 1.0f / rayDirection.z};

        // mTopFrontLeft is the corner of AABB with minimal coordinates - left top.
        float t1 = (mMinimum.x - rayOrigin.x) * inverseDirection.x;
        float t2 = (mMaximum.x - rayOrigin.x) * inverseDirection.x;
        float t3 = (mMinimum.y - rayOrigin.y) * inverseDirection.y;
        float t4 = (mMaximum.y - rayOrigin.y) * inverseDirection.y;
        float t5 = (mMinimum.z - rayOrigin.z) * inverseDirection.z;
        float t6 = (mMaximum.z - rayOrigin.z) * inverseDirection.z;

        float tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
        float tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

        // if tmax < 0, ray the ray intersects AABB, but the AABB is behind the ray (direction faces away)
        // or no intersection at all.
        if (tmax < 0 || tmin > tmax)
        {
            return std::make_pair(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        }

        return std::make_pair(tmin, tmax);
    }


    bool AABB::contains(const glm::vec4& point) const
    {
        return point.x >= mMinimum.x && point.x <= mMaximum.x &&
            point.y >= mMinimum.y && point.y <= mMaximum.y &&
            point.z >= mMinimum.z && point.z <= mMaximum.z;
    }


    Intersection AABB::contains(const AABB& aabb) const
    {
        const auto verticies = aabb.get_cube_as_vertex_array();

        Intersection inside = Intersection::None;
        bool all = true;
        for (const auto& vertex : verticies)
        {
            const bool isInside = contains(vertex);
            all = all && isInside;
            inside = static_cast<Intersection>(inside | (isInside ? Intersection::Partial : Intersection::None));
        }

        return static_cast<Intersection>(inside | (all ? Intersection::Contains : Intersection::None));
    }


    AABB& AABB::operator*=(const glm::mat4x4& mat)
    {
        // Keep track of the max/min values seen on each axis
        // so tha we still have an AABB not an OOBB.
        const auto cubeVerticies= get_cube_as_vertex_array();
        glm::vec4 smallest = glm::vec4(10000000.0f);
        glm::vec4 largest = glm::vec4(-10000000.0f);
        for (const auto& vertex : cubeVerticies)
        {
            glm::vec4 transformedPoint = mat * vertex;
            transformedPoint /= transformedPoint.w;

            smallest = component_wise_min(smallest, transformedPoint);
            largest = component_wise_max(largest, transformedPoint);
        }

        mMinimum = smallest;
        mMaximum = largest;

        return *this;
    }


    AABB& AABB::operator*=(const glm::vec4& vec)
    {
        mMinimum *= vec;
        mMaximum *= vec;

        return *this;
    }


    AABB& AABB::operator+=(const glm::vec4& vec)
    {
        mMinimum += vec;
        mMaximum += vec;

        return *this;
    }


    AABB& AABB::operator-=(const glm::vec4& vec)
    {
        mMinimum -= vec;
        mMaximum -= vec;

        return *this;
    }


    AABB AABB::operator*(const glm::mat4x4& mat) const
    {
        // Keep track of the max/min values seen on each axis
        // so tha we still have an AABB not an OOBB.
        const auto cubeVerticies = get_cube_as_vertex_array();
        glm::vec4 smallest = glm::vec4(10000000.0f);
        glm::vec4 largest = glm::vec4(-10000000.0f);
        for (const auto& vertex : cubeVerticies)
        {
            glm::vec4 transformedPoint = mat * vertex;
            transformedPoint /= transformedPoint.w;

            smallest = component_wise_min(smallest, transformedPoint);
            largest = component_wise_max(largest, transformedPoint);
        }

        return AABB{ smallest, largest };
    }


    AABB AABB::operator*(const glm::vec4& vec) const
    {
        return AABB{ mMinimum * vec, mMaximum * vec };
    }


    AABB AABB::operator+(const glm::vec4& vec) const
    {
        return AABB{ mMinimum + vec, mMaximum + vec };
    }


    AABB AABB::operator-(const glm::vec4& vec) const
    {
        return AABB{ mMinimum - vec, mMaximum - vec };
    }

}
