#include "AABB.hpp"
#include "Core/Asserts.hpp"

namespace Core
{

    // Return a vector constructed from the minimum of each component.
    glm::vec3 component_wise_min(const glm::vec3& lhs, const glm::vec3& rhs)
    {
        return glm::min(lhs, rhs);
    }


    glm::vec4 component_wise_min(const glm::vec4& lhs, const glm::vec4& rhs)
    {
        return glm::min(lhs, rhs);
    }


    glm::vec3 component_wise_max(const glm::vec3& lhs, const glm::vec3& rhs)
    {
        return glm::max(lhs, rhs);
    }


    glm::vec4 component_wise_max(const glm::vec4& lhs, const glm::vec4& rhs)
    {
        return glm::max(lhs, rhs);
    }

    uint32_t maximum_component_index(const glm::vec3& v)
    {
        if((v.x >= v.y) && (v.x >= v.z))
            return 0;
        else if((v.y >= v.x) && (v.y >= v.z))
            return 1;
        else
            return 2;
    }

    Ray transform_ray(const Ray& ray, const glm::mat4x4& transform)
    {
        Ray newRay{};
        newRay.mOrigin = transform * ray.mOrigin;
        newRay.mDirection = glm::mat3x3(transform) * ray.mDirection;
        newRay.mLenght = ray.mLenght;

        return newRay;
    }

    void Ray::push_index_of_refraction(const float IoR)
    {
        m_index_or_refraction_stack.push_back(IoR);
    }

    float Ray::pop_index_of_refraction()
    {
        PICO_ASSERT(!m_index_or_refraction_stack.empty());
        const float IoR = m_index_or_refraction_stack.back();
        m_index_or_refraction_stack.pop_back();
        PICO_ASSERT(!m_index_or_refraction_stack.empty());

        return IoR;
    }

    float Ray::get_current_index_of_refraction() const
    {
        PICO_ASSERT(!m_index_or_refraction_stack.empty());

        return m_index_or_refraction_stack.back();
    }

    bool Ray::inside_geometry() const
    {
        return m_index_or_refraction_stack.size() > 1; 
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


    Cube AABB::get_cube() const
    {

        auto verticies = get_cube_as_vertex_array();

        return Cube{verticies[0], verticies[1], verticies[2], verticies[3],
                    verticies[4], verticies[5], verticies[6], verticies[7]};
    }


    float AABB::intersection_distance(const Ray& ray) const
    {
        struct MinMaxResult
        {
            float min;
            float max;
        };
        auto min_max = [](const float a, const float b) -> MinMaxResult
        {
            return a > b ? MinMaxResult{b, a} : MinMaxResult{a, b};
        };

        // mTopFrontLeft is the corner of AABB with minimal coordinates - left top.
        const float t1 = (mMinimum.x - ray.mOrigin.x) * ray.mInverseDirection.x;
        const float t2 = (mMaximum.x - ray.mOrigin.x) * ray.mInverseDirection.x;
        const float t3 = (mMinimum.y - ray.mOrigin.y) * ray.mInverseDirection.y;
        const float t4 = (mMaximum.y - ray.mOrigin.y) * ray.mInverseDirection.y;
        const float t5 = (mMinimum.z - ray.mOrigin.z) * ray.mInverseDirection.z;
        const float t6 = (mMaximum.z - ray.mOrigin.z) * ray.mInverseDirection.z;

        // Keeping the old code here commented out as it's much easier to read than the newer optimised version.
        //float tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
        //float tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

        const auto [a1, b1] = min_max(t1, t2);
        const auto [a2, b2] = min_max(t3, t4);
        const auto [a3, b3] = min_max(t5, t6);

        const float tmin = std::max(std::max(a1, a2), a3);
        const float tmax = std::min(std::min(b1, b2), b3);

        // if tmax < 0, ray the ray intersects AABB, but the AABB is behind the ray (direction faces away)
        // or no intersection at all.
        if (tmax < 0 || tmin > tmax)
        {
            return INFINITY;
        }

        return tmin;
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

    void AABB::add_point(const glm::vec4& p)
    {
        mMinimum = component_wise_min(mMinimum, p);
        mMaximum = component_wise_max(mMaximum, p);
    }

    void AABB::union_of(const AABB& aabb)
    {
        mMinimum = component_wise_min(mMinimum, aabb.get_min());
        mMaximum = component_wise_max(mMaximum, aabb.get_max());
    }

    AABB AABB::union_of(const AABB& lhs, const AABB& rhs)
    {
        AABB aabb = lhs;

        aabb.union_of(rhs);
        return aabb;
    }

    glm::vec3 AABB::get_offset(const glm::vec4& p)
    {
        glm::vec3 o = p - mMinimum;
        if (mMaximum.x > mMinimum.x) o.x /= mMaximum.x - mMinimum.x;
        if (mMaximum.y > mMinimum.y) o.y /= mMaximum.y - mMinimum.y;
        if (mMaximum.z > mMinimum.z) o.z /= mMaximum.z - mMinimum.z;
        return o;
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
