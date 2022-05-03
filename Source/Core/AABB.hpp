#ifndef AABB_HPP
#define AABB_HPP

#include <array>
#include <vector>

#include "glm/vec4.hpp"
#include "glm/mat4x4.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Core
{

    enum Intersection : uint8_t
    {
        None = 0,
        Contains = 1 << 1,
        Partial = 1 << 2,
    };

    struct Ray
    {
        glm::vec4 mOrigin;
        glm::vec3 mDirection;
        float     mLenght;

        glm::vec4 m_payload;
        float     m_weight;

        void push_index_of_refraction(const float);
        float pop_index_of_refraction();
        float get_current_index_of_refraction() const;

    private:

        std::vector<float> m_index_or_refraction_stack;
    };

    Ray transform_ray(const Ray&, const glm::mat4x4& transform);

    // Return a vector constructed from the minimum/maximum of each component.
    glm::vec3 component_wise_min(const glm::vec3& lhs, const glm::vec3& rhs);

    glm::vec4 component_wise_min(const glm::vec4& lhs, const glm::vec4& rhs);

    glm::vec3 component_wise_max(const glm::vec3& lhs, const glm::vec3& rhs);

    glm::vec4 component_wise_max(const glm::vec4& lhs, const glm::vec4& rhs);


    // 8 float4 representing the 8 verticies of a cude
    struct Cube
    {
        // Verticies going clockwise looking down (perpendicular) to a single face
        glm::vec4 mUpper1; // Corresponds to TopFrontLeft
        glm::vec4 mUpper2;
        glm::vec4 mUpper3;
        glm::vec4 mUpper4;

        glm::vec4 mLower1;
        glm::vec4 mLower2;
        glm::vec4 mLower3; // Corresponds to BottomBackRight
        glm::vec4 mLower4;
    };


    // An axis aligned bounding box, using the vulkan corrdinate system (origin in the top left corner).
    class AABB
    {
    public:
        AABB(const glm::vec4& diagonalTop, const glm::vec4& diagonalBottom) :
            mMinimum{diagonalTop},
            mMaximum{diagonalBottom} {}

        AABB(const Cube& cube) :
            mMinimum{cube.mUpper1},
            mMaximum{cube.mLower3} {}

        // Allow a zero sized AABB to be constructed
        AABB() = default;

        Cube get_cube() const;
        std::array<glm::vec4, 8> get_cube_as_vertex_array() const;

        // std::limits<float>::max() to indicate no intersection
        std::pair<float, float> intersection_distances(const Ray&) const;

        bool contains(const glm::vec4&) const;
        Intersection contains(const AABB&) const;

        AABB& operator*=(const glm::mat4x4&);

        AABB& operator*=(const glm::vec4&);
        AABB& operator+=(const glm::vec4&);
        AABB& operator-=(const glm::vec4&);

        AABB operator*(const glm::mat4&) const;

        AABB operator*(const glm::vec4&) const;
        AABB operator+(const glm::vec4&) const;
        AABB operator-(const glm::vec4&) const;

        const glm::vec4& get_max() const
        { return mMaximum; }

        const glm::vec4& get_min() const
        { return mMinimum; }

        glm::vec4 get_central_point() const
        { return mMinimum + (mMaximum - mMinimum) * 0.5f; }

        glm::vec3 get_side_lengths() const
        {
            return glm::abs(mMaximum - mMinimum);
        }

    private:

        glm::vec4 mMinimum;
        glm::vec4 mMaximum;

    };

}

#endif
