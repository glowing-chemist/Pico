#ifndef AABB_HPP
#define AABB_HPP

#include <array>

#include "glm/vec4.hpp"
#include "glm/mat4x4.hpp"

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
    };

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

        Cube getCube() const;
        std::array<glm::vec4, 8> getCubeAsVertexArray() const;

        // std::limits<float>::max() to indicate no intersection
        float intersectionDistance(const Ray&) const;

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

        const glm::vec4& getMax() const
        { return mMaximum; }

        const glm::vec4& getMin() const
        { return mMinimum; }

        glm::vec4 getCentralPoint() const
        { return mMinimum + (mMaximum - mMinimum) * 0.5f; }

        glm::vec3 getSideLengths() const
        {
            return glm::abs(mMaximum - mMinimum);
        }

    private:

        glm::vec4 mMinimum;
        glm::vec4 mMaximum;

    };

}

#endif
