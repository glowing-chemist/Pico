#ifndef SOLID_ANGLE_HPP
#define SOLID_ANGLE_HPP

#include "Core/AABB.hpp"

#include "glm/glm.hpp"

namespace Render
{
    float solid_angle_from_bounds(const Core::AABB& bounds, const glm::vec3& pos);

    float solid_angle(const glm::vec3& pos, const glm::vec3& intersect_point, const glm::vec3& intersect_normal, const float area);
}

#endif
