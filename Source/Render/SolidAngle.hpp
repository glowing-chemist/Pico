#ifndef SOLID_ANGLE_HPP
#define SOLID_ANGLE_HPP

#include "Core/AABB.hpp"

namespace Render
{

    float solid_angle_from_bounds(const Core::AABB& bounds, const glm::vec3& pos);

}

#endif
