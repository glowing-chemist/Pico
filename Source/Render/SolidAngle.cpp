#include "SolidAngle.hpp"


namespace Render
{
    float solid_angle_from_bounds(const Core::AABB& bounds, const glm::vec3& pos)
    {
        const glm::vec3 bounds_center = bounds.get_central_point();
        const glm::vec3 bounds_size = bounds.get_side_lengths();
        const float bounds_sphere_radius = std::max(bounds_size.x, std::max(bounds_size.y, bounds_size.z)) / 2.0f;
        const float area = M_PI * bounds_sphere_radius * bounds_sphere_radius;

        return solid_angle(pos, bounds_center, glm::normalize(pos - bounds_center), area);
    }

    float solid_angle(const glm::vec3& pos, const glm::vec3& intersect_point, const glm::vec3& intersect_normal, const float area)
    {
        const glm::vec3 to_point = intersect_point - pos;
        const float distance_squared = glm::length(to_point) * glm::length(to_point);
        const glm::vec3 wi = glm::normalize(to_point);

        return (std::abs(glm::dot(intersect_normal, -wi)) * area) / distance_squared;
    }
}
