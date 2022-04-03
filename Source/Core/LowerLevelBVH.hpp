#ifndef LOWER_LEVEL_BVH_HPP
#define LOWER_LEVEL_BVH_HPP

#include "glm/common.hpp"
#include "AABB.hpp"
#include "Core/RandUtils.hpp"
#include "Render/BSRDF.hpp"

#include <memory>

namespace Core
{

    namespace BVH
    {

        struct InterpolatedVertex
        {
            glm::vec4 mPosition;
            glm::vec2 mUV;
            glm::vec3 mNormal;
            glm::vec4 mVertexColour;
            std::shared_ptr<Render::BSRDF> m_bsrdf;
        };

        class LowerLevelBVH
        {
        public:

            LowerLevelBVH() = default;
            virtual ~LowerLevelBVH() = default;

            virtual bool calculate_intersection(const Ray&, InterpolatedVertex* result) const = 0;

            virtual AABB get_bounds() const = 0;

            // Light sampling methods.
            virtual void generate_sampling_data() = 0;

            virtual bool sample_geometry(Core::Rand::Hammersley_Generator&, const glm::vec3& point, const glm::vec3& N, glm::vec3& sample_point, float& solid_angle) = 0;
        };

    }

}

#endif
