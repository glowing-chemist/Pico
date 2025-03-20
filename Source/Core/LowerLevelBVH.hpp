#ifndef LOWER_LEVEL_BVH_HPP
#define LOWER_LEVEL_BVH_HPP

#include "AABB.hpp"
#include "Core/MaterialManager.hpp"
#include "Core/RandUtils.hpp"
#include "Render/BSRDF.hpp"

namespace Core
{

    namespace Acceleration_Structures
    {

        struct InterpolatedVertex
        {
            glm::vec4 mPosition;
            glm::vec2 mUV;
            glm::vec3 mNormal;
            glm::vec4 mVertexColour;
            Render::BSRDF* m_bsrdf;

            float direct_lighting_pdf(const glm::vec3& wi, const glm::vec3& wo, const EvaluatedMaterial&) const;
        };

        class LowerLevelBVH
        {
        public:

            LowerLevelBVH() = default;
            virtual ~LowerLevelBVH() = default;

            virtual bool calculate_intersection(Ray&, InterpolatedVertex* result) const = 0;

            virtual AABB get_bounds() const = 0;

            // Light sampling methods.
            virtual void generate_sampling_data() = 0;

            virtual bool sample_geometry(Core::Rand::Hammersley_Generator&, glm::vec3& sample_point, float& pdf) = 0;
        };

    }

}

#endif
