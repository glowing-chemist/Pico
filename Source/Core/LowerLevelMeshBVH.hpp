#ifndef LOWER_LEVEL_MESH_BVH_HPP
#define LOWER_LEVEL_MESH_BVH_HPP

#include "LowerLevelBVH.hpp"
#include "Core/BVH.hpp"

#include "assimp/mesh.h"

#include <vector>

#define LOWER_ACCELERATION_STRUCTURE BVH<uint32_t, 2>

namespace Core
{

    namespace Acceleration_Structures
    {

        class LowerLevelMeshBVH : public LowerLevelBVH
        {
        public:

            LowerLevelMeshBVH(const aiMesh*);

            virtual bool calculate_intersection(Ray&, InterpolatedVertex* result) const;

            virtual AABB get_bounds() const final
            {
                return mAABB;
            }

            virtual void generate_sampling_data() final;

            virtual bool sample_geometry(Core::Rand::Hammersley_Generator& Xi, const glm::vec3& point, const glm::vec3& N,  glm::vec3&, float&) final;

        private:

            std::string m_name;

            std::vector<glm::vec3> mPositions;
            std::vector<glm::vec2> mUVs;
            std::vector<glm::vec3> mNormals;
            std::vector<glm::vec4> mVertexColours;
            std::vector<uint32_t>  mIndicies;

            // Data used for light sampling.
            std::vector<float> m_triangle_area;
            float m_total_surface_area;

            class Mesh_Intersector : public Intersector<uint32_t>
            {
                public:

                Mesh_Intersector(glm::vec3* positions, glm::vec2* uvs, glm::vec3* normals, glm::vec4* colours, uint32_t* indicies) :
                        m_positions(positions),
                        m_uvs(uvs),
                        m_normals(normals),
                        m_colours(colours),
                        m_indicies(indicies) {}

                virtual bool intersects(const Ray&, uint32_t, float& distance, InterpolatedVertex&) const override;

            private:

                InterpolatedVertex interpolate_fragment(const uint32_t primID, const float u, const float v) const;

                glm::vec3* m_positions;
                glm::vec2* m_uvs;
                glm::vec3* m_normals;
                glm::vec4* m_colours;
                uint32_t* m_indicies;
            };

            LOWER_ACCELERATION_STRUCTURE* m_acceleration_structure;

            AABB mAABB;

        };

    }

}


#endif
