#ifndef LOWER_LEVEL_MESH_BVH_HPP
#define LOWER_LEVEL_MESH_BVH_HPP

#include "LowerLevelBVH.hpp"

#define NANORT_USE_CPP11_FEATURE 1
#include "ThirdParty/nanort/nanort.h"
#include "assimp/mesh.h"

#include <vector>

namespace Core
{

    namespace BVH
    {

        class LowerLevelMeshBVH : public LowerLevelBVH
        {
        public:

            LowerLevelMeshBVH(const aiMesh*);

            virtual bool calculate_intersection(const Ray&, InterpolatedVertex* result) const;

            virtual AABB get_bounds() const final
            {
                return mAABB;
            }

            virtual void generate_sampling_data() final;

            virtual bool sample_geometry(Core::Rand::Hammersley_Generator& Xi, const glm::vec3& point, const glm::vec3& N,  glm::vec3&, float&) final;

        private:

            bool trace_ray(const nanort::Ray<float>& ray, Core::BVH::InterpolatedVertex* result) const;

            InterpolatedVertex interpolate_fragment(const uint32_t primID, const float u, const float v) const;

            std::string m_name;

            std::vector<glm::vec3> mPositions;
            std::vector<glm::vec2> mUVs;
            std::vector<glm::vec3> mNormals;
            std::vector<glm::vec4> mVertexColours;
            std::vector<uint32_t>  mIndicies;

            // Data used for light sampling.
            struct TriangleFace
            {
                glm::vec3 m_normal;
                float m_area;
            };
            std::vector<TriangleFace> m_triangle_faces;

            std::unique_ptr<nanort::TriangleMesh<float>> mMeshes;
            std::unique_ptr<nanort::TriangleSAHPred<float>> mPred;

            nanort::BVHAccel<float> mAccelerationStructure;

            AABB mAABB;

        };

    }

}


#endif
