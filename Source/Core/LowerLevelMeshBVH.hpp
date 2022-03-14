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

        private:

            bool trace_ray(const nanort::Ray<float>& ray, Core::BVH::InterpolatedVertex* result) const;

            InterpolatedVertex interpolate_fragment(const uint32_t primID, const float u, const float v) const;

            std::vector<glm::vec3> mPositions;
            std::vector<glm::vec2> mUVs;
            std::vector<glm::vec3> mNormals;
            std::vector<glm::vec4> mVertexColours;
            std::vector<uint32_t>  mIndicies;

            std::unique_ptr<nanort::TriangleMesh<float>> mMeshes;
            std::unique_ptr<nanort::TriangleSAHPred<float>> mPred;

            nanort::BVHAccel<float> mAccelerationStructure;

            AABB mAABB;

        };

    }

}


#endif
