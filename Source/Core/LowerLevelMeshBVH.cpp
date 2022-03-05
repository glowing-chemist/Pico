#include "LowerLevelMeshBVH.hpp"

namespace Core
{

    namespace BVH
    {

        LowerLevelMeshBVH::LowerLevelMeshBVH(const aiMesh* mesh) :
            LowerLevelBVH()
        {
            // Copy the index data
            for(uint32_t i = 0; i < mesh->mNumFaces; ++i)
            {
                for(uint32_t j = 0; j < mesh->mFaces[i].mNumIndices; ++j)
                {
                    mIndicies.push_back(mesh->mFaces[i].mIndices[j]);
                }
            }

            static_assert (sizeof(glm::vec3) == sizeof(aiVector3t<float>), "position size mismatch");
            mPositions.resize(mesh->mNumVertices);
            memcpy(mPositions.data(), mesh->mVertices, sizeof(glm::vec3) * mesh->mNumVertices);

            static_assert (sizeof(glm::vec2) == sizeof(aiVector2t<float>), "uv size mismatch");
            mUVs.resize(mesh->mNumVertices);
            memcpy(mUVs.data(), mesh->mTextureCoords[0], sizeof(glm::vec2) * mesh->mNumVertices);

            static_assert (sizeof(glm::vec3) == sizeof(aiVector3t<float>), "normals size mismatch");
            mNormals.resize(mesh->mNumVertices);
            memcpy(mNormals.data(), mesh->mNormals, sizeof(glm::vec3) * mesh->mNumVertices);

            static_assert (sizeof(glm::vec4) == sizeof(aiColor4D), "colour size mismatch");
            mVertexColours.resize(mesh->mNumVertices);
            memcpy(mVertexColours.data(), mesh->mColors, sizeof(glm::vec4) * mesh->mNumVertices);

            mMeshes = std::make_unique<nanort::TriangleMesh<float>>(reinterpret_cast<float*>(mPositions.data()), mIndicies.data(), sizeof(glm::vec3));
            mPred = std::make_unique<nanort::TriangleSAHPred<float>>(reinterpret_cast<float*>(mPositions.data()), mIndicies.data(), sizeof(glm::vec3));

            mAccelerationStructure.Build(mIndicies.size() / 3, *mMeshes, *mPred);
        }

        bool LowerLevelMeshBVH::calculate_intersection(const Ray& ray, InterpolatedVertex* result) const
        {
            nanort::Ray<float> newRay{};
            newRay.org[0] = ray.mOrigin.x;
            newRay.org[1] = ray.mOrigin.y;
            newRay.org[2] = ray.mOrigin.z;
            newRay.dir[0] = ray.mDirection.x;
            newRay.dir[1] = ray.mDirection.y;
            newRay.dir[2] = ray.mDirection.z;
            newRay.min_t = 0.01f;
            newRay.max_t = 2000.0f;

            return trace_ray(newRay, result);
        }

        bool LowerLevelMeshBVH::trace_ray(const nanort::Ray<float>& ray, InterpolatedVertex *result) const
        {
            nanort::TriangleIntersector triangle_intersecter(reinterpret_cast<const float*>(mPositions.data()), mIndicies.data(), sizeof(glm::vec3));
            nanort::TriangleIntersection intersection;
            const bool hit = mAccelerationStructure.Traverse(ray, triangle_intersecter, &intersection);

            if(hit)
            {
                *result = interpolate_fragment(intersection.prim_id, intersection.u, intersection.v);
            }

            return hit;
        }

        InterpolatedVertex LowerLevelMeshBVH::interpolate_fragment(const uint32_t primID, const float u, const float v) const
        {
            const uint32_t baseIndiciesIndex = primID * 3;

            const uint32_t firstIndex = mIndicies[baseIndiciesIndex];
            const glm::vec3& firstPosition = mPositions[firstIndex];
            const glm::vec2& firstuv = mUVs[firstIndex];
            const glm::vec3& firstNormal = mNormals[firstIndex];
            const glm::vec4 firstColour = mVertexColours[firstIndex];

            const uint32_t secondIndex = mIndicies[baseIndiciesIndex + 1];
            const glm::vec3& secondPosition = mPositions[secondIndex];
            const glm::vec2& seconduv = mUVs[secondIndex];
            const glm::vec3& secondNormal = mNormals[secondIndex];
            const glm::vec4& secondColour = mVertexColours[secondIndex];

            const uint32_t thirdIndex = mIndicies[baseIndiciesIndex + 2];
            const glm::vec3& thirdPosition = mPositions[thirdIndex];
            const glm::vec2& thirduv = mUVs[thirdIndex];
            const glm::vec3& thirdNormal = mNormals[thirdIndex];
            const glm::vec4& thirdColour = mVertexColours[thirdIndex];

            InterpolatedVertex frag{};
            frag.mPosition = glm::vec4(((1.0f - v - u) * firstPosition) + (u * secondPosition) + (v * thirdPosition), 1.0f);
            frag.mUV = ((1.0f - v - u) * firstuv) + (u * seconduv) + (v * thirduv);
            frag.mNormal = glm::normalize(glm::vec3(((1.0f - v - u) * firstNormal) + (u * secondNormal) + (v * thirdNormal)));
            frag.mVertexColour = ((1.0f - v - u) * firstColour) + (u * secondColour) + (v * thirdColour);
            frag.mPrimID = primID;

            return frag;
        }

    }

}
