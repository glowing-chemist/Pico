#include "LowerLevelMeshBVH.hpp"
#include "Render/SolidAngle.hpp"
#include "Core/Asserts.hpp"

namespace Core
{

    namespace BVH
    {

        LowerLevelMeshBVH::LowerLevelMeshBVH(const aiMesh* mesh) :
            LowerLevelBVH()
        {
            m_name = mesh->mName.C_Str();

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

            if(mesh->mTextureCoords[0])
            {
                mUVs.resize(mesh->mNumVertices);
                for(uint32_t i = 0; i < mesh->mNumVertices; ++i)
                    mUVs[i] = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
            }

            static_assert (sizeof(glm::vec3) == sizeof(aiVector3t<float>), "normals size mismatch");
            mNormals.resize(mesh->mNumVertices);
            memcpy(mNormals.data(), mesh->mNormals, sizeof(glm::vec3) * mesh->mNumVertices);

            static_assert (sizeof(glm::vec4) == sizeof(aiColor4D), "colour size mismatch");
            mVertexColours.resize(mesh->mNumVertices);
            memcpy(mVertexColours.data(), mesh->mColors, sizeof(glm::vec4) * mesh->mNumVertices);

            mMeshes = std::make_unique<nanort::TriangleMesh<float>>(reinterpret_cast<float*>(mPositions.data()), mIndicies.data(), sizeof(glm::vec3));
            mPred = std::make_unique<nanort::TriangleSAHPred<float>>(reinterpret_cast<float*>(mPositions.data()), mIndicies.data(), sizeof(glm::vec3));

            mAccelerationStructure.Build(mIndicies.size() / 3, *mMeshes, *mPred);

            aiAABB aabb = mesh->mAABB;
            mAABB = AABB({aabb.mMin.x, aabb.mMin.y, aabb.mMin.z, 1.0f}, {aabb.mMax.x, aabb.mMax.y, aabb.mMax.z, 1.0f});
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
            newRay.min_t = 0.0f;
            newRay.max_t = ray.mLenght;

            return trace_ray(newRay, result);
        }

        void LowerLevelMeshBVH::generate_sampling_data()
        {
            for(uint32_t i_index = 0; i_index < mIndicies.size(); i_index += 3)
            {
                TriangleFace face{};
                face.m_normal = glm::normalize(mNormals[mIndicies[i_index]] + mNormals[mIndicies[i_index + 1]] + mNormals[mIndicies[i_index + 1]]);

                // calculate triangle area.
                const glm::vec3 a = mPositions[mIndicies[i_index]];
                const glm::vec3 b = mPositions[mIndicies[i_index + 1]];
                const glm::vec3 c = mPositions[mIndicies[i_index + 2]];

                const glm::vec3 ac = a - c;
                const glm::vec3 bc = b - c;
                const float bc_length = glm::length(bc);

                const float base = glm::length(ac);
                const float theta = std::sin(std::acos(glm::dot(glm::normalize(ac), glm::normalize(bc))) * (180.0f / M_PI));
                const float height = theta * bc_length;

                face.m_area = (base * height) / 2.0f;

                m_triangle_faces.push_back(face);
            }

            PICO_LOG("Generating sampling data for %s. %zu faces generated\n", m_name.c_str(), m_triangle_faces.size());
        }

        bool LowerLevelMeshBVH::sample_geometry(Rand::Hammersley_Generator& rand, const glm::vec3& point, const glm::vec3& N, glm::vec3& sample_point, float& solid_angle)
        {
            std::vector<float> sample_solid_angle{};
            sample_solid_angle.reserve(m_triangle_faces.size());
            std::vector<glm::vec3> sample_positions{};
            sample_positions.reserve(m_triangle_faces.size());

            float total_solid_angle = 0.0f;
            for(uint32_t i_face = 0; i_face < m_triangle_faces.size(); ++i_face)
            {
                const TriangleFace& face = m_triangle_faces[i_face];

                if(glm::dot(-N, face.m_normal) > 0.0f)
                {
                    const glm::vec2 Xi = rand.next();
                    const glm::vec2 barycentrics = Core::Rand::uniform_sample_triangle(Xi);

                    const uint32_t index_start = 3 * i_face;
                    const glm::vec3 sampled_pos = ((1.0f - barycentrics.x - barycentrics.y) * mPositions[mIndicies[index_start]] +
                                                   (barycentrics.x * mPositions[mIndicies[index_start + 1]])) +
                                                    (barycentrics.y * mPositions[mIndicies[index_start + 2]]);

                    glm::vec3 wi = glm::normalize(sampled_pos - point);

                    if(glm::dot(face.m_normal, -wi) > 0.0f)
                    {
                        sample_positions.push_back(sampled_pos);

                        const float face_solid_angle = Render::solid_angle(point, sampled_pos, face.m_normal, face.m_area);
                        total_solid_angle += face_solid_angle;
                        sample_solid_angle.push_back(face_solid_angle);
                    }
                }
            }

            const glm::vec2 Xi = rand.next();
            uint32_t sample_index = Core::Rand::choose(Xi.y, sample_solid_angle, total_solid_angle);
            if(sample_index != UINT_MAX)
            {
                sample_point = sample_positions[sample_index];
                solid_angle = total_solid_angle;
                return true;
            }

            return false;
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
            const glm::vec2& firstuv = mUVs.empty() ? glm::vec2(0.0f) : mUVs[firstIndex];
            const glm::vec3& firstNormal = mNormals[firstIndex];
            const glm::vec4 firstColour = mVertexColours[firstIndex];

            const uint32_t secondIndex = mIndicies[baseIndiciesIndex + 1];
            const glm::vec3& secondPosition = mPositions[secondIndex];
            const glm::vec2& seconduv = mUVs.empty() ? glm::vec2(0.0f) : mUVs[secondIndex];
            const glm::vec3& secondNormal = mNormals[secondIndex];
            const glm::vec4& secondColour = mVertexColours[secondIndex];

            const uint32_t thirdIndex = mIndicies[baseIndiciesIndex + 2];
            const glm::vec3& thirdPosition = mPositions[thirdIndex];
            const glm::vec2& thirduv = mUVs.empty() ? glm::vec2(0.0f) : mUVs[thirdIndex];
            const glm::vec3& thirdNormal = mNormals[thirdIndex];
            const glm::vec4& thirdColour = mVertexColours[thirdIndex];

            InterpolatedVertex frag{};
            frag.mPosition = glm::vec4(((1.0f - v - u) * firstPosition) + (u * secondPosition) + (v * thirdPosition), 1.0f);
            frag.mUV = ((1.0f - v - u) * firstuv) + (u * seconduv) + (v * thirduv);
            frag.mNormal = glm::normalize(glm::vec3(((1.0f - v - u) * firstNormal) + (u * secondNormal) + (v * thirdNormal)));
            frag.mVertexColour = ((1.0f - v - u) * firstColour) + (u * secondColour) + (v * thirdColour);

            return frag;
        }

    }

}
