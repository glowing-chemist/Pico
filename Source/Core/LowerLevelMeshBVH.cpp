#include "LowerLevelMeshBVH.hpp"
#include "Render/SolidAngle.hpp"
#include "Core/Asserts.hpp"

namespace Core
{

    namespace Acceleration_Structures
    {

        LowerLevelMeshBVH::LowerLevelMeshBVH(const aiMesh* mesh) :
            LowerLevelBVH()
        {
            m_name = mesh->mName.C_Str();

            // Copy the index data
            mIndicies.reserve(mesh->mNumFaces * 3);
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
            if (mesh->mColors[0])
            {
                mVertexColours.resize(mesh->mNumVertices);
                memcpy(mVertexColours.data(), mesh->mColors[0], sizeof(glm::vec4) * mesh->mNumVertices);
            }

            std::vector<LOWER_ACCELERATION_STRUCTURE::BoundedValue> primitive_bounds;
            primitive_bounds.reserve(mIndicies.size());
            for(uint32_t i = 0; i < mIndicies.size(); i += 3)
            {
                glm::vec3 min = glm::vec3(INFINITY, INFINITY, INFINITY);
                glm::vec3 max = glm::vec3(-INFINITY, -INFINITY, -INFINITY);

                min = Core::component_wise_min(min, mPositions[mIndicies[i]]);
                min = Core::component_wise_min(min, mPositions[mIndicies[i + 1]]);
                min = Core::component_wise_min(min, mPositions[mIndicies[i + 2]]);

                max = Core::component_wise_max(max, mPositions[mIndicies[i]]);
                max = Core::component_wise_max(max, mPositions[mIndicies[i + 1]]);
                max = Core::component_wise_max(max, mPositions[mIndicies[i + 2]]);

                primitive_bounds.push_back({AABB{glm::vec4(min, 1.0f), glm::vec4(max, 1.0f)}, i / 3});
            }

            aiAABB aabb = mesh->mAABB;
            mAABB = AABB({aabb.mMin.x, aabb.mMin.y, aabb.mMin.z, 1.0f}, {aabb.mMax.x, aabb.mMax.y, aabb.mMax.z, 1.0f});

#ifdef USE_OCTTREE
            m_acceleration_structure = OctTreeFactory<uint32_t>(mAABB, primitive_bounds)
                                           .set_intersector(std::make_unique<Mesh_Intersector>(mPositions.data(), mUVs.data(), mNormals.data(), mVertexColours.data(), mIndicies.data()))
                                           .generate_octTree();
#else
            m_acceleration_structure = BVHFactory<uint32_t>(mAABB, primitive_bounds)
                                           .set_intersector(std::make_unique<Mesh_Intersector>(mPositions.data(), mUVs.data(), mNormals.data(), mVertexColours.data(), mIndicies.data()))
                                           .set_parition_scheme(std::make_unique<SAH_parition_Scheme<uint32_t>>())
                                           .generate_BVH();
#endif
        }

        bool LowerLevelMeshBVH::calculate_intersection(const Ray& ray, InterpolatedVertex* result) const
        {
            InterpolatedVertex vertex{};
            const bool intersects = m_acceleration_structure->get_first_intersection(ray, vertex);

            if(intersects)
                *result = vertex;

            return intersects;
        }

        void LowerLevelMeshBVH::generate_sampling_data()
        {
            m_total_surface_area = 0.0f;

            m_triangle_area.reserve(mIndicies.size() / 3);
            for(uint32_t i_index = 0; i_index < mIndicies.size(); i_index += 3)
            {
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

                const float triangle_area = (base * height) / 2.0f;
                m_total_surface_area += triangle_area;

                m_triangle_area.push_back(triangle_area);
            }

            PICO_LOG("Generating sampling data for %s. %zu faces generated\n", m_name.c_str(), m_triangle_area.size());
        }

        bool LowerLevelMeshBVH::sample_geometry(Rand::Hammersley_Generator& rand, const glm::vec3&, const glm::vec3&, glm::vec3& sample_point, float& pdf)
        {
            const glm::vec2 xi = rand.next();
            const uint32_t triangle_index = xi.y * m_triangle_area.size();

            const glm::vec2 Xi = rand.next();
            const glm::vec2 barycentrics = Core::Rand::uniform_sample_triangle(Xi);

            const uint32_t index_start = 3 * triangle_index;
            sample_point = ((1.0f - barycentrics.x - barycentrics.y) * mPositions[mIndicies[index_start]] +
                (barycentrics.x * mPositions[mIndicies[index_start + 1]])) +
                (barycentrics.y * mPositions[mIndicies[index_start + 2]]);

            pdf = m_triangle_area[triangle_index] / m_total_surface_area;

            return true;
        }

        InterpolatedVertex LowerLevelMeshBVH::Mesh_Intersector::interpolate_fragment(const uint32_t primID, const float u, const float v) const
        {
            const uint32_t baseIndiciesIndex = primID * 3;

            const uint32_t firstIndex = m_indicies[baseIndiciesIndex];
            const glm::vec3& firstPosition = m_positions[firstIndex];
            const glm::vec2& firstuv = !m_uvs ? glm::vec2(0.0f) : m_uvs[firstIndex];
            const glm::vec3& firstNormal = m_normals[firstIndex];
            const glm::vec4 firstColour = !m_colours ? glm::vec4(1, 1, 1, 1) : m_colours[firstIndex];

            const uint32_t secondIndex = m_indicies[baseIndiciesIndex + 1];
            const glm::vec3& secondPosition = m_positions[secondIndex];
            const glm::vec2& seconduv = !m_uvs ? glm::vec2(0.0f) : m_uvs[secondIndex];
            const glm::vec3& secondNormal = m_normals[secondIndex];
            const glm::vec4& secondColour = !m_colours ? glm::vec4(1, 1, 1, 1) : m_colours[secondIndex];

            const uint32_t thirdIndex = m_indicies[baseIndiciesIndex + 2];
            const glm::vec3& thirdPosition = m_positions[thirdIndex];
            const glm::vec2& thirduv = !m_uvs ? glm::vec2(0.0f) : m_uvs[thirdIndex];
            const glm::vec3& thirdNormal = m_normals[thirdIndex];
            const glm::vec4& thirdColour = !m_colours ? glm::vec4(1, 1, 1, 1) : m_colours[thirdIndex];

            InterpolatedVertex frag{};
            frag.mPosition = glm::vec4(((1.0f - v - u) * firstPosition) + (u * secondPosition) + (v * thirdPosition), 1.0f);
            frag.mUV = ((1.0f - v - u) * firstuv) + (u * seconduv) + (v * thirduv);
            frag.mNormal = glm::normalize(glm::vec3(((1.0f - v - u) * firstNormal) + (u * secondNormal) + (v * thirdNormal)));
            frag.mVertexColour = ((1.0f - v - u) * firstColour) + (u * secondColour) + (v * thirdColour);

            return frag;
        }

        bool LowerLevelMeshBVH::Mesh_Intersector::intersects(const Ray& ray, uint32_t primID, float& distance, InterpolatedVertex& vertex) const
        {
            const float EPSILON = 0.0000001f;
            const glm::vec3 vertex0 = m_positions[m_indicies[primID * 3]];
            const glm::vec3 vertex1 = m_positions[m_indicies[(primID * 3) + 1]];
            const glm::vec3 vertex2 = m_positions[m_indicies[(primID * 3) + 2]];

            const glm::vec3 edge1 = vertex1 - vertex0;
            const glm::vec3 edge2 = vertex2 - vertex0;
            const glm::vec3 h = glm::cross(ray.mDirection, edge2);
            const float a = glm::dot(edge1, h);

            if (a > -EPSILON && a < EPSILON)
                return false;    // This ray is parallel to this triangle.

            const float f = 1.0 / a;
            const glm::vec3 s = glm::vec3(ray.mOrigin.x, ray.mOrigin.y, ray.mOrigin.z) - vertex0;
            const float u = f * glm::dot(s, h);

            if (u < 0.0 || u > 1.0)
                return false;

            glm::vec3 q = glm::cross(s, edge1);
            const float v = f * glm::dot(ray.mDirection, q);

            if (v < 0.0 || u + v > 1.0)
                return false;

            // At this stage we can compute t to find out where the intersection point is on the line.
            const float t = f * glm::dot(edge2, q);

            if (t > EPSILON) // ray intersection
            {
                distance = t;

                vertex = interpolate_fragment(primID, u, v);

                return true;
            }


            return false;
        }

    }

}
