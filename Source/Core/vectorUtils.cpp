#include "vectorUtils.hpp"
#include "Asserts.hpp"

namespace Core
{

    namespace TangentSpace
    {
        glm::mat3x3 construct_world_to_tangent_transform(const glm::vec3& V, const glm::vec3& N)
        {
            PICO_ASSERT_VALID(N);
            PICO_ASSERT_VALID(V);
            // If N and V aparralle of anti parralle we shoudl use a different vector for calculatign the tangent frame.
            glm::vec3 tangent;
            if(std::abs(glm::dot(V, N)) > 0.95f)
            {
                glm::vec3 v_vec = std::abs(N.z) > 0.99f ? glm::vec3(1, 0, 0) : glm::vec3(0, 0, 1);

                PICO_ASSERT(!std::isnan(glm::dot(v_vec, N)));

                tangent = glm::normalize(glm::cross(v_vec, N));

                PICO_ASSERT_VALID(tangent);
            }
            else
            {
                tangent = glm::normalize(glm::cross(V, N));
                PICO_ASSERT_VALID(tangent);
            }

            const glm::vec3 bitangent = glm::normalize(glm::cross(tangent, N));
            PICO_ASSERT_VALID(bitangent);

            return glm::transpose(glm::mat3x3(tangent, bitangent, N));
        }
    }

}
