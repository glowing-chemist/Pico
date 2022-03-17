#ifndef MATERIAL_MANAGER_HPP
#define MATERIAL_MANAGER_HPP

#include <memory>
#include <vector>

#include "glm/glm.hpp"
#include "glm/vec3.hpp"
#include "glm/vec4.hpp"

#include "Loadable.hpp"


namespace Core
{

    struct EvaluatedMaterial
    {
        glm::vec4 diffuse;
        glm::vec4 specularRoughness; // xyz specular w roughness
        glm::vec3 normal;
        glm::vec3 emissive;

        bool emits_light() const
        {
            return (emissive.x > 0.0f) || (emissive.y > 0.0f) || (emissive.z > 0.0f);
        }

        float diffuse_magnitude() const
        {
            return glm::length(glm::vec3(diffuse));
        }

        float specular_magnitude() const
        {
            return glm::length(glm::vec3(specularRoughness.x, specularRoughness.y, specularRoughness.z));
        }
    };


    // Material / shader interface.
    class Material : public Loadable
    {
    public:

        Material() = default;
        virtual ~Material() = default;

        virtual EvaluatedMaterial evaluate_material(const glm::vec2& uv) = 0;
    };

    // TODO add memory constraints.
    class MaterialManager
    {
    public:

        MaterialManager() = default;
        ~MaterialManager();


        using MaterialID = uint64_t;


        MaterialID add_material(std::unique_ptr<Material>&);

        EvaluatedMaterial evaluate_material(const MaterialID id, const glm::vec2& uv);

    private:

        std::vector<std::unique_ptr<Material>> mMaterials;
    };

}

#endif
