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
        glm::vec3 diffuse;
        glm::vec3 specular;
        float     roughness;
        glm::vec3 normal;
        glm::vec3 emissive;

        bool emits_light() const
        {
            return (emissive.x > 0.0f) || (emissive.y > 0.0f) || (emissive.z > 0.0f);
        }

        float diffuse_magnitude() const
        {
            return glm::length(diffuse);
        }

        float specular_magnitude() const
        {
            return glm::length(specular);
        }

        float get_reflectance() const
        {
            const float specular = specular_magnitude();

            return specular / (specular + diffuse_magnitude());
        }
    };


    // Material / shader interface.
    class Material : public Loadable
    {
    public:

        Material() = default;
        virtual ~Material() = default;

        virtual EvaluatedMaterial evaluate_material(const glm::vec2& uv) const = 0;

        virtual bool              is_light() const = 0;
    };

    // TODO add memory constraints.
    class MaterialManager
    {
    public:

        MaterialManager() = default;
        ~MaterialManager();


        using MaterialID = uint64_t;


        MaterialID add_material(std::unique_ptr<Material>&);

        EvaluatedMaterial evaluate_material(const MaterialID id, const glm::vec2& uv) const;

        const std::unique_ptr<Material>& get_material(const MaterialID id) const
        {
            return mMaterials[id];
        }

    private:

        std::vector<std::unique_ptr<Material>> mMaterials;
    };

}

#endif
