#ifndef MATERIAL_MANAGER_HPP
#define MATERIAL_MANAGER_HPP

#include <memory>
#include <vector>

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
    };


    // Material / shader interface.
    class Material : public Loadable
    {
    public:

        Material();
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


        MaterialID add_material(std::shared_ptr<Material>&);

        EvaluatedMaterial evaluate_material(const MaterialID id, const glm::vec2& uv);

    private:

        std::vector<std::shared_ptr<Material>> mMaterials;
    };

}

#endif
