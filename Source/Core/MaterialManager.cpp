#include "MaterialManager.hpp"

namespace Core
{

    MaterialManager::~MaterialManager()
    {
        // TODO memory cleanup.
    }

    MaterialManager::MaterialID MaterialManager::add_material(std::unique_ptr<Material>& mat)
    {
        const MaterialID newID = mMaterials.size();
        mMaterials.push_back(std::move(mat));

        return newID;
    }

    EvaluatedMaterial MaterialManager::evaluate_material(const MaterialID id, const glm::vec2& uv)
    {
        std::unique_ptr<Material>& mat = mMaterials[id];
        if(!mat->is_resident())
        {
            const size_t requiredSize = mat->get_residence_size();
            void* memory = new char[requiredSize];

            mat->make_resident(memory);
        }

        return mat->evaluate_material(uv);
    }

}
