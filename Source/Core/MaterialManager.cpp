#include "MaterialManager.hpp"

namespace Core
{

    MaterialManager::~MaterialManager()
    {
        for(auto& mat : mMaterials)
                mat->make_nonresident();
    }

    MaterialManager::MaterialID MaterialManager::add_material(std::unique_ptr<Material>& mat)
    {
        if(!mat->is_resident())
        {
            const size_t requiredSize = mat->get_residence_size();
            void* memory = new char[requiredSize];

            mat->make_resident(memory);
        }

        const MaterialID newID = mMaterials.size();
        mMaterials.push_back(std::move(mat));

        return newID;
    }

    EvaluatedMaterial MaterialManager::evaluate_material(const MaterialID id, const glm::vec2& uv) const
    {
        const std::unique_ptr<Material>& mat = mMaterials[id];

        return mat->evaluate_material(uv);
    }

}
