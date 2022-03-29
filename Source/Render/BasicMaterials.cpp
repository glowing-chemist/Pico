#include "BasicMaterials.hpp"

#include "glm/gtx/compatibility.hpp"

namespace Render
{

    Core::EvaluatedMaterial SmoothMetalMaterial::evaluate_material(const glm::vec2&)
    {
        return {{0.04f, 0.04f, 0.04f, 1.0f}, {m_colour, 0.05f}, {0.f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    }

    Core::EvaluatedMaterial RoughMetalMaterial::evaluate_material(const glm::vec2&)
    {
        return {{0.04f, 0.04f, 0.04f, 1.0f}, {m_colour, 0.8f}, {0.f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    }

    Core::EvaluatedMaterial MattPlasticMaterial::evaluate_material(const glm::vec2&)
    {
        return {{m_colour, 1.0f}, {0.04f, 0.04f, 0.04f, 0.8f}, {0.f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    }

    Core::EvaluatedMaterial EmmissiveMaterial::evaluate_material(const glm::vec2&)
    {
        return {{0.8f, 0.8f, 0.8f, 0.8f}, {0.0f, 0.0f, 0.0f, 0.8f}, {0.f, 1.0f, 0.0f}, {0.5f, 0.5f, 0.5f}};
    }

    ConstantMetalnessRoughnessMaterial::ConstantMetalnessRoughnessMaterial(const glm::vec3& albedo, const float metalness, const float roughness, const glm::vec3& emissive)
    {
        mMaterial.diffuse = glm::vec4(albedo * (1.0f - 0.04f) * (1.0f - metalness), 1.0f);

        const glm::vec3 F0 = glm::lerp(glm::vec3(0.04f, 0.04f, 0.04f), glm::vec3(albedo), metalness);
        mMaterial.specularRoughness.x = F0.x;
        mMaterial.specularRoughness.y = F0.y;
        mMaterial.specularRoughness.z = F0.z;

        mMaterial.specularRoughness.w = roughness;
        mMaterial.emissive = emissive;
    }

    ConstantDiffuseSpecularMaterial::ConstantDiffuseSpecularMaterial(const glm::vec3& diffuse, const glm::vec3& specular, const float gloss, const glm::vec3& emissive)
    {
        mMaterial.diffuse = glm::vec4(diffuse, 1.0f);
        mMaterial.specularRoughness = glm::vec4(specular, 1.0f - (gloss * gloss));
        mMaterial.emissive = emissive;
    }

    MetalnessRoughnessMaterial::MetalnessRoughnessMaterial(std::unique_ptr<Core::Image2D>& albedo,
                                                           std::unique_ptr<Core::Image2D>& metalness,
                                                           std::unique_ptr<Core::Image2D>& roughness,
                                                           std::unique_ptr<Core::Image2D>& emmissive)
        :
          mAlbedoTexture(std::move(albedo)),
          mMetalnessTexture(std::move(metalness)),
          mRoughnessTexture(std::move(roughness)),
          mEmmissiveTexture(std::move(emmissive)) {}

    MetalnessRoughnessMaterial::MetalnessRoughnessMaterial(std::unique_ptr<Core::Image2D>& albedo,
                               std::unique_ptr<Core::Image2D>& combinedMetalnessRoughness,
                               std::unique_ptr<Core::Image2D>& emmissive)
        :
          m_combined_metalness_roughness(true),
          mAlbedoTexture(std::move(albedo)),
          mMetalnessTexture(std::move(combinedMetalnessRoughness)),
          mEmmissiveTexture(std::move(emmissive)) {}


    size_t MetalnessRoughnessMaterial::get_residence_size() const
    {
        return (mAlbedoTexture ? mAlbedoTexture->get_residence_size() : 0) +
                (mMetalnessTexture ? mMetalnessTexture->get_residence_size() : 0) +
                (mRoughnessTexture ? mRoughnessTexture->get_residence_size() : 0) +
                (mEmmissiveTexture ? mEmmissiveTexture->get_residence_size() : 0);
    }

    bool MetalnessRoughnessMaterial::is_resident() const
    {
        return (mAlbedoTexture && mAlbedoTexture->is_resident()) ||
               (mMetalnessTexture && mMetalnessTexture->is_resident()) ||
               (mRoughnessTexture && mRoughnessTexture->is_resident()) ||
               (mEmmissiveTexture && mEmmissiveTexture->is_resident());
    }

    void MetalnessRoughnessMaterial::make_resident(void* memory)
    {
        unsigned char* texture_mem = static_cast<unsigned char*>(memory);
        if(mAlbedoTexture)
        {
            mAlbedoTexture->make_resident(texture_mem);
            texture_mem += mAlbedoTexture->get_residence_size();
        }

        if(mMetalnessTexture)
        {
            mMetalnessTexture->make_resident(texture_mem);
            texture_mem += mMetalnessTexture->get_residence_size();
        }

        if(mRoughnessTexture)
        {
            mRoughnessTexture->make_resident(texture_mem);
            texture_mem += mRoughnessTexture->get_residence_size();
        }

        if(mEmmissiveTexture)
        {
            texture_mem += mEmmissiveTexture->get_residence_size();
            mEmmissiveTexture->make_resident(texture_mem);
        }

    }

    void MetalnessRoughnessMaterial::make_nonresident()
    {
        if(mAlbedoTexture)
            mAlbedoTexture->make_nonresident();

        if(mMetalnessTexture)
            mMetalnessTexture->make_nonresident();

        if(mRoughnessTexture)
            mRoughnessTexture->make_nonresident();

        if(mEmmissiveTexture)
            mEmmissiveTexture->make_nonresident();
    }

    Core::EvaluatedMaterial MetalnessRoughnessMaterial::evaluate_material(const glm::vec2& uv)
    {
        Core::EvaluatedMaterial material{{0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

        const glm::vec4 albedo = mAlbedoTexture->sample4(uv);

        float metalness = 0.0f;
        if(m_combined_metalness_roughness) // Follow the gltf convention of metalness roughness is the zy channels.
        {
            if(mMetalnessTexture)
            {
                const glm::vec4 combined_metalness_roughness = mMetalnessTexture->sample4(uv);
                metalness = combined_metalness_roughness.z;
                material.specularRoughness.w = combined_metalness_roughness.y;
            }
        }
        else
        {
            if(mMetalnessTexture)
                metalness = mMetalnessTexture->sample(uv);

            if(mRoughnessTexture)
                material.specularRoughness.w = mRoughnessTexture->sample(uv);
        }

        material.diffuse = albedo * (1.0f - 0.04f) * (1.0f - metalness);
        material.diffuse.w = albedo.w;

        const glm::vec3 F0 = glm::lerp(glm::vec3(0.04f, 0.04f, 0.04f), glm::vec3(albedo), metalness);
        material.specularRoughness.x = F0.x;
        material.specularRoughness.y = F0.y;
        material.specularRoughness.z = F0.z;

        if(mEmmissiveTexture)
            material.emissive = mEmmissiveTexture->sample3(uv);

        return material;
    }


    SpecularGlossMaterial::SpecularGlossMaterial(std::unique_ptr<Core::Image2D>& diffuse,
                                                 std::unique_ptr<Core::Image2D>& specular,
                                                 std::unique_ptr<Core::Image2D>& gloss,
                                                 std::unique_ptr<Core::Image2D>& emmissive)
        :
          mDiffuseTexture(std::move(diffuse)),
          mSpecularTexture(std::move(specular)),
          mGlossTexture(std::move(gloss)),
          mEmmissiveTexture(std::move(emmissive)) {}

    size_t SpecularGlossMaterial::get_residence_size() const
    {
        return (mDiffuseTexture ? mDiffuseTexture->get_residence_size() : 0) +
                (mSpecularTexture ? mSpecularTexture->get_residence_size() : 0) +
                (mGlossTexture ? mGlossTexture->get_residence_size() : 0) +
                (mEmmissiveTexture ? mEmmissiveTexture->get_residence_size() : 0);
    }

    bool SpecularGlossMaterial::is_resident() const
    {
        return (mDiffuseTexture && mDiffuseTexture->is_resident()) ||
               (mSpecularTexture && mSpecularTexture->is_resident()) ||
               (mGlossTexture && mGlossTexture->is_resident()) ||
               (mEmmissiveTexture && mEmmissiveTexture->is_resident());
    }

    void SpecularGlossMaterial::make_resident(void* memory)
    {
        unsigned char* texture_mem = static_cast<unsigned char*>(memory);
        if(mDiffuseTexture)
        {
            mDiffuseTexture->make_resident(texture_mem);
            texture_mem += mDiffuseTexture->get_residence_size();
        }

        if(mSpecularTexture)
        {
            mSpecularTexture->make_resident(texture_mem);
            texture_mem += mSpecularTexture->get_residence_size();
        }

        if(mGlossTexture)
        {
            mGlossTexture->make_resident(texture_mem);
            texture_mem += mGlossTexture->get_residence_size();
        }

        if(mEmmissiveTexture)
            mEmmissiveTexture->make_resident(texture_mem);

    }

    void SpecularGlossMaterial::make_nonresident()
    {
        if(mDiffuseTexture)
            mDiffuseTexture->make_nonresident();

        if(mSpecularTexture)
            mSpecularTexture->make_nonresident();

        if(mGlossTexture)
            mGlossTexture->make_nonresident();

        if(mEmmissiveTexture)
            mEmmissiveTexture->make_nonresident();
    }

    Core::EvaluatedMaterial SpecularGlossMaterial::evaluate_material(const glm::vec2& uv)
    {
        Core::EvaluatedMaterial material{{0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

        material.diffuse = mDiffuseTexture->sample4(uv);

        if(mSpecularTexture)
            material.specularRoughness = glm::vec4(mSpecularTexture->sample3(uv), 1.0f);

        if(mEmmissiveTexture)
            material.emissive = mEmmissiveTexture->sample3(uv);

        if(mGlossTexture)
            material.specularRoughness.w = 1.0f - mGlossTexture->sample(uv);

        return material;
    }
}
