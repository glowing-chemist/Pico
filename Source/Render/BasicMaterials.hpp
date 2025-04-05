#ifndef BASIC_MATERIALS_HPP
#define BASIC_MATERIALS_HPP

#include "Core/MaterialManager.hpp"
#include "Core/Image.hpp"

#include <memory>

namespace Render
{

    // Base material that doesn't reply on external data/textures.
    class ConstantMaterial : public Core::Material
    {
    public:

        ConstantMaterial() = default;
        virtual ~ConstantMaterial() = default;

        virtual size_t get_residence_size() const final
        {
            return 0;
        }

        virtual bool is_resident() const
        {
            return true;
        }

        virtual void make_resident(void*) final
        {

        }

        virtual void make_nonresident() final
        {

        }
    };

    class ConstantTransparentMaterial : public ConstantMaterial
    {
    public:

        ConstantTransparentMaterial(const float transparency, const float IoR) :
            m_transparency{transparency},
            m_index_of_refraction{IoR} {}

        virtual ~ConstantTransparentMaterial() = default;

        float get_transparency_factor() const
        {
            return m_transparency;
        }

        float get_index_of_refraction() const
        {
            return m_index_of_refraction;
        }

    private:

        float m_transparency;
        float m_index_of_refraction;
    };


    class SmoothMetalMaterial : public ConstantMaterial
    {
    public:

        SmoothMetalMaterial(const glm::vec3 colour) :
            m_colour(colour) {}

        virtual Core::EvaluatedMaterial evaluate_material(const glm::vec2& uv) const final;

        virtual bool              is_light() const final
        {
            return false;
        }

    private:

        glm::vec3 m_colour;
    };


    class RoughMetalMaterial : public ConstantMaterial
    {
    public:

        RoughMetalMaterial(const glm::vec3 colour) :
            m_colour(colour) {}

        virtual Core::EvaluatedMaterial evaluate_material(const glm::vec2& uv) const final;

        virtual bool              is_light() const final
        {
            return false;
        }

    private:

        glm::vec3 m_colour;
    };

    class MattPlasticMaterial : public ConstantMaterial
    {
    public:

        MattPlasticMaterial(const glm::vec3 colour) :
            m_colour(colour) {}

        virtual Core::EvaluatedMaterial evaluate_material(const glm::vec2& uv) const final;

        virtual bool              is_light() const final
        {
            return false;
        }

    private:

        glm::vec3 m_colour;
    };


    class EmmissiveMaterial : public ConstantMaterial
    {
    public:

        EmmissiveMaterial() = default;

        virtual Core::EvaluatedMaterial evaluate_material(const glm::vec2& uv) const final;

        virtual bool              is_light() const final
        {
            return true;
        }
    };

    class ConstantMetalnessRoughnessMaterial : public ConstantMaterial
    {
    public:
        ConstantMetalnessRoughnessMaterial(const glm::vec3& albedo, const float metalness, const float roughness, const glm::vec3& emissive);

        virtual Core::EvaluatedMaterial evaluate_material(const glm::vec2&) const final
        {
            return mMaterial;
        }

        virtual bool              is_light() const final
        {
            return mMaterial.emissive.x > 0.0f || mMaterial.emissive.y > 0.0f || mMaterial.emissive.z > 0.0f;
        }

    private:

        Core::EvaluatedMaterial mMaterial;
    };

    class ConstantDiffuseSpecularMaterial : public ConstantMaterial
    {
    public:
        ConstantDiffuseSpecularMaterial(const glm::vec3& diffuse, const glm::vec3& specular, const float gloss, const glm::vec3& emissive);

        virtual Core::EvaluatedMaterial evaluate_material(const glm::vec2&) const final
        {
            return mMaterial;
        }

        virtual bool              is_light() const final
        {
            return mMaterial.emissive.x > 0.0f || mMaterial.emissive.y > 0.0f || mMaterial.emissive.z > 0.0f;
        }

    private:

        Core::EvaluatedMaterial mMaterial;
    };


    class ConstantTransparentDiffuseSpecularMaterial : public ConstantTransparentMaterial
    {
    public:
        ConstantTransparentDiffuseSpecularMaterial(const glm::vec3& diffuse, const glm::vec3& specular, const float gloss, const float transparency, const float IoR);

        virtual Core::EvaluatedMaterial evaluate_material(const glm::vec2&) const final
        {
            return mMaterial;
        }

        virtual bool              is_light() const final
        {
            return false;
        }

    private:

        Core::EvaluatedMaterial mMaterial;
    };


    class ConstantTransparentMetalnessRoughnessMaterial : public ConstantTransparentMaterial
    {
    public:
        ConstantTransparentMetalnessRoughnessMaterial(const glm::vec3& albedo, const float metalness, const float roughness, const float transparency, const float IoR);

        virtual Core::EvaluatedMaterial evaluate_material(const glm::vec2&) const final
        {
            return mMaterial;
        }

        virtual bool              is_light() const final
        {
            return false;
        }

    private:

        Core::EvaluatedMaterial mMaterial;
    };


    // Non constant basic materials.
    class MetalnessRoughnessMaterial : public Core::Material
    {
    public:

        MetalnessRoughnessMaterial(std::unique_ptr<Core::Image2D>& albedo,
                                   std::unique_ptr<Core::Image2D>& metalness,
                                   std::unique_ptr<Core::Image2D>& roughness,
                                   std::unique_ptr<Core::Image2D>& emmissive);

        MetalnessRoughnessMaterial(std::unique_ptr<Core::Image2D>& albedo,
                                   std::unique_ptr<Core::Image2D>& combinedMetalnessRoughness,
                                   std::unique_ptr<Core::Image2D>& emmissive);

        virtual size_t get_residence_size() const final;

        virtual bool is_resident() const;

        virtual void make_resident(void*) final;

        virtual void make_nonresident() final;

        virtual Core::EvaluatedMaterial evaluate_material(const glm::vec2& uv) const final;

        virtual bool              is_light() const final
        {
            return mEmmissiveTexture.get();
        }

    private:

        bool m_combined_metalness_roughness = false;

        std::unique_ptr<Core::Image2D> mAlbedoTexture;
        std::unique_ptr<Core::Image2D> mMetalnessTexture;
        std::unique_ptr<Core::Image2D> mRoughnessTexture;
        std::unique_ptr<Core::Image2D> mEmmissiveTexture;

    };

    class SpecularGlossMaterial : public Core::Material
    {
    public:

        SpecularGlossMaterial(std::unique_ptr<Core::Image2D>& diffuse,
                              std::unique_ptr<Core::Image2D>& specular,
                              std::unique_ptr<Core::Image2D>& gloss,
                              std::unique_ptr<Core::Image2D>& emmissive);

        virtual size_t get_residence_size() const final;

        virtual bool is_resident() const;

        virtual void make_resident(void*) final;

        virtual void make_nonresident() final;

        virtual Core::EvaluatedMaterial evaluate_material(const glm::vec2& uv) const final;

        virtual bool              is_light() const final
        {
            return mEmmissiveTexture.get();
        }

    private:

        std::unique_ptr<Core::Image2D> mDiffuseTexture;
        std::unique_ptr<Core::Image2D> mSpecularTexture;
        std::unique_ptr<Core::Image2D> mGlossTexture;
        std::unique_ptr<Core::Image2D> mEmmissiveTexture;

    };
}

#endif
