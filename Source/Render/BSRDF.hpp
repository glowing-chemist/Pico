#ifndef BSSRDF_HPP
#define BSSRDF_HPP

#include "Distributions.hpp"
#include "BasicMaterials.hpp"
#include "Core/RandUtils.hpp"

#include "glm/glm.hpp"

#include <memory>

namespace Core
{
    namespace Acceleration_Structures
    {
        struct InterpolatedVertex;
    }

    struct Ray;
}

namespace Render
{

    struct Sample
    {
        glm::vec3 L;
        float P;
        glm::vec3 energy;
    };

    enum class BSRDF_Type
    {
        kDiffuse_BRDF,
        kSpecular_BRDF,
        kDielectric_BRDF,
        kBTDF,
        kLight
    };

    class BSRDF
    {
    public:
        BSRDF(Core::MaterialManager& manager, Core::MaterialManager::MaterialID id) :
            m_material_manager(manager),
            m_mat_id{id} {}

        virtual ~BSRDF() = default;

        virtual Sample sample(Core::Rand::Hammersley_Generator& rand, const Core::Acceleration_Structures::InterpolatedVertex& position, Core::Ray& ray) = 0;

        virtual float pdf(const glm::vec3& wo, const glm::vec3& H, const float roughness, const float reflectance) = 0;

        virtual glm::vec3 energy(const Core::Acceleration_Structures::InterpolatedVertex& position, const glm::vec3& wo, const glm::vec3& H) = 0;

        virtual BSRDF_Type get_type() const = 0;

        Core::MaterialManager::MaterialID get_material_id() const
        {
            return m_mat_id;
        }

    protected:

        Core::MaterialManager& m_material_manager;

        Core::MaterialManager::MaterialID m_mat_id;
    };


    class Diffuse_BRDF : public BSRDF
    {
    public:

        Diffuse_BRDF(std::unique_ptr<Render::Distribution>& dist, Core::MaterialManager& mat_manager, Core::MaterialManager::MaterialID id) :
            BSRDF(mat_manager, id),
            m_distribution(std::move(dist)) {}

        virtual Sample sample(Core::Rand::Hammersley_Generator& rand, const Core::Acceleration_Structures::InterpolatedVertex& position, Core::Ray& ray) final;

        virtual float pdf(const glm::vec3& wo, const glm::vec3& H, const float roughness, const float reflectance) final;

        virtual glm::vec3 energy(const Core::Acceleration_Structures::InterpolatedVertex& position, const glm::vec3& wo, const glm::vec3& H) final;

        virtual BSRDF_Type get_type() const final
        {
            return BSRDF_Type::kDiffuse_BRDF;
        }

    private:

        std::unique_ptr<Render::Distribution> m_distribution;
    };


    class Specular_BRDF : public BSRDF
    {
    public:

        Specular_BRDF(std::unique_ptr<Render::Distribution>& dist, Core::MaterialManager& mat_manager, Core::MaterialManager::MaterialID id) :
            BSRDF(mat_manager, id),
            m_distribution(std::move(dist)) {}

        virtual Sample sample(Core::Rand::Hammersley_Generator& rand, const Core::Acceleration_Structures::InterpolatedVertex &position, Core::Ray& ray) final;

        virtual float pdf(const glm::vec3& wo, const glm::vec3& H, const float roughness, const float reflectance) final;

        virtual glm::vec3 energy(const Core::Acceleration_Structures::InterpolatedVertex& position, const glm::vec3& wo, const glm::vec3& H) final;

        virtual BSRDF_Type get_type() const final
        {
            return BSRDF_Type::kSpecular_BRDF;
        }

    private:

        std::unique_ptr<Render::Distribution> m_distribution;
    };

    class Dielectric_BRDF : public BSRDF
    {
    public:

        Dielectric_BRDF(std::unique_ptr<Render::Distribution>& diffuse_dist, std::unique_ptr<Render::Distribution>& specular_dist, Core::MaterialManager& mat_manager, Core::MaterialManager::MaterialID id) :
            BSRDF(mat_manager, id),
            m_diffuse_distribution(std::move(diffuse_dist)),
            m_specular_distribution(std::move(specular_dist)) {}

        virtual Sample sample(Core::Rand::Hammersley_Generator& rand, const Core::Acceleration_Structures::InterpolatedVertex &position, Core::Ray& ray) final;

        virtual float pdf(const glm::vec3& wo, const glm::vec3& H, const float roughness, const float reflectance) final;

        virtual glm::vec3 energy(const Core::Acceleration_Structures::InterpolatedVertex& position, const glm::vec3& wo, const glm::vec3& H) final;

        virtual BSRDF_Type get_type() const final
        {
            return BSRDF_Type::kDielectric_BRDF;
        }

    private:

        std::unique_ptr<Render::Distribution> m_diffuse_distribution;
        std::unique_ptr<Render::Distribution> m_specular_distribution;
    };

    class Light_BRDF : public BSRDF
    {
    public:

        Light_BRDF(Core::MaterialManager& mat_manager, Core::MaterialManager::MaterialID id);

        virtual Sample sample(Core::Rand::Hammersley_Generator& rand, const Core::Acceleration_Structures::InterpolatedVertex &position, Core::Ray& ray) final;

        virtual float pdf(const glm::vec3& wo, const glm::vec3& H, const float roughness, const float reflectance) final;

        virtual glm::vec3 energy(const Core::Acceleration_Structures::InterpolatedVertex& position, const glm::vec3& wo, const glm::vec3& H) final;

        virtual BSRDF_Type get_type() const final
        {
            return BSRDF_Type::kLight;
        }

    private:

        std::unique_ptr<Render::Distribution> m_distribution;

    };

    class Specular_Delta_BRDF : public BSRDF
    {
    public:

        Specular_Delta_BRDF(Core::MaterialManager& mat_manager, Core::MaterialManager::MaterialID id) :
            BSRDF(mat_manager, id) {}

        virtual Sample sample(Core::Rand::Hammersley_Generator& rand, const Core::Acceleration_Structures::InterpolatedVertex &position, Core::Ray& ray) final;

        virtual float pdf(const glm::vec3& wo, const glm::vec3& H, const float roughness, const float reflectance) final;

        virtual glm::vec3 energy(const Core::Acceleration_Structures::InterpolatedVertex& position, const glm::vec3& wo, const glm::vec3& H) final;

        virtual BSRDF_Type get_type() const final
        {
            return BSRDF_Type::kSpecular_BRDF;
        }
    };

    class Transparent_BTDF : public BSRDF
    {
    public:

        Transparent_BTDF(std::unique_ptr<Render::Distribution>& dist, Core::MaterialManager& mat_manager, Core::MaterialManager::MaterialID id) :
            BSRDF(mat_manager, id),
            m_distribution(std::move(dist))
        {
            const auto& material = m_material_manager.get_material(id);
            if(auto* transparent_material = dynamic_cast<Render::ConstantTransparentMaterial*>(material.get()))
            {
                m_index_of_refraction = transparent_material->get_index_of_refraction();
            }
        }


        virtual Sample sample(Core::Rand::Hammersley_Generator& rand, const Core::Acceleration_Structures::InterpolatedVertex &position, Core::Ray &ray) final;

        virtual float pdf(const glm::vec3& wo, const glm::vec3& H, const float roughness, const float reflectance) final;

        virtual glm::vec3 energy(const Core::Acceleration_Structures::InterpolatedVertex& position, const glm::vec3& wo, const glm::vec3& H) final;

        virtual BSRDF_Type get_type() const final
        {
            return BSRDF_Type::kBTDF;
        }

        float get_index_of_refraction() const
        {
            return m_index_of_refraction;
        }

    private:

        bool refract(const glm::vec3& wi, const glm::vec3& n, const float eta, glm::vec3& wt);

        float calculate_critical_angle(const float outer_IoR) const;

        std::unique_ptr<Render::Distribution> m_distribution;

        float m_index_of_refraction;
    };

    class Fresnel_BTDF : public BSRDF
    {
    public:

        Fresnel_BTDF(std::unique_ptr<Render::Distribution>& specular_dist, std::unique_ptr<Render::Distribution>& transparent_dist,
                     Core::MaterialManager& mat_manager, Core::MaterialManager::MaterialID id) :
            BSRDF(mat_manager, id)
        {
            m_transparent_bsrdf = std::make_unique<Transparent_BTDF>(transparent_dist, mat_manager, id);
            m_specular_bsrdf    = std::make_unique<Specular_BRDF>(specular_dist, mat_manager, id);
        }


        virtual Sample sample(Core::Rand::Hammersley_Generator& rand, const Core::Acceleration_Structures::InterpolatedVertex &position, Core::Ray& ray) final;

        virtual float pdf(const glm::vec3& wo, const glm::vec3& H, const float roughness, const float reflectance) final;

        virtual glm::vec3 energy(const Core::Acceleration_Structures::InterpolatedVertex& position, const glm::vec3& wo, const glm::vec3& H) final;

        virtual BSRDF_Type get_type() const final
        {
            return BSRDF_Type::kBTDF;
        }

    private:

        float fresnel_factor(const float cosThetaI, float etaI, float etaT);

        std::unique_ptr<Transparent_BTDF> m_transparent_bsrdf;
        std::unique_ptr<Specular_BRDF>    m_specular_bsrdf;

    };

}

#endif
