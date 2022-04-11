#ifndef BSSRDF_HPP
#define BSSRDF_HPP

#include "Distributions.hpp"
#include "BasicMaterials.hpp"
#include "Core/RandUtils.hpp"

#include "glm/glm.hpp"

#include <memory>

namespace Core
{
    namespace BVH
    {
        struct InterpolatedVertex;
    }
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
        kBRDF,
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

        virtual Sample sample(Core::Rand::Hammersley_Generator& rand, const Core::BVH::InterpolatedVertex& position, const glm::vec3& V) = 0;

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

        virtual Sample sample(Core::Rand::Hammersley_Generator& rand, const Core::BVH::InterpolatedVertex& position, const glm::vec3& V) final;

        virtual BSRDF_Type get_type() const final
        {
            return BSRDF_Type::kBRDF;
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

        virtual Sample sample(Core::Rand::Hammersley_Generator& rand, const Core::BVH::InterpolatedVertex &position, const glm::vec3& V) final;

        virtual BSRDF_Type get_type() const final
        {
            return BSRDF_Type::kBRDF;
        }

    private:

        std::unique_ptr<Render::Distribution> m_distribution;
    };

    class Light_BRDF : public BSRDF
    {
    public:

        Light_BRDF(Core::MaterialManager& mat_manager, Core::MaterialManager::MaterialID id) :
            BSRDF(mat_manager, id) {}

        virtual Sample sample(Core::Rand::Hammersley_Generator& rand, const Core::BVH::InterpolatedVertex &position, const glm::vec3& V) final;

        virtual BSRDF_Type get_type() const final
        {
            return BSRDF_Type::kLight;
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


        virtual Sample sample(Core::Rand::Hammersley_Generator& rand, const Core::BVH::InterpolatedVertex &position, const glm::vec3& V) final;

        virtual BSRDF_Type get_type() const final
        {
            return BSRDF_Type::kBTDF;
        }

    private:

        float calculate_critical_angle(const float outer_IoR) const;

        std::unique_ptr<Render::Distribution> m_distribution;

        float m_index_of_refraction;
    };

}

#endif
