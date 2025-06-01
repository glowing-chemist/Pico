#include "AliasTable.hpp"
#include <numeric>

namespace Util
{

    AliasTable::AliasTable(const std::vector<float>& weights)
    {
        build_table(weights);
    }

    void AliasTable::build_table(const std::vector<float>& weights)
    {
        const uint32_t bucket_count = weights.size();

        m_buckets.resize(bucket_count);

        const float total_weight = std::accumulate(weights.begin(), weights.end(), 0.0);

        for(uint32_t i = 0; i < bucket_count; ++i)
        {
            m_buckets[i].m_weight = weights[i] / total_weight;
        }

        struct Outcome
        {
            float pHat;
            uint32_t index;
        };

        std::vector<Outcome> over, under;
        for(uint32_t i = 0; i < bucket_count; ++i)
        {
            const float pHat = m_buckets[i].m_weight * bucket_count;
            if(pHat < 1.0f)
            {
                under.emplace_back(pHat, i);
            }
            else
            {
                over.emplace_back(pHat, i);
            }
        }

        // Loop over and select pairs of weights that sumto at least 1 / bucket_count.
        while(!over.empty() && !under.empty())
        {
            const Outcome ov = over.back();
            over.pop_back();

            const Outcome un = under.back();
            under.pop_back();

            m_buckets[un.index].m_weight = un.pHat;
            m_buckets[un.index].m_alias_index = ov.index;

            // Now hadnle any left over probability
            const float excess = un.pHat + ov.pHat - 1.0f;
            if(excess < 1.0f)
            {
                under.emplace_back(excess, ov.index);
            }
            else
            {
                over.emplace_back(excess, ov.index);
            }
        }

        // Handle any left over weights
        while(!under.empty())
        {
            const Outcome un = under.back();
            under.pop_back();

            m_buckets[un.index].m_alias_index = 0;
            m_buckets[un.index].m_weight = 1.0f;
        }

        while(!over.empty())
        {
            const Outcome ov = over.back();
            over.pop_back();

            m_buckets[ov.index].m_alias_index = 0;
            m_buckets[ov.index].m_weight = 1.0f;
        }
    }

    uint32_t AliasTable::sample(Core::Rand::xorshift_random& rng, float& pdf) const
    {
        const uint32_t bucket_index = rng.next() % m_buckets.size();

        const float coin_flip = float(rng.next()) / float(rng.max());
        if(coin_flip <= m_buckets[bucket_index].m_weight)
        {
            pdf = m_buckets[bucket_index].m_weight / m_buckets.size();
            return bucket_index;
        }
        else
        {
            pdf = 1.0f - (m_buckets[bucket_index].m_weight / m_buckets.size());
            return m_buckets[bucket_index].m_alias_index;
        }
    }

}
