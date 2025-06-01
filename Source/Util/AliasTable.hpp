#ifndef PICO_ALIAS_TABLE_HPP
#define PICO_ALIAS_TABLE_HPP

#include <vector>

#include "Core/RandUtils.hpp"

namespace Util
{
    class AliasTable
    {
    public:
        AliasTable(const std::vector<float>& weights);
        AliasTable() = default;

        uint32_t sample(Core::Rand::xorshift_random& rng, float& pdf) const;

        void build_table(const std::vector<float>& weights);

    private:

        struct Bucket
        {
            float   m_weight;
            uint32_t m_alias_index;
        };

        std::vector<Bucket> m_buckets;
    };
}

#endif
