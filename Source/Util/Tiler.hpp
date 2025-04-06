#ifndef PICO_TILER_HPP
#define PICO_TILER_HPP

#include "Core/RandUtils.hpp"
#include "Core/ThreadPool.hpp"

namespace Util
{
    class Tiler
    {
        public:

        Tiler(ThreadPool& workers, Core::Rand::xorshift_random& rand , const glm::uvec2& resolution, const glm::uvec2& tile_size) :
            m_thread_pool(workers),
            m_rng(rand),
            m_resolution(resolution),
            m_tile_size(tile_size)
        {}

        template<typename F, typename ...A>
        void execute_over_surface(F& f, A... a)
        {
            const uint32_t tile_count_x = m_resolution.x / m_tile_size.x;
            const uint32_t tile_count_y = m_resolution.y / m_tile_size.y;
            const uint32_t tile_count = tile_count_x * tile_count_y;

            std::vector<std::future<bool>> handles{};
            handles.reserve(tile_count);

            for(uint32_t x = 0; x < m_resolution.x; x += m_tile_size.x)
            {
                for(uint32_t y = 0; y < m_resolution.y; y += m_tile_size.y)
                {
                    glm::uvec2 clamped_tile_size = m_tile_size;
                    if((x + m_tile_size.x) > m_resolution.x)
                    {
                        clamped_tile_size.x -= (x + m_tile_size.x) - m_resolution.x;
                    }
                    if((y + m_tile_size.y) > m_resolution.y)
                    {
                        clamped_tile_size.y -= (y + m_tile_size.y) - m_resolution.y;
                    }
                    handles.push_back(m_thread_pool.add_task(f, glm::uvec2(x, y), clamped_tile_size, m_resolution, m_rng.next(), a...));
                }
            }

            for(auto& thread : handles)
                thread.wait();
        }

        private:

        ThreadPool& m_thread_pool;
        Core::Rand::xorshift_random m_rng;
        glm::uvec2 m_resolution;
        glm::uvec2 m_tile_size;
    };

}

#endif
