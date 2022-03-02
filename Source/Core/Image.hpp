#ifndef PICO_IMAGE_HPP
#define PICO_IMAGE_HPP

#include <vector>
#include <cstdint>

#include "glm/common.hpp"

namespace Core
{
    struct ImageExtent
    {
        uint32_t width, height, z;
    };

    enum class Format
    {
        kRBGA_8UNorm
    };

    size_t get_pixel_size(const Format);

    class Image
    {
    public:
        Image(std::vector<unsigned char>&& data, const ImageExtent& extent, const Format format) :
        mData(data),
        mExtent(extent),
        mFormat(format),
        mPixelSize(get_pixel_size(mFormat))
        {}

        ~Image();

        float sample(const glm::vec2& uv) const;
        glm::vec2 sample2(const glm::vec2& uv) const;
        glm::vec3 sample3(const glm::vec2& uv) const;
        glm::vec4 sample4(const glm::vec2& uv) const;

        float sample_cube(const glm::vec3& uv) const;
        glm::vec2 sample_cube2(const glm::vec3& uv) const;
        glm::vec3 sample_cube3(const glm::vec3& uv) const;
        glm::vec4 sample_cube4(const glm::vec3& uv) const;

        const ImageExtent& get_extent() const
        {
            return mExtent;
        }

    private:

        const unsigned char* get_data_ptr(const glm::vec2& uv) const;
        const unsigned char* get_data_ptr(const glm::vec2& uv, const uint32_t face) const;

        void resolve_cubemap_UV(const glm::vec3& v, uint32_t& faceIndex, glm::vec2& uvOut) const;

        std::vector<unsigned char> mData;
        ImageExtent mExtent;
        Format mFormat;
        uint32_t mPixelSize;
    };

}

#endif
