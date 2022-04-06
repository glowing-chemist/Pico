#ifndef PICO_IMAGE_HPP
#define PICO_IMAGE_HPP

#include <filesystem>
#include <string>
#include <vector>
#include <cstdint>

#include "glm/common.hpp"

#include "Loadable.hpp"

namespace Core
{
    struct ImageExtent
    {
        uint32_t width, height, depth;
    };

    enum class Format
    {
        kRBGA_8UNorm,
        kRGB_8UNorm,
        kR_8UNorm
    };

    size_t get_pixel_size(const Format);
    size_t get_format_channels(const Format);

    class Image : public Loadable
    {
    public:
        Image(unsigned char* data, const ImageExtent& extent, const Format format) :
        mData(data),
        mExtent(extent),
        mFormat(format),
        mPixelSize(get_pixel_size(mFormat))
        {}

        Image(const std::filesystem::path& path);

        virtual ~Image();

        virtual size_t get_residence_size() const final;

        virtual bool is_resident() const final
        {
            return mData;
        }

        virtual void make_resident(void*) final;

        virtual void make_nonresident() final;

        const ImageExtent& get_extent() const
        {
            return mExtent;
        }

    protected:

        const unsigned char* get_data_ptr(const glm::vec2& uv) const;
        const unsigned char* get_data_ptr(const glm::vec2& uv, const uint32_t face) const;

        std::filesystem::path mPath;

        unsigned char* mData;
        ImageExtent mExtent;
        Format mFormat;
        uint32_t mPixelSize;
    };


    class Image2D : public Image
    {
    public:

        Image2D(unsigned char* data, const ImageExtent& extent, const Format format);

        Image2D(const std::filesystem::path& path) :
            Image(path) {}

        float sample(const glm::vec2& uv) const;
        glm::vec2 sample2(const glm::vec2& uv) const;
        glm::vec3 sample3(const glm::vec2& uv) const;
        glm::vec4 sample4(const glm::vec2& uv) const;

    };


    class ImageCube : public Image
    {
    public:

        ImageCube(unsigned char* data, const ImageExtent& extent, const Format format);


        float sample(const glm::vec3& uv) const;
        glm::vec2 sample2(const glm::vec3& uv) const;
        glm::vec3 sample3(const glm::vec3& uv) const;
        glm::vec4 sample4(const glm::vec3& uv) const;

    private:

        void resolve_cubemap_UV(const glm::vec3& v, uint32_t& faceIndex, glm::vec2& uvOut) const;

    };
}

#endif
