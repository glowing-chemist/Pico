#include "Image.hpp"
#include "stb_image.h"

namespace Core
{

    size_t get_pixel_size(const Format format)
    {
        switch(format)
        {
            case Format::kRBGA_8UNorm:
                return 4;

            case Format::kRGB_8UNorm:
                return 3;

            case Format::kR_8UNorm:
                return 1;
        }

        return 4;
    }

    size_t get_format_channels(const Format format)
    {
        switch(format)
        {
            case Format::kRBGA_8UNorm:
                return 4;

            case Format::kRGB_8UNorm:
                return 3;

            case Format::kR_8UNorm:
                return 1;
        }

        return 4;
    }

    Image::Image(const std::string& path) :
        mPath(path),
        mData(nullptr)
    {
        int x, y, components;
        stbi_info(mPath.c_str(), &x, &y, &components);

        mExtent = {static_cast<uint32_t>(x), static_cast<uint32_t>(y), 1};
        if(components == 4)
            mFormat = Format::kRBGA_8UNorm;
        else if(components == 3)
            mFormat = Format::kRGB_8UNorm;
        else if(components == 1)
            mFormat = Format::kR_8UNorm;

        mPixelSize = get_pixel_size(mFormat);
    }


    Image::~Image()
    {
        delete[] mData;
    }

    size_t Image::get_residence_size() const
    {
        return mExtent.height * mExtent.width * mExtent.depth * mPixelSize;
    }

    void Image::make_resident(void* mem)
    {
        int x, y, c;
        auto* image = stbi_load(mPath.c_str(), &x, &y, &c, get_format_channels(mFormat));

        const size_t size = get_residence_size();
        mData = static_cast<unsigned char*>(mem);
        memcpy(mData, image, size);

        free(image);
    }

    void Image::make_nonresident()
    {
        if(!mPath.empty())
            mData = nullptr;
    }


    Image2D::Image2D(unsigned char* data, const ImageExtent& extent, const Format format) :
        Image(data, extent, format)
    {
    }


    float Image2D::sample(const glm::vec2& uv) const
    {
        const unsigned char* data = get_data_ptr(uv);

        if(mPixelSize == 4)
        {
            const float* pix = reinterpret_cast<const float*>(data);
            float c = pix[0];

            return c;
        }
        else if(mPixelSize == 1)
        {
            const uint8_t* pix = reinterpret_cast<const uint8_t*>(data);
            float c = pix[0] / 255.0f;
            return c;
        }


        return 0.0f;
    }


    glm::vec2 Image2D::sample2(const glm::vec2& uv) const
    {
        const unsigned char* data = get_data_ptr(uv);

        if(mPixelSize == 8)
        {
            const float* pix = reinterpret_cast<const float*>(data);
            glm::vec2 c;
            c.x = pix[0];
            c.y = pix[1];

            return c;
        }
        else if(mPixelSize == 2)
        {
            const uint8_t* pix = reinterpret_cast<const uint8_t*>(data);
            glm::vec2 c;
            c.x = pix[0] / 255.0f;
            c.y = pix[1] / 255.0f;

            return c;
        }

        return glm::vec2(0.0f, 0.0f);
    }


    glm::vec3 Image2D::sample3(const glm::vec2& uv) const
    {
        const unsigned char* data = get_data_ptr(uv);

        if(mPixelSize == 12)
        {
            const float* pix = reinterpret_cast<const float*>(data);
            glm::vec3 c;
            c.x = pix[0];
            c.y = pix[1];
            c.z = pix[2];

            return c;
        }
        else if(mPixelSize == 3)
        {
            const uint8_t* pix = reinterpret_cast<const uint8_t*>(data);
            glm::vec3 c;
            c.x = pix[0] / 255.0f;
            c.y = pix[1] / 255.0f;
            c.z = pix[2] / 255.0f;

            return c;
        }

        return glm::vec3{0.0f, 0.0f, 0.0f};
    }


    glm::vec4 Image2D::sample4(const glm::vec2& uv) const
    {
        const unsigned char* data = get_data_ptr(uv);

        if(mPixelSize == 16)
        {
            const float* pix = reinterpret_cast<const float*>(data);
            glm::vec4 c;
            c.x = pix[0];
            c.y = pix[1];
            c.z = pix[2];
            c.w = pix[3];

            return c;
        }
        else if(mPixelSize == 3)
        {
            const uint8_t* pix = reinterpret_cast<const uint8_t*>(data);
            glm::vec4 c;
            c.x = pix[0] / 255.0f;
            c.y = pix[1] / 255.0f;
            c.z = pix[2] / 255.0f;
            c.w = 0.0f;

            return c;
        }
        else if(mPixelSize == 4)
        {
            const uint8_t* pix = reinterpret_cast<const uint8_t*>(data);
            glm::vec4 c;
            c.x = pix[0] / 255.0f;
            c.y = pix[1] / 255.0f;
            c.z = pix[2] / 255.0f;
            c.w = pix[3] / 255.0f;

            return c;
        }

        return glm::vec4{0.0f, 0.0f, 0.0f, 0.0f};
    }


    ImageCube::ImageCube(unsigned char* data, const ImageExtent& extent, const Format format) :
        Image(data, extent, format)
    {
    }


    float ImageCube::sample(const glm::vec3& d) const
    {
        uint32_t faceIndex;
        glm::vec2 uv;
        resolve_cubemap_UV(d, faceIndex, uv);

        const unsigned char* data = get_data_ptr(uv, faceIndex);

        if(mPixelSize == 4)
        {
            const float* pix = reinterpret_cast<const float*>(data);
            float c = pix[0];

            return c;
        }
        else if(mPixelSize == 1)
        {
            const uint8_t* pix = reinterpret_cast<const uint8_t*>(data);
            float c = pix[0] / 255.0f;
            return c;
        }

        return 0.0f;
    }


    glm::vec2 ImageCube::sample2(const glm::vec3& d) const
    {
        uint32_t faceIndex;
        glm::vec2 uv;
        resolve_cubemap_UV(d, faceIndex, uv);

        const unsigned char* data = get_data_ptr(uv, faceIndex);

        if(mPixelSize == 8)
        {
            const float* pix = reinterpret_cast<const float*>(data);
            glm::vec2 c;
            c.x = pix[0];
            c.y = pix[1];

            return c;
        }
        else if(mPixelSize == 2)
        {
            const uint8_t* pix = reinterpret_cast<const uint8_t*>(data);
            glm::vec2 c;
            c.x = pix[0] / 255.0f;
            c.y = pix[1] / 255.0f;

            return c;
        }

        return glm::vec2(0.0f, 0.0f);
    }


    glm::vec3 ImageCube::sample3(const glm::vec3& d) const
    {
        uint32_t faceIndex;
        glm::vec2 uv;
        resolve_cubemap_UV(d, faceIndex, uv);

        const unsigned char* data = get_data_ptr(uv, faceIndex);

        if(mPixelSize == 12)
        {
            const float* pix = reinterpret_cast<const float*>(data);
            glm::vec3 c;
            c.x = pix[0];
            c.y = pix[1];
            c.z = pix[2];

            return c;
        }
        else if(mPixelSize == 3)
        {
            const uint8_t* pix = reinterpret_cast<const uint8_t*>(data);
            glm::vec3 c;
            c.x = pix[0] / 255.0f;
            c.y = pix[1] / 255.0f;
            c.z = pix[2] / 255.0f;

            return c;
        }

        return glm::vec3{0.0f, 0.0f, 0.0f};
    }


    glm::vec4 ImageCube::sample4(const glm::vec3& d) const
    {
        uint32_t faceIndex;
        glm::vec2 uv;
        resolve_cubemap_UV(d, faceIndex, uv);

        const unsigned char* data = get_data_ptr(uv, faceIndex);

        if(mPixelSize == 16)
        {
            const float* pix = reinterpret_cast<const float*>(data);
            glm::vec4 c;
            c.x = pix[0];
            c.y = pix[1];
            c.z = pix[2];
            c.w = pix[3];

            return c;
        }
        else if(mPixelSize == 4)
        {
            const uint8_t* pix = reinterpret_cast<const uint8_t*>(data);
            glm::vec4 c;
            c.x = pix[0] / 255.0f;
            c.y = pix[1] / 255.0f;
            c.z = pix[2] / 255.0f;
            c.w = pix[3] / 255.0f;

            return c;
        }

        return glm::vec4{0.0f, 0.0f, 0.0f, 0.0f};
    }


    const unsigned char* Image::get_data_ptr(const glm::vec2& uv) const
    {
        const uint32_t x = uint32_t(uv.x * mExtent.width) % mExtent.width;
        const uint32_t y = uint32_t(uv.y * mExtent.height) % mExtent.height;

        const unsigned char* data = mData;
        const uint64_t offset = (mPixelSize * y * mExtent.width) + (mPixelSize * x);
        data += offset;

        return data;
    }


    const unsigned char* Image::get_data_ptr(const glm::vec2& uv, const uint32_t face) const
    {
        const uint32_t x = uint32_t(uv.x * mExtent.width) % mExtent.width;
        const uint32_t y = uint32_t(uv.y * mExtent.height) % mExtent.height;

        const unsigned char* data = mData;
        const uint64_t offset = (mPixelSize * y * mExtent.width) + (mPixelSize * x) + (face * mExtent.width * mExtent.height * mPixelSize);
        data += offset;

        return data;
    }


    void ImageCube::resolve_cubemap_UV(const glm::vec3& v, uint32_t& faceIndex, glm::vec2& uvOut) const
    {
        glm::vec3 vAbs = abs(v);
        float ma;
        glm::vec2 uv;
        if(vAbs.z >= vAbs.x && vAbs.z >= vAbs.y)
        {
            faceIndex = v.z < 0.0f ? 5 : 4;
            ma = 0.5f / vAbs.z;
            uv = glm::vec2(v.z < 0.0f ? -v.x : v.x, -v.y);
        }
        else if(vAbs.y >= vAbs.x)
        {
            faceIndex = v.y < 0.0f ? 3 : 2;
            ma = 0.5f / vAbs.y;
            uv = glm::vec2(v.x, v.y < 0.0f ? -v.z : v.z);
        }
        else
        {
            faceIndex = v.x < 0.0f ? 1 : 0;
            ma = 0.5f / vAbs.x;
            uv = glm::vec2(v.x < 0.0f ? v.z : -v.z, -v.y);
        }
        uvOut = uv * ma + 0.5f;
    }

}
