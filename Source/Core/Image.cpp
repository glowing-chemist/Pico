#include "Image.hpp"

namespace Core
{

    Image::~Image()
    {

    }

    float Image::sample(const glm::vec2& uv) const
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


    glm::vec2 Image::sample2(const glm::vec2& uv) const
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


    glm::vec3 Image::sample3(const glm::vec2& uv) const
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


    glm::vec4 Image::sample4(const glm::vec2& uv) const
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


    float Image::sample_cube(const glm::vec3& d) const
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


    glm::vec2 Image::sample_cube2(const glm::vec3& d) const
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


    glm::vec3 Image::sample_cube3(const glm::vec3& d) const
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


    glm::vec4 Image::sample_cube4(const glm::vec3& d) const
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

        const unsigned char* data = mData.data();
        const uint64_t offset = (mPixelSize * y * mExtent.width) + (mPixelSize * x);
        data += offset;

        return data;
    }


    const unsigned char* Image::get_data_ptr(const glm::vec2& uv, const uint32_t face) const
    {
        const uint32_t x = uint32_t(uv.x * mExtent.width) % mExtent.width;
        const uint32_t y = uint32_t(uv.y * mExtent.height) % mExtent.height;

        const unsigned char* data = mData.data();
        const uint64_t offset = (mPixelSize * y * mExtent.width) + (mPixelSize * x) + (face * mExtent.width * mExtent.height * mPixelSize);
        data += offset;

        return data;
    }


    void Image::resolve_cubemap_UV(const glm::vec3& v, uint32_t& faceIndex, glm::vec2& uvOut) const
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
