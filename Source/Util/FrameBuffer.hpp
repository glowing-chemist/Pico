#ifndef PICO_FRAEMBUFFER_HPP
#define PICO_FRAEMBUFFER_HPP

#include <glad/glad.h>
#include "GLFW/glfw3.h"

#include <cstdint>

namespace Util
{

    class FrameBuffer
    {
    public:

        FrameBuffer(uint32_t width, const uint32_t height);
        ~FrameBuffer();

        void set_image(uint32_t* data);

    private:

        uint32_t m_width, m_height;

        GLuint m_texture;
        GLuint m_vertexShader;
        GLuint m_fragmentShader;
        GLuint m_pipeline;
    };

}

#endif
