#ifndef PICO_FRAEMBUFFER_HPP
#define PICO_FRAEMBUFFER_HPP

#include <glad/glad.h>
#include "GLFW/glfw3.h"

#include "glm/vec4.hpp"

#include <cstdint>
#include <vector>

namespace Util
{

    class FrameBuffer
    {
    public:

        FrameBuffer(uint32_t width, const uint32_t height);
        ~FrameBuffer();

        void set_image(glm::vec4* data);

    private:

        uint32_t m_width, m_height;

        GLuint m_texture;
        GLuint m_vertexShader;
        GLuint m_fragmentShader;
        GLuint m_pipeline;
    };

}

#endif
