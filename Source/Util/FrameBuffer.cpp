#include "FrameBuffer.hpp"
#include "Core/vectorUtils.hpp"

#include <iostream>


namespace Util
{

    constexpr const char* kVertexSource =
           "#version 330 core\n"
           "out vec2 UV;\n "
            "void main()\n"
            "{\n"
                "vec2 uv = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);\n"
                "UV = uv;\n"
                "gl_Position = vec4(uv * 2.0f + -1.0f, 0.0f, 1.0f);\n"
            "}\n";

    constexpr const char* kfragmentSource =
           "#version 330 core\n"
           "in vec2 UV;\n"
           "layout(location = 0) out vec4 fragColour;"
           "uniform sampler2D tex;\n"
            "void main()\n"
            "{\n"
                "fragColour = texture(tex, UV);\n"
            "}\n";



    FrameBuffer::FrameBuffer(uint32_t width, const uint32_t height) :
        m_width(width),
        m_height(height),
        m_texture{0},
        m_vertexShader{0},
        m_fragmentShader{0},
        m_pipeline{0}
    {
        {
            glGenTextures(1, &m_texture);
            glBindTexture(GL_TEXTURE_2D, m_texture);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        }

        {
            m_vertexShader = glCreateShader(GL_VERTEX_SHADER);
            glShaderSource(m_vertexShader, 1, &kVertexSource, NULL);
            glCompileShader(m_vertexShader);
        }

        {
            m_fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
            glShaderSource(m_fragmentShader, 1, &kfragmentSource, NULL);
            glCompileShader(m_fragmentShader);

        }

        {
            m_pipeline = glCreateProgram();
            glAttachShader(m_pipeline, m_vertexShader);
            glAttachShader(m_pipeline, m_fragmentShader);
            glLinkProgram(m_pipeline);

            glUseProgram(m_pipeline);

            glUniform1i(glGetUniformLocation(m_pipeline, "tex"), 0);

            // Bind a dummy vao.
            GLuint vao = 0;
            glGenVertexArrays( 1, &vao );
            glBindVertexArray( vao );
        }

    }


    FrameBuffer::~FrameBuffer()
    {
        glDeleteTextures(1, &m_texture);

        glDeleteProgram(m_pipeline);

        glDeleteShader(m_vertexShader);
        glDeleteShader(m_fragmentShader);


    }


    void FrameBuffer::set_image(glm::vec4* data)
    {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_width, m_height, 0, GL_RGBA,  GL_FLOAT, data);

        glUseProgram(m_pipeline);

        glDrawArrays(GL_TRIANGLES, 0, 3);
    }

}
