#include "Options.hpp"

namespace Util
{

    Options::Options(const char** cmd, const uint32_t argCount) :
        m_camera_position(0.0f, 0.0f, 0.0f),
        m_camera_direction(1.0f, 0.0f, 0.0f),
        m_camera_name("MainCamera"),
        m_skybox("skybox"),
        m_scene_file("./scene.json"),
        m_output_file("./output.hdr"),
        m_resolution(1920, 1080),
        m_sample_count(256),
        m_sun_direction(0.0f, -1.0f, 0.0f),
        m_sun_colour(1.0f, 1.0f, 1.0f),
        m_denoise(false),
        m_tonemap(false),
        m_option_bitset{0}
    {
        for(uint32_t i = 1; i < argCount; ++i)
        {
            if(strcmp(cmd[i], "-Skybox") == 0)
            {
                m_option_bitset |= Option::kSkybox;
                m_skybox = std::string(cmd[++i]);
            }
            else if(strcmp(cmd[i], "-CameraPosition") == 0)
            {
                const float x = std::atof(cmd[++i]);
                const float y = std::atof(cmd[++i]);
                const float z = std::atof(cmd[++i]);

                m_option_bitset |= Option::kCameraPosition;
                m_camera_position =  glm::vec3(x, y, z);
            }
            else if(strcmp(cmd[i], "-CameraDirection") == 0)
            {
                const float x = std::atof(cmd[++i]);
                const float y = std::atof(cmd[++i]);
                const float z = std::atof(cmd[++i]);

                m_option_bitset |= Option::kCameraDirection;
                m_camera_direction = glm::vec3(x, y, z);
            }
            else if(strcmp(cmd[i], "-Camera") == 0)
            {
                m_option_bitset |= Option::kCameraName;
                m_camera_name = cmd[++i];
            }
            else if(strcmp(cmd[i], "-Scene") == 0)
            {
                m_option_bitset |= Option::kSceneFile;
                m_scene_file = std::string(cmd[++i]);
            }
            else if(strcmp(cmd[i], "-OutputFile") == 0)
            {
                m_option_bitset |= Option::kOutputFile;
                m_output_file = std::string(cmd[++i]);
            }
            else if(strcmp(cmd[i], "-Resolution") == 0)
            {
               const uint32_t resX = std::atoi(cmd[++i]);
               const uint32_t resY = std::atoi(cmd[++i]);

               m_option_bitset |= Option::kResolution;
               m_resolution = glm::uvec2(resX, resY);
            }
            else if(strcmp(cmd[i], "-SampleCount") == 0)
            {
                const uint32_t sample_count = std::atoi(cmd[++i]);

                m_option_bitset |= Option::kSampleCount;
                m_sample_count = sample_count;
            }
            else if(strcmp(cmd[i], "-SunDirection") == 0)
            {
                const float x = std::atof(cmd[++i]);
                const float y = std::atof(cmd[++i]);
                const float z = std::atof(cmd[++i]);

                m_option_bitset |= Option::kSunDirection;
                m_sun_direction = glm::vec3(x, y, z);
            }
            else if(strcmp(cmd[i], "-SunColour") == 0)
            {
                const float x = std::atof(cmd[++i]);
                const float y = std::atof(cmd[++i]);
                const float z = std::atof(cmd[++i]);

                m_option_bitset |= Option::kSunColour;
                m_sun_colour = glm::vec3(x, y, z);
            }
            else if(strcmp(cmd[i], "-Denoise") == 0)
            {
                m_option_bitset |= Option::kDenoise;
                m_denoise = true;
            }
            else if(strcmp(cmd[i], "-Tonemap") == 0)
            {
                m_option_bitset |= Option::kToneMap;
                m_tonemap = true;
            }
            else
            {
                printf("Unrecognised command %s \n", cmd[i]);
            }
        }
    }

}
