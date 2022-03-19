#include "Options.hpp"

namespace Util
{

    Options::Options(const char** cmd, const uint32_t argCount) :
        m_values{}
    {
        for(uint32_t i = 1; i < argCount; ++i)
        {
            if(strcmp(cmd[i], "-Skybox") == 0)
            {
                m_values.push_back({Option::kSkybox, std::string(cmd[++i])});
            }
            else if(strcmp(cmd[i], "-CameraPosition") == 0)
            {
                const float x = std::atof(cmd[++i]);
                const float y = std::atof(cmd[++i]);
                const float z = std::atof(cmd[++i]);

                m_values.push_back({Option::kCameraPosition, glm::vec3(x, y, z)});
            }
            else if(strcmp(cmd[i], "-CameraDirection") == 0)
            {
                const float x = std::atof(cmd[++i]);
                const float y = std::atof(cmd[++i]);
                const float z = std::atof(cmd[++i]);

                m_values.push_back({Option::kCameraDirection, glm::vec3(x, y, z)});
            }
            else if(strcmp(cmd[i], "-Scene") == 0)
            {
                m_values.push_back({Option::kSceneFile, std::string(cmd[++i])});
            }
            else if(strcmp(cmd[i], "-OutputFile") == 0)
            {
                m_values.push_back({Option::kOutputFile, std::string(cmd[++i])});
            }
            else if(strcmp(cmd[i], "-Resolution") == 0)
            {
               const uint32_t resX = std::atoi(cmd[++i]);
               const uint32_t resY = std::atoi(cmd[++i]);

               m_values.push_back({Option::kResolution, glm::ivec2(resX, resY)});
            }
            else
            {
                printf("Unrecognised command %s \n", cmd[i]);
            }
        }
    }

}
