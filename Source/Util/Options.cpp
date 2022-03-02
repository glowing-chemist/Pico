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
                // TODO
            }
            else if(strcmp(cmd[i], "-SceneFile") == 0)
            {
                m_values.push_back({Option::kSceneFile, std::string(cmd[++i])});
            }
            else if(strcmp(cmd[i], "-OutputFile") == 0)
            {
                m_values.push_back({Option::kOutputFile, std::string(cmd[++i])});
            }
            else if(strcmp(cmd[i], "Resolution") == 0)
            {
               // TODO
            }
            else
            {
                printf("Unrecognised command %s \n", cmd[i]);
            }
        }
    }


     Options::Value Options::get_option(const Option option) const
    {
        for(uint32_t i = 0; i < m_values.size(); ++i)
        {
            if(m_values[i].m_type == option)
                return m_values[i].m_value;
        }

        return std::monostate{};
    }

}
