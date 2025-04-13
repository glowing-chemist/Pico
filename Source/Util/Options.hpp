#ifndef PICO_OPTIONS_HPP
#define PICO_OPTIONS_HPP

#include <array>
#include <string>
#include <variant>
#include <vector>

#include "glm/common.hpp"

namespace Util
{

    enum Option : uint32_t
    {
        kCameraPosition = 1,
        kCameraDirection = 1 << 1,
        kCameraName = 1 << 2,
        kSkybox = 1 << 3,
        kSceneFile = 1 << 4,
        kOutputFile = 1 << 5,
        kResolution = 1 << 6,
        kSampleCount = 1 << 7,
        kSunDirection = 1 << 8,
        kSunColour = 1 << 9,

        kCount = 10
    };


    class Options
    {
    public:

        Options(const char** cmd, const uint32_t argCount);

        bool has_option(const Option o) const
        {
            return (static_cast<uint32_t>(o) & m_option_bitset) > 0;
        }

    glm::vec3 m_camera_position;
    glm::vec3 m_camera_direction; 
    std::string m_camera_name;
    std::string m_skybox;
    std::string m_scene_file;
    std::string m_output_file;
    glm::uvec2 m_resolution;
    uint32_t m_sample_count;
    glm::vec3 m_sun_direction;
    glm::vec3  m_sun_colour;

    private:
    uint32_t m_option_bitset;

    };


}

#endif
