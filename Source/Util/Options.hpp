#ifndef PICO_OPTIONS_HPP
#define PICO_OPTIONS_HPP

#include <string>
#include <variant>
#include <vector>

#include "glm/common.hpp"

namespace Util
{

    enum class Option
    {
        kNone,
        kCameraPosition,
        kSkybox,
        kSceneFile,
        kOutputFile,
        kResolution
    };

    class Options
    {
    public:

        Options(const char** cmd, const uint32_t argCount);

        using Value = std::variant<std::monostate, glm::ivec2, glm::vec3, std::string>;

        Value get_option(const Option) const;

    private:

        struct OptionValue
        {
            Option m_type;
            Value m_value;
        };

        std::vector<OptionValue> m_values;

    };


}

#endif
