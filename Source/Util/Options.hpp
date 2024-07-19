#ifndef PICO_OPTIONS_HPP
#define PICO_OPTIONS_HPP

#include <array>
#include <string>
#include <variant>
#include <vector>

#include "glm/common.hpp"

namespace Util
{

    enum class Option
    {
        kNone = 0,
        kCameraPosition,
        kCameraDirection,
        kCameraName,
        kSkybox,
        kSceneFile,
        kOutputFile,
        kResolution,
        kSampleCount,

        kCount
    };

    template<Option O>
    struct option_map { using Type = void; };

    template<>
    struct option_map<Option::kCameraPosition> { using Type = glm::vec3; };

    template<>
    struct option_map<Option::kCameraDirection> { using Type = glm::vec3; };

    template<>
    struct option_map<Option::kCameraName> { using Type = std::string; };

    template<>
    struct option_map<Option::kSkybox> { using Type = std::array<std::string, 6>; };

    template<>
    struct option_map<Option::kSceneFile> { using Type = std::string; };

    template<>
    struct option_map<Option::kOutputFile> { using Type = std::string; };

    template<>
    struct option_map<Option::kResolution> { using Type = glm::ivec2; };

    template<>
    struct option_map<Option::kSampleCount> { using Type = uint32_t; };

    template<Option O>
    using option_type = typename option_map<O>::Type;

    class Options
    {
    public:

        Options(const char** cmd, const uint32_t argCount);

        using Value = std::variant<std::monostate, glm::ivec2, glm::vec3, std::string, std::array<std::string, 6>, uint32_t>;

        bool has_option(const Option o)
        {
            for(uint32_t i = 0; i < m_values.size(); ++i)
            {
                if(m_values[i].m_type == o)
                    return true;
            }

            return false;
        }

        template<Option O>
        option_type<O> get_option() const
        {

            for(uint32_t i = 0; i < m_values.size(); ++i)
            {
                if(m_values[i].m_type == O)
                    return std::get<option_type<O>>(m_values[i].m_value);
            }

            return option_type<O>{};
        }

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
