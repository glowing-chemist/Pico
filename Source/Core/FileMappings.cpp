#include "FileMappings.hpp"

namespace Core
{

    File_System_Mappings::File_System_Mappings(const std::filesystem::path& root) :
        m_root(root),
        m_mappings{}
    {
        populate_directory(m_root);
    }

    void File_System_Mappings::populate_directory(const std::filesystem::path& path)
    {
        for(const auto& child : std::filesystem::directory_iterator(path))
        {
            if(child.is_regular_file())
            {
                std::string child_path = child.path().string();

                const std::filesystem::path relative_path = std::filesystem::relative(child, m_root);

                // Create the all lower-case mappign entry.
                std::transform(child_path.begin(), child_path.end(), child_path.begin(),
                    [](unsigned char c){ return std::tolower(c); });

                m_mappings[std::hash<std::string>{}(child_path)] = relative_path;
            }
            else if(child.is_directory())
                populate_directory(child);
        }
    }

    std::filesystem::path File_System_Mappings::resolve_path(const std::filesystem::path& path) const
    {
        std::string lower_path;
        if(path.is_relative())
            lower_path = (m_root / path).lexically_normal().string();
        else
            lower_path = path.lexically_normal().string();

        std::transform(lower_path.begin(), lower_path.end(), lower_path.begin(),
            [](unsigned char c){ return std::tolower(c); });

        std::replace(lower_path.begin(), lower_path.end(),
             #ifdef _MSC_VER
                '/', '\\'
             #else
                '\\', '/'
             #endif
                     );

        if(auto it = m_mappings.find(std::hash<std::string>{}(lower_path)); it != m_mappings.end())
            return m_root / it->second;
        else
            return {};
    }

}
