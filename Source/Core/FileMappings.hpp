#ifndef PICO_FILE_MAPPINGS_HPP
#define PICO_FILE_MAPPINGS_HPP

#include <filesystem>
#include <unordered_map>


namespace Core
{

    class File_System_Mappings
    {
    public:

        File_System_Mappings(const std::filesystem::path& root);
        ~File_System_Mappings() = default;

        std::filesystem::path resolve_path(const std::filesystem::path& path) const;

    private:

        void populate_directory(const std::filesystem::path& path);

        std::filesystem::path m_root;

        std::unordered_map<size_t, std::filesystem::path> m_mappings;

    };

}

#endif
