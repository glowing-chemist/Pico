#include "Profiler.hpp"


namespace Core
{
    void Profiler::DumpStatsToFile(const  std::filesystem::path& file) const
    {
        if(!std::filesystem::exists(file))
        {
            std::filesystem::create_directories(file.parent_path());
        }


    }

    MacroProfiler::MacroProfiler(MacroProfiler* parent) :
    m_parent(parent)
    {
    }

    Profiler* MacroProfiler::CloneForThread()
    {
        return new MacroProfiler(this);
    }

    void MacroProfiler::StartEvent(const char* tag)
    {

    }

    void MacroProfiler::EndEvent()
    {

    }
}
