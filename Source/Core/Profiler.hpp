#ifndef PICO_PROFILER_HPP
#define PICO_PROFILER_HPP

#include <filesystem>


namespace Core
{
    class Profiler
    {
    public:

        Profiler() = default;

        virtual Profiler* CloneForThread() = 0;

        virtual void StartEvent(const char*) = 0;


        virtual void EndEvent() = 0;

        void DumpStatsToFile(const std::filesystem::path& file) const;
    };

    // Only profiler outer scopes (ignore inner)
    class MacroProfiler : public Profiler
    {
    public:

        MacroProfiler(MacroProfiler*);

        virtual Profiler* CloneForThread() override final;

        virtual void StartEvent(const char*) override final;

        virtual void EndEvent() override final;

    private:

        MacroProfiler* m_parent;
    };

    // Log all scopes
    class MicroProfiler : public Profiler
    {
    public:

        MicroProfiler(MicroProfiler*);

        virtual Profiler* CloneForThread() override final;

        virtual void StartEvent(const char*) override final;

        virtual void EndEvent() override final;
    };
}

#endif
