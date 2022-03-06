#ifndef LOADABLE_HPP
#define LOADABLE_HPP

#include <cstddef>

namespace Core
{

    class Loadable
    {
    public:

        Loadable() = default;
        virtual ~Loadable() = default;

        virtual size_t get_residence_size() const = 0;

        virtual bool is_resident() const = 0;

        virtual void make_resident(void*) = 0;

        virtual void make_nonresident() = 0;
    };

}

#endif
