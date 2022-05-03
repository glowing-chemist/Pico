#ifndef PICO_ASSERTS_HPP
#define PICO_ASSERTS_HPP

#include "glm/glm.hpp"

#define PICO_LOG(f, ...) printf(f, __VA_ARGS__)

#ifndef NDEBUG

#ifdef _WIN32
#define PICO_DEBUG_BREAK() __debugbreak();
#else
#define PICO_DEBUG_BREAK() __asm("int3")
#endif

#define PICO_ASSERT_VALID(v) if(glm::any(glm::isnan(v)) || glm::any(glm::isinf(v))) \
                                PICO_DEBUG_BREAK();

#define PICO_ASSERT(c) if(!(c)) \
                            PICO_DEBUG_BREAK();

#define PICO_ASSERT_NORMALISED(v) PICO_ASSERT(glm::length(v) > 0.99f && glm::length(v) < 1.01f)

#

#else

#define PICO_DEBUG_BREAK()

#define PICO_ASSERT_VALID(v)

#define PICO_ASSERT(c)

#define PICO_ASSERT_NORMALISED(v)

#endif


#endif

