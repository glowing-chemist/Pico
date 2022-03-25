#ifndef PICO_ASSERTS_HPP
#define PICO_ASSERTS_HPP

#include "glm/glm.hpp"

#ifndef NDEBUG

#define PICO_DEBUG_BREAK() __asm("int3")

#define PICO_ASSERT_VALID(v) if(glm::any(glm::isnan(v)) || glm::any(glm::isinf(v))) \
                                PICO_DEBUG_BREAK();

#define PICO_ASSERT(c) if(!(c)) \
                            PICO_DEBUG_BREAK();

#

#else

#define PICO_DEBUG_BREAK()

#define PICO_ASSERT_VALID(v)

#define PICO_ASSERT(c)

#endif


#endif

