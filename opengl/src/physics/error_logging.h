#pragma once

#include <initializer_list>
#include <iostream>

namespace PhysicsDebug {
    struct Check {
        bool passed;
        const char* failureMessage;
    };

    template<typename Handle>
    inline void logFirstFailedCheck(
        const char* functionName,
        const Handle& handle,
        std::initializer_list<Check> checks)
    {
#ifdef PHYSICS_ENABLE_LOGGING
        for (const Check& check : checks) {
            if (!check.passed) {
                std::cerr
                    << '[' << functionName << "] "
                    << check.failureMessage
                    << " Handle: "
                    << handle.slot << ':' << handle.gen << '\n';

                return;
            }
        }
#else
        (void)functionName;
        (void)handle;
        (void)checks;
#endif
    }
}