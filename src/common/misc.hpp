#pragma once

#include <type_traits>
#include <chrono>
#include <string>

namespace util
{
    using Clock = std::conditional_t<std::chrono::high_resolution_clock::is_steady, std::chrono::high_resolution_clock, std::chrono::steady_clock>;

    template <typename Rep, typename Period>
    static std::string DurationToString(std::chrono::duration<Rep, Period> duration)
    {
        double milliseconds = std::chrono::duration<double, std::milli>(duration).count();
        std::string str = std::to_string(milliseconds);
        str += " ms";
        return str;
    }
}