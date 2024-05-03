#pragma once

#include <type_traits>
#include <chrono>
#include <string>
#include <sstream>

namespace util
{
    using Clock = std::conditional_t<std::chrono::high_resolution_clock::is_steady, std::chrono::high_resolution_clock, std::chrono::steady_clock>;

    template <typename T>
    static std::string ToStringWithPrecision(T value, std::streamsize precision)
    {
        std::stringstream ss;
        ss.precision(precision);
        ss << std::fixed;
        ss << value;
        return ss.str();
    }

    template <typename Rep, typename Period>
    static std::string DurationToString(std::chrono::duration<Rep, Period> duration)
    {
        double milliseconds = std::chrono::duration<double, std::milli>(duration).count();
        std::string str = ToStringWithPrecision(milliseconds, 2);
        str.append(" ms");
        return str;
    }
}
