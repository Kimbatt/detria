module;

#include <vector>
#include <array>
#include <cmath>
#include <cstdint>
#include <type_traits>
#include <optional>
#include <variant>
#include <string>
#include <sstream>

#if !defined(NDEBUG)
#include <iostream>
#include <limits>

#if !defined(_WIN32) || !_WIN32
#include <csignal>
#endif

#endif

export module detria;

#define DETRIA_CXXMODULE

#include "detria.hpp"
