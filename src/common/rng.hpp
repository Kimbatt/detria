#pragma once

#include <cstdint>
#include <cstring>

namespace util
{
    // Website: https://www.romu-random.org/
    // Copyright 2020 Mark A. Overton
    // Licensed under the Apache License, Version 2.0, see http://www.apache.org/licenses/LICENSE-2.0

    class RomuTrio
    {
    public:

        RomuTrio(uint64_t seedX = 0, uint64_t seedY = 0) : xState(seedX), yState(seedY), zState(seedY + 1) // Requires at least one non-zero seed
        {
            nextU64();
            nextU64();
            nextU64();
            nextU64();
        }

        inline uint64_t nextU64()
        {
            uint64_t xp = xState;
            uint64_t yp = yState;
            uint64_t zp = zState;

            xState = 15241094284759029579u * zp;

            yState = yp - xp;
            yState = rotl<uint64_t, 12>(yState);

            zState = zp - yp;
            zState = rotl<uint64_t, 44>(zState);

            return xp;
        }

        double nextDouble()
        {
            // Directly construct the bits of a 64-bit float
            // Layout: 1 sign bit, 11 exponent bits, 52 mantissa bits
            // If we start with 1.0 (0x3ff0000000000000), we can set the mantissa bits to any random value,
            // which will give an uniformly distributed random value in [1.0, 2.0)
            // Finally, we need to subtract 1.0

            constexpr uint64_t doubleOneBits = 0x3ff0000000000000; // Bit representation of 1.0
            constexpr uint64_t mask = 0x000fffffffffffffu; // Low 52 bits

            uint64_t randomValue = nextU64();
            uint64_t randomMantissaBits = randomValue & mask;
            uint64_t bits = doubleOneBits | randomMantissaBits;

            static_assert(sizeof(uint64_t) == sizeof(double));
            static_assert(alignof(uint64_t) == alignof(double));

            // Convert bits to double
            double d{ };
            std::memcpy(&d, &bits, sizeof(double));
            return d - 1.0;
        }

    private:

        template <typename T, T lrot>
        inline static T rotl(T d)
        {
            return (d << lrot) | (d >> (T(8 * sizeof(T)) - lrot));
        }

    private:
        uint64_t xState;
        uint64_t yState;
        uint64_t zState;
    };
}
