#pragma once

#include <vector>

#include "rng.hpp"

namespace util
{
    template <typename Point, typename Idx>
    static void generateRandomPoints(size_t numPoints, std::vector<Point>& resultPoints, std::vector<Idx>& resultOutline)
    {
        using Scalar = decltype(Point::x);

        constexpr Scalar boxExtents = Scalar(1000);
        constexpr Scalar padding = Scalar(10);

        constexpr Scalar rngMin = -boxExtents + padding;
        constexpr Scalar rngMax = boxExtents - padding;
        constexpr Scalar rngRange = rngMax - rngMin;

        RomuTrio rng(0, 0);

        resultPoints.emplace_back(Point{ -boxExtents, -boxExtents });
        resultPoints.emplace_back(Point{ boxExtents, -boxExtents });
        resultPoints.emplace_back(Point{ boxExtents, boxExtents });
        resultPoints.emplace_back(Point{ -boxExtents, boxExtents });

        resultOutline.push_back(0);
        resultOutline.push_back(1);
        resultOutline.push_back(2);
        resultOutline.push_back(3);

        for (size_t i = 0; i < numPoints; ++i)
        {
            Scalar x = rngMin + Scalar(rng.nextDouble()) * rngRange;
            Scalar y = rngMin + Scalar(rng.nextDouble()) * rngRange;

            resultPoints.emplace_back(Point{ x, y });
        }
    }

    template <typename Point, typename Idx>
    static void fractal(size_t depth, decltype(Point::x) cx, decltype(Point::x) cy, decltype(Point::x) size, decltype(Point::x) thickness,
        std::vector<Point>& points, std::vector<std::pair<std::vector<Idx>, bool>>& polylines)
    {
        using Scalar = decltype(Point::x);

        auto fractalInner = [&](auto self, size_t currentDepth, Scalar currentX, Scalar currentY, Scalar currentSize, Scalar currentThickness) -> void
        {
            Idx startIndex = Idx(points.size());

            points.emplace_back(Point{ currentX - currentSize, currentY - currentSize });
            points.emplace_back(Point{ currentX + currentSize, currentY - currentSize });
            points.emplace_back(Point{ currentX + currentSize, currentY + currentSize });
            points.emplace_back(Point{ currentX - currentSize, currentY + currentSize });

            polylines.emplace_back(std::make_pair(
                std::vector<Idx>{ startIndex, startIndex + 1, startIndex + 2, startIndex + 3 },
                currentDepth % 2 == 0
            ));

#if false // Can get a different fractal with this
            points.emplace_back(Point{ cx - size + thickness, cy - size + thickness });
            points.emplace_back(Point{ cx + size - thickness, cy - size + thickness });
            points.emplace_back(Point{ cx + size - thickness, cy + size - thickness });
            points.emplace_back(Point{ cx - size + thickness, cy + size - thickness });

            polylines.emplace_back(Polyline<Idx>
            {
                .pointIndices = { startIndex + 4, startIndex + 5, startIndex + 6, startIndex + 7 },
                    .type = PolylineType::AutoDetect
            });
#endif

            if (currentDepth < depth)
            {
                constexpr Scalar sizeFactor = Scalar(0.35);
                constexpr Scalar spacingFactor = Scalar(0.425);

                Scalar half = currentSize * spacingFactor;
                self(self, currentDepth + 1, currentX - half, currentY - half, currentSize * sizeFactor, currentThickness * sizeFactor);
                self(self, currentDepth + 1, currentX + half, currentY - half, currentSize * sizeFactor, currentThickness * sizeFactor);
                self(self, currentDepth + 1, currentX + half, currentY + half, currentSize * sizeFactor, currentThickness * sizeFactor);
                self(self, currentDepth + 1, currentX - half, currentY + half, currentSize * sizeFactor, currentThickness * sizeFactor);
            }
        };

        fractalInner(fractalInner, 0, cx, cy, size, thickness);
    }
}
