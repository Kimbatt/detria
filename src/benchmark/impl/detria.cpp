#include <triangulator_benchmark.h>

#include <detria.hpp>

using namespace util;

std::string DetriaTriangulator::name() const
{
    return "detria";
}

TriangulatorCapabilities DetriaTriangulator::getCapabilities() const
{
    uint8_t flags =
        static_cast<uint8_t>(TriangulatorCapabilities::MultipleOutlines) |
        static_cast<uint8_t>(TriangulatorCapabilities::AutoDetectPolylineType);

    return static_cast<TriangulatorCapabilities>(flags);
}

Clock::duration DetriaTriangulator::triangulateAndMeasure(const TriangulationInput& input, std::vector<Triangle>& resultTriangles) const
{
    detria::Triangulation<Vec2, Idx> triangulation;

    triangulation.setPoints(input.points);

    for (const auto& outline : input.outlines)
    {
        triangulation.addOutline(outline);
    }

    for (const auto& hole : input.holes)
    {
        triangulation.addHole(hole);
    }

    bool delaunayOnly = input.outlines.empty() && input.holes.empty();

    Clock::time_point start = Clock::now();

    bool success = triangulation.triangulate(true);

    resultTriangles.reserve(triangulation.getMaxNumTriangles());

    if (delaunayOnly)
    {
        triangulation.forEachTriangleOfEveryLocation([&](const auto& tri)
        {
            resultTriangles.emplace_back(tri.x, tri.y, tri.z);
        }, false);
    }
    else
    {
        triangulation.forEachTriangle([&](const auto& tri)
        {
            resultTriangles.emplace_back(tri.x, tri.y, tri.z);
        }, false);
    }

    Clock::time_point end = Clock::now();

    return end - start;
}
