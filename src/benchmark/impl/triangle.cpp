#include <triangulator_benchmark.h>

#define TRILIBRARY
#define NO_TIMER
#define ANSI_DECLARATORS
#define CDT_ONLY
// Note that the source file is included to simplify things, since it sets up other defines,
// and this allows us to use Triangle without creating a separate CMake project
#include <3rdParty/triangle/triangle.c>

using namespace util;

std::string TriangleTriangulator::name() const
{
    return "triangle";
}

TriangulatorCapabilities TriangleTriangulator::getCapabilities() const
{
    uint8_t flags =
        static_cast<uint8_t>(TriangulatorCapabilities::MultipleOutlines) |
        static_cast<uint8_t>(TriangulatorCapabilities::AutoDetectPolylineType);

    return static_cast<TriangulatorCapabilities>(flags);
}

Clock::duration TriangleTriangulator::triangulateAndMeasure(const TriangulationInput& input, std::vector<Triangle>& resultTriangles) const
{
    // p - use planar straight line graph
    // z - index from 0
    // Q - quiet mode
    std::string flags = "pzQ";

    bool delaunayOnly = input.outlines.empty() && input.holes.empty();
    if (delaunayOnly)
    {
        // c - encloses the convex hull with segments
        flags.push_back('c');
    }

    struct triangulateio in { };

    std::vector<int> segments;
    for (const auto& outline : input.outlines)
    {
        size_t prevIndex = outline.size() - 1;
        for (size_t i = 0; i < outline.size(); ++i)
        {
            segments.push_back(outline[prevIndex]);
            segments.push_back(outline[i]);
            prevIndex = i;
        }
    }

    std::vector<Vec2> holePoints;
    for (const auto& hole : input.holes)
    {
        size_t prevIndex = hole.size() - 1;
        for (size_t i = 0; i < hole.size(); ++i)
        {
            segments.push_back(hole[prevIndex]);
            segments.push_back(hole[i]);
            prevIndex = i;
        }

        // Since holes can only be set by positions, select the midpoint of a segment and offset it a bit in a perpendicular direction
        // This requires that the segments are oriented CCW (reversed segments will flip the direction),
        // and that there is no other point or segment within epsilon distance of the midpoint
        // Also, this is not robust, but shouldn't be a problem in the benchmarks

        Vec2 first = input.points[hole.front()];
        Vec2 last = input.points[hole.back()];

        Scalar dirX = last.x - first.x;
        Scalar dirY = last.y - first.y;

        Scalar centerX = first.x + dirX * 0.5;
        Scalar centerY = first.y + dirY * 0.5;

        constexpr Scalar eps = 1e-12;
        Scalar perpendicularDirX = dirY;
        Scalar perpendicularDirY = -dirX;

        Scalar holeX = centerX + perpendicularDirX * eps;
        Scalar holeY = centerY + perpendicularDirY * eps;

        holePoints.emplace_back(holeX, holeY);
    }

    in.pointlist = (Scalar*)input.points.data();
    in.numberofpoints = (int)input.points.size();

    in.segmentlist = segments.data();
    in.numberofsegments = (int)(segments.size() / 2);

    in.holelist = (Scalar*)holePoints.data();
    in.numberofholes = (int)holePoints.size();

    struct triangulateio out { };

    Clock::time_point start = Clock::now();
    triangulate(flags.data(), &in, &out, nullptr);
    Clock::time_point end = Clock::now();

    size_t numTriangles = size_t(out.numberoftriangles);
    resultTriangles.reserve(numTriangles);

    for (size_t i = 0; i < numTriangles; ++i)
    {
        size_t arrayIndex = i * 3;
        resultTriangles.emplace_back(
            out.trianglelist[arrayIndex],
            out.trianglelist[arrayIndex + 1],
            out.trianglelist[arrayIndex + 2]
        );
    }

    trifree((int*)out.pointlist);
    trifree((int*)out.pointattributelist);
    trifree((int*)out.trianglelist);
    trifree((int*)out.triangleattributelist);
    trifree((int*)out.pointmarkerlist);
    trifree((int*)out.segmentlist);
    trifree((int*)out.segmentmarkerlist);
    // Holes points are not copied, only the pointer

    return end - start;
}
