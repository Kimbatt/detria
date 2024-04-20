#include <unordered_map>

#include <triangulator_benchmark.h>

#include <poly2tri/poly2tri.h>

using namespace util;

std::string Poly2triTriangulator::name() const
{
    return "poly2tri";
}

TriangulatorCapabilities Poly2triTriangulator::getCapabilities() const
{
    return static_cast<TriangulatorCapabilities>(0);
}

Clock::duration Poly2triTriangulator::triangulateAndMeasure(const TriangulationInput& input, std::vector<Triangle>& resultTriangles) const
{
    std::vector<p2t::Point> points;
    std::vector<p2t::Point> steinerPoints;
    std::vector<std::vector<p2t::Point>> holes;

    // Check if a point is referenced by a polyline, if not, then add them as steiner points
    std::vector<bool> referencedPoints(input.points.size());

    // Only one outline is supported in poly2tri
    for (Idx idx : input.outlines[0])
    {
        Vec2 p = input.points[idx];
        points.emplace_back(p.x, p.y);

        referencedPoints[idx] = true;
    }

    for (const auto& hole : input.holes)
    {
        auto& holePoints = holes.emplace_back();

        for (Idx idx : hole)
        {
            Vec2 p = input.points[idx];
            holePoints.emplace_back(p.x, p.y);

            referencedPoints[idx] = true;
        }
    }

    std::vector<Idx> steinerPointIndices;
    for (size_t i = 0; i < referencedPoints.size(); ++i)
    {
        if (referencedPoints[i])
        {
            continue;
        }

        Vec2 p = input.points[i];
        steinerPoints.emplace_back(p.x, p.y);
        steinerPointIndices.push_back(i);
    }

    std::vector<p2t::Point*> pointPtrs;
    std::vector<std::vector<p2t::Point*>> holePtrs;

    for (auto& point : points)
    {
        pointPtrs.push_back(&point);
    }

    for (auto& hole : holes)
    {
        auto& holePointPtrs = holePtrs.emplace_back();

        for (p2t::Point& point : hole)
        {
            holePointPtrs.push_back(&point);
        }
    }

    // Must be set up after all points are added, so the vectors don't reallocate (and invalidate the pointers)
    std::unordered_map<p2t::Point*, Idx> pointPtrToIndex;

    auto addPointersToMap = [&](const std::vector<Idx>& pointIndices, const std::vector<p2t::Point*>& pointPtrs)
    {
        for (size_t i = 0; i < pointPtrs.size(); ++i)
        {
            p2t::Point* ptr = pointPtrs[i];
            Idx idx = pointIndices[i];

            pointPtrToIndex[ptr] = idx;
        }
    };

    addPointersToMap(input.outlines[0], pointPtrs);

    for (size_t i = 0; i < holePtrs.size(); ++i)
    {
        addPointersToMap(input.holes[i], holePtrs[i]);
    }

    for (size_t i = 0; i < steinerPointIndices.size(); ++i)
    {
        Idx idx = steinerPointIndices[i];
        p2t::Point* ptr = &steinerPoints[i];

        pointPtrToIndex[ptr] = idx;
    }

    p2t::CDT cdt(pointPtrs);

    for (auto& hole : holePtrs)
    {
        cdt.AddHole(hole);
    }

    for (auto& steiner : steinerPoints)
    {
        cdt.AddPoint(&steiner);
    }

    Clock::time_point start = Clock::now();
    cdt.Triangulate();
    Clock::time_point end = Clock::now();

    std::vector triangulated = cdt.GetTriangles(); // Returns a copy

    resultTriangles.reserve(triangulated.size());

    for (size_t i = 0; i < triangulated.size(); ++i)
    {
        p2t::Triangle* tri = triangulated[i];

        resultTriangles.emplace_back(
            pointPtrToIndex[tri->GetPoint(0)],
            pointPtrToIndex[tri->GetPoint(1)],
            pointPtrToIndex[tri->GetPoint(2)]
        );
    };

    return end - start;
}
