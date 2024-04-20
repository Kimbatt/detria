#include <triangulator_benchmark.h>

#include <CDT.h>

using namespace util;

std::string CdtTriangulator::name() const
{
    return "cdt";
}

TriangulatorCapabilities CdtTriangulator::getCapabilities() const
{
    uint8_t flags =
        static_cast<uint8_t>(TriangulatorCapabilities::MultipleOutlines) |
        static_cast<uint8_t>(TriangulatorCapabilities::AutoDetectPolylineType);

    return static_cast<TriangulatorCapabilities>(flags);
}

Clock::duration CdtTriangulator::triangulateAndMeasure(const TriangulationInput& input, std::vector<Triangle>& resultTriangles) const
{
    // The inputs are prepared in the expected format, because the triangulation process already starts when we start adding the points and edges
    // There is no single `triangulate` function that could be used for benchmark

    std::vector<CDT::V2d<Scalar>> points;
    std::vector<CDT::EdgeVec> edgeVec;

    for (const Vec2& p : input.points)
    {
        points.emplace_back(CDT::V2d<Scalar>::make(p.x, p.y));
    }

    auto addPolyline = [&](const std::vector<Idx>& polyline)
    {
        auto& target = edgeVec.emplace_back();

        size_t prevIndex = polyline.size() - 1;
        for (size_t i = 0; i < polyline.size(); ++i)
        {
            target.emplace_back(polyline[prevIndex], polyline[i]);
            prevIndex = i;
        }
    };

    for (const auto& outline : input.outlines)
    {
        addPolyline(outline);
    }

    for (const auto& hole : input.holes)
    {
        addPolyline(hole);
    }

    bool delaunayOnly = edgeVec.empty();

    Clock::time_point start = Clock::now();

    CDT::Triangulation<Scalar> cdt;
    cdt.insertVertices(points);

    for (const auto& edges : edgeVec)
    {
        cdt.insertEdges(edges);
    }

    if (delaunayOnly)
    {
        cdt.eraseSuperTriangle();
    }
    else
    {
        cdt.eraseOuterTrianglesAndHoles();
    }

    Clock::time_point end = Clock::now();

    resultTriangles.reserve(cdt.triangles.size());

    for (size_t i = 0; i < cdt.triangles.size(); ++i)
    {
        const auto& vertexIndices = cdt.triangles[i].vertices;
        resultTriangles.emplace_back(
            vertexIndices[0],
            vertexIndices[1],
            vertexIndices[2]
        );
    }

    return end - start;
}
