#include <filesystem>
#include <iterator>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <utility>

#include "../include/detria.hpp"


template <typename Point, typename Idx>
void exportObj(const std::vector<Point>& points, const std::vector<detria::Triangle<Idx>>& triangles, const std::filesystem::path& targetFile)
{
    std::ofstream file(targetFile, std::ios::binary);
    file << std::setprecision(20);

    for (const Point& p : points)
    {
        file << "v " << p.x << ' ' << p.y << " 0" << std::endl;
    }

    for (const detria::Triangle<Idx>& tri : triangles)
    {
        file << "f " << (tri.x + 1) << ' ' << (tri.z + 1) << ' ' << (tri.y + 1) << std::endl;
    }
}

template <typename Scalar>
bool StringToScalar(const std::string& str, Scalar& result)
{
    if constexpr (std::is_same_v<Scalar, float>)
    {
        try
        {
            result = std::stof(str);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }
    else if constexpr (std::is_same_v<Scalar, double>)
    {
        try
        {
            result = std::stod(str);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }
    else if constexpr (std::is_same_v<Scalar, long double>)
    {
        try
        {
            result = std::stold(str);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

template <typename Rep, typename Period>
std::string DurationToString(std::chrono::duration<Rep, Period> duration)
{
    double milliseconds = std::chrono::duration<double, std::milli>(duration).count();
    std::string str = std::to_string(milliseconds);
    str += " ms";
    return str;
}

// for handling test files from poly2tri
// from https://github.com/jhasse/poly2tri/blob/master/testbed/main.cc
template <typename Point>
bool ParseFileP2T(const std::filesystem::path& filePath, std::vector<Point>& out_polyline, std::vector<std::vector<Point>>& out_holes,
    std::vector<Point>& out_steiner)
{
    using Scalar = decltype(Point::x);

    enum class ParserState
    {
        Polyline,
        Hole,
        Steiner,
    };

    ParserState state = ParserState::Polyline;
    std::vector<Point>* hole = nullptr;

    std::string line;
    std::ifstream myfile(filePath);
    if (myfile.is_open())
    {
        while (!myfile.eof())
        {
            std::getline(myfile, line);
            if (line.empty())
            {
                break;
            }
            std::istringstream iss(line);
            std::vector<std::string> tokens;
            std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(tokens));
            if (tokens.empty())
            {
                break;
            }
            else if (tokens.size() == 1u)
            {
                const std::string& token = tokens[0];
                if (token == "HOLE")
                {
                    state = ParserState::Hole;
                    out_holes.emplace_back();
                    hole = &out_holes.back();
                }
                else if (token == "STEINER")
                {
                    state = ParserState::Steiner;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                Scalar x{ };
                Scalar y{ };
                if (!StringToScalar(tokens[0], x) || !StringToScalar(tokens[1], y))
                {
                    return false;
                }

                Point point{ .x = x, .y = y };

                switch (state)
                {
                    case ParserState::Polyline:
                        out_polyline.push_back(point);
                        break;
                    case ParserState::Hole:
                        if (hole == nullptr)
                        {
                            return false;
                        }

                        hole->push_back(point);
                        break;
                    case ParserState::Steiner:
                        out_steiner.push_back(point);
                        break;
                    default:
                        return false;
                }
            }
        }
    }
    else
    {
        return false;
    }

    return true;
}

enum class PolylineType
{
    Outline,
    Hole,
    AutoDetect
};

template <typename Idx>
struct Polyline
{
    std::vector<Idx> pointIndices;
    PolylineType type;
};

template <typename Point, typename Idx>
bool ReadFileP2T(const std::filesystem::path& path, std::vector<Point>& allPoints, std::vector<Polyline<Idx>>& resultPolylines)
{
    std::vector<Point> polyline;
    std::vector<std::vector<Point>> holes;
    std::vector<Point> steiner;
    if (ParseFileP2T(path, polyline, holes, steiner))
    {
        auto addPolyline = [&](const std::vector<Point>& pl, bool isHole)
        {
            size_t numPoints = allPoints.size();
            allPoints.insert(allPoints.end(), pl.begin(), pl.end());

            Polyline<Idx>& currentPolyline = resultPolylines.emplace_back(Polyline<Idx>{ });
            currentPolyline.type = isHole ? PolylineType::Hole : PolylineType::Outline;

            std::vector<Idx>& indices = currentPolyline.pointIndices;
            indices.reserve(pl.size());
            for (size_t i = 0; i < pl.size(); ++i)
            {
                indices.push_back(Idx(numPoints + i));
            }
        };

        addPolyline(polyline, false);

        for (const auto& hole : holes)
        {
            addPolyline(hole, true);
        }

        for (const auto& steinerPoint : steiner)
        {
            allPoints.push_back(steinerPoint);
        }

        return true;
    }
    else
    {
        return false;
    }
}

// for handling test files
template <typename Point, typename Idx>
bool readFile(const std::filesystem::path& path, std::vector<Point>& allPoints, std::vector<Polyline<Idx>>& resultPolylines,
    std::vector<std::pair<Idx, Idx>>& resultManuallyConstrainedEdges)
{
    using Scalar = decltype(Point::x);

    std::ifstream f(path);
    if (!f.is_open())
    {
        return false;
    }

    // file structure:
    // numVertices numPolylines numManuallyConstrainedEdges
    // for numPolylines: numVerticesInPolyline
    // list of vertices (x, y)
    // list of vertices in polylines (index)
    // list of manually constrained edges (index, index)

    try
    {
        size_t numVertices{ };
        size_t numPolylines{ };
        size_t numManuallyConstrainedEdges{ };
        f >> numVertices >> numPolylines >> numManuallyConstrainedEdges;

        std::vector<size_t> polylineLengths;
        polylineLengths.reserve(numPolylines);

        // read polyline lengths
        for (size_t i = 0; i < numPolylines; ++i)
        {
            size_t length{ };
            f >> length;
            polylineLengths.push_back(length);
        }

        // read vertices
        allPoints.reserve(numVertices);
        for (size_t i = 0; i < numVertices; ++i)
        {
            Scalar x{ };
            Scalar y{ };
            f >> x >> y;
            allPoints.emplace_back(Point{ .x = x, .y = y });
        }

        // read polylines
        for (size_t i = 0; i < numPolylines; ++i)
        {
            size_t numVerticesInPolyline = polylineLengths[i];
            Polyline<Idx>& currentPolyline = resultPolylines.emplace_back(Polyline<Idx>
            {
                .pointIndices = { },
                .type = PolylineType::AutoDetect
            });
            currentPolyline.pointIndices.reserve(numVerticesInPolyline);

            for (size_t j = 0; j < numVerticesInPolyline; ++j)
            {
                Idx idx{ };
                f >> idx;
                currentPolyline.pointIndices.push_back(idx);
            }
        }

        // read manually constrained edges
        resultManuallyConstrainedEdges.reserve(numManuallyConstrainedEdges);
        for (size_t i = 0; i < numManuallyConstrainedEdges; ++i)
        {
            Idx p0{ };
            Idx p1{ };
            f >> p0 >> p1;
            resultManuallyConstrainedEdges.emplace_back(p0, p1);
        }
    }
    catch (...)
    {
        return false;
    }

    return true;
}

template <typename Point, typename Idx>
bool Triangulate(const std::filesystem::path& filePath, const std::filesystem::path& exportFolder,
    const std::vector<Point>& points, const std::vector<Polyline<Idx>>& polylines, const std::vector<std::pair<Idx, Idx>>& manuallyConstrainedEdges)
{
    std::filesystem::path fileName = filePath.filename();

    detria::Triangulation<Point, Idx> tri;

    tri.setPoints(points);

    for (const Polyline<Idx>& polyline : polylines)
    {
        if (polyline.type == PolylineType::Outline)
        {
            tri.addOutline(polyline.pointIndices);
        }
        else if (polyline.type == PolylineType::Hole)
        {
            tri.addHole(polyline.pointIndices);
        }
        else if (polyline.type == PolylineType::AutoDetect)
        {
            tri.addPolylineAutoDetectType(polyline.pointIndices);
        }
    }

    for (const std::pair<Idx, Idx>& edge : manuallyConstrainedEdges)
    {
        tri.setConstrainedEdge(edge.first, edge.second);
    }

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    bool success = tri.triangulate(true);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    if (success)
    {
        // test export

        // if there are no polylines, then export the entire triangulation
        // otherwise, only export the interior triangles
        detria::TriangleLocation exportLocation = polylines.empty() ? detria::TriangleLocation::All : detria::TriangleLocation::Interior;

        std::vector<detria::Triangle<Idx>> triangles;
        tri.forEachTriangleOfLocation([&](const detria::Triangle<Idx>& triangle, const detria::TriangleLocation& /* location */)
        {
            triangles.push_back(triangle);
        }, exportLocation);

        exportObj(points, triangles, exportFolder / fileName.replace_extension("obj"));
        std::cout << "Triangulation successful: " << fileName << std::endl;
    }
    else
    {
        std::cerr << "Triangulation failed: " << fileName << std::endl << "Message: " << tri.getErrorMessage() << std::endl;
    }

    std::cout << "Done in " << DurationToString(end - start) << std::endl;
    return success;
}

// simple seeded random number generator
struct Mulberry32
{
    Mulberry32(uint32_t seed) : _state(seed)
    {
    }

    double operator()()
    {
        _state += 0x6D2B79F5;
        int32_t t = int32_t(_state);
        t = (t ^ (t >> 15)) * (t | 1);
        t = t ^ ((t ^ (t >> 7)) * (t | 61));
        t = t ^ (t >> 14);
        return double(uint32_t(t)) / 4294967296.0;
    }

private:
    uint32_t _state;
};

using Scalar = double;
using Point = detria::Vec2<Scalar>;
using Idx = uint32_t;

bool RandomTest()
{
    // generate some random points, and triangulate them

    std::vector<size_t> randomPointCounts = { 3, 10, 100, 1000, 100000 };
    constexpr Scalar minSize = Scalar(-10);
    constexpr Scalar maxSize = Scalar(10);
    constexpr Scalar padding = Scalar(1);

    Mulberry32 rng(0);

    auto randomMinMax = [&](Scalar min, Scalar max)
    {
        min += padding;
        max -= padding;
        Scalar randomValue = Scalar(rng());
        Scalar diff = max - min;
        return min + randomValue * diff;
    };

    detria::Triangulation<Point, Idx> tri;

    // first 4 point indices, 
    std::vector<Idx> constrainedEdgeIndices{ 0, 1, 2, 3 };

    auto doRandomTest = [&](bool addConstrainedEdges, bool delaunay)
    {
        for (size_t pointCount : randomPointCounts)
        {
            tri.clear();

            std::vector<Point> points;

            // if addConstrainedEdges is true, then we "enclose" the points in a box, and add the points as steiner points
            if (addConstrainedEdges)
            {
                points =
                {
                    { .x = minSize, .y = minSize },
                    { .x = maxSize, .y = minSize },
                    { .x = maxSize, .y = maxSize },
                    { .x = minSize, .y = maxSize },
                };

                tri.addOutline(constrainedEdgeIndices);
            }

            points.reserve(pointCount + 4);

            for (size_t i = 0; i < pointCount; ++i)
            {
                points.emplace_back(Point
                {
                    .x = randomMinMax(minSize, maxSize),
                    .y = randomMinMax(minSize, maxSize)
                });
            }

            tri.setPoints(points);

            std::cout << "Running random test with " << pointCount << " random points ("
                << (addConstrainedEdges ? "with constrained edges" : "no constrained edges")
                << ", "
                << (delaunay ? "delaunay" : "not delaunay")
                << ")" << std::endl;

            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            bool success = tri.triangulate(delaunay);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            std::cout << "Done in " << DurationToString(end - start) << std::endl;

            if (!success)
            {
                std::cerr << tri.getErrorMessage() << std::endl;
                return false;
            }
        }

        return true;
    };

    bool success =
        doRandomTest(false, false) &&
        doRandomTest(true, false) &&
        doRandomTest(false, true) &&
        doRandomTest(true, true);

    return success;
}

void fractal(int depth, float cx, float cy, float size, float thickness, std::vector<Point>& points, std::vector<Polyline<Idx>>& polylines)
{
    Idx startIndex = Idx(points.size());

    points.emplace_back(Point{ cx - size, cy - size });
    points.emplace_back(Point{ cx + size, cy - size });
    points.emplace_back(Point{ cx + size, cy + size });
    points.emplace_back(Point{ cx - size, cy + size });
    polylines.emplace_back(Polyline<Idx>
    {
        .pointIndices = { startIndex, startIndex + 1, startIndex + 2, startIndex + 3 },
        .type = PolylineType::AutoDetect
    });

#if false // can get a different fractal with this
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

    if (depth != 0)
    {
        constexpr float sizeFactor = 0.35f;
        constexpr float spacingFactor = 0.425f;

        float half = size * spacingFactor;
        fractal(depth - 1, cx - half, cy - half, size * sizeFactor, thickness * sizeFactor, points, polylines);
        fractal(depth - 1, cx + half, cy - half, size * sizeFactor, thickness * sizeFactor, points, polylines);
        fractal(depth - 1, cx + half, cy + half, size * sizeFactor, thickness * sizeFactor, points, polylines);
        fractal(depth - 1, cx - half, cy + half, size * sizeFactor, thickness * sizeFactor, points, polylines);
    }
}

bool FractalTest()
{
    std::vector<Point> points;
    std::vector<Polyline<Idx>> polylines;

    fractal(6, 0.0f, 0.0f, 1.0f, 0.125f, points, polylines);

    detria::Triangulation<Point, Idx> tri;

    tri.setPoints(points);

    for (const Polyline<Idx>& polyline : polylines)
    {
        tri.addPolylineAutoDetectType(polyline.pointIndices);
    }

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    bool success = tri.triangulate(false);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Fractal test non delaunay done in " << DurationToString(end - start) << std::endl;

    if (!success)
    {
        std::cerr << tri.getErrorMessage() << std::endl;
        return false;
    }

    start = std::chrono::steady_clock::now();
    success = tri.triangulate(true);
    end = std::chrono::steady_clock::now();

    std::cout << "Fractal test delaunay done in " << DurationToString(end - start) << std::endl;

    if (!success)
    {
        std::cerr << tri.getErrorMessage() << std::endl;
        return false;
    }

    return true;
}

int main()
{
    std::filesystem::path testFilesFolder = std::filesystem::path("test-data");
    std::filesystem::path testExportFolder = std::filesystem::path("test-export");

    std::vector<std::string> failedTestCases;
    auto failTest = [&](const std::string& name)
    {
        failedTestCases.push_back(name);
        std::cerr << "Test case failed: " << name << std::endl;
    };

#ifndef EMSCRIPTEN
    std::vector<std::filesystem::path> p2tFilesExpectedToFail =
    {
        "custom.dat",
        "collinear.dat",
        "sketchup.dat"
    };

    auto contains = [](const std::vector<std::filesystem::path>& files, const std::filesystem::path file)
    {
        std::filesystem::path fileName = file.filename();
        for (const std::filesystem::path& currentFile : files)
        {
            if (currentFile.filename() == fileName)
            {
                return true;
            }
        }

        return false;
    };

    for (const std::filesystem::directory_entry& dir : std::filesystem::directory_iterator(testFilesFolder / "p2t"))
    {
        if (dir.is_directory())
        {
            continue;
        }

        std::filesystem::path filePath = dir.path();

        if (filePath.extension() != ".dat")
        {
            continue;
        }

        std::vector<Point> allPoints;
        std::vector<Polyline<Idx>> allPolylines;
        if (!ReadFileP2T(filePath, allPoints, allPolylines))
        {
            std::cerr << "Unable to read file: " << filePath << std::endl;
            continue;
        }

        bool success = Triangulate(filePath, testExportFolder / "p2t", allPoints, allPolylines, { });
        bool shouldFail = contains(p2tFilesExpectedToFail, filePath);

        if (success == shouldFail)
        {
            failTest(filePath.string());
        }
    }

    auto testFilesFromFolder = [&](const std::filesystem::path& folder, const std::vector<std::filesystem::path>& filesExpectedToFail)
    {
        for (const std::filesystem::directory_entry& dir : std::filesystem::directory_iterator(testFilesFolder / folder))
        {
            if (dir.is_directory())
            {
                continue;
            }

            std::filesystem::path filePath = dir.path();

            if (filePath.extension() != ".txt")
            {
                continue;
            }

            std::vector<Point> allPoints;
            std::vector<Polyline<Idx>> allPolylines;
            std::vector<std::pair<Idx, Idx>> allManuallyConstrainedEdges;
            if (!readFile(filePath, allPoints, allPolylines, allManuallyConstrainedEdges))
            {
                std::cerr << "Unable to read file: " << filePath << std::endl;
                continue;
            }

            bool success = Triangulate(filePath, testExportFolder / folder, allPoints, allPolylines, allManuallyConstrainedEdges);
            bool shouldFail = contains(filesExpectedToFail, filePath);

            if (success == shouldFail)
            {
                failTest(filePath.string());
            }
        }
    };

    std::vector<std::filesystem::path> cdtFilesExpectedToFail =
    {
        "corner cases.txt",
        "crossing-edges.txt",
        "issue-42-hole-overlaps-bondary.txt",
        "issue-42-multiple-boundary-overlaps-conform-to-edge.txt",
        "issue-42-multiple-boundary-overlaps.txt",
        "overlapping constraints.txt",
        "overlapping constraints2.txt",
        "points_on_constraint_edge.txt"
    };

    std::vector<std::filesystem::path> detriaFilesExpectedToFail =
    {
        "constrained edge intersection.txt",
        "duplicate point indices.txt",
        "invalid indices.txt"
    };

    testFilesFromFolder("CDT", cdtFilesExpectedToFail);
    testFilesFromFolder("detria", detriaFilesExpectedToFail);

#endif

    int exitCode = 0;

    if (!RandomTest())
    {
        failTest("Random test");
    }

    if (!FractalTest())
    {
        failTest("Fractal test");
    }

    constexpr char separator[] = "================================================================";

    if (failedTestCases.empty())
    {
        std::cout << std::endl
            << separator << std::endl
            << "All tests successful" << std::endl
            << separator << std::endl;
    }
    else
    {
        std::cerr << std::endl
            << separator << std::endl
            << "The following tests failed:" << std::endl << std::endl;

        for (const std::string& name : failedTestCases)
        {
            std::cerr << name << std::endl;
        }

        std::cerr << separator << std::endl;

        exitCode = 1;
    }

    return exitCode;
}
