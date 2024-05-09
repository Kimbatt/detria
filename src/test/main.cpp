#include <filesystem>
#include <iterator>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <utility>

#include <detria.hpp>

#include <fileformat.hpp>
#include <rng.hpp>
#include <misc.hpp>
#include <generate_inputs.hpp>

using namespace util;

template <typename Point, typename Idx>
bool Triangulate(const std::filesystem::path& filePath, const std::filesystem::path& exportFolder, bool expectedToFail,
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

    Clock::time_point start = Clock::now();
    bool triangulationSuccess = tri.triangulate(true);
    Clock::time_point end = Clock::now();

    bool success = true;
    if (triangulationSuccess == expectedToFail)
    {
        if (expectedToFail)
        {
            std::cout << "Triangulation succeeded, but it was expected to fail" << std::endl;
        }
        else
        {
            std::cout << "Triangulation failed, but it was expected to succeed" << std::endl;
        }

        success = false;
    }

    constexpr bool integerCoordinates = std::is_integral_v<decltype(Point::x)>;
    constexpr const char* maybeIntegerCoordinatesText = integerCoordinates ? " (integer coordinates)" : "";

    if (triangulationSuccess)
    {
        // Test export

        // If there are no polylines, then export the entire triangulation
        // Otherwise, only export the interior triangles
        detria::TriangleLocationMask exportLocation = polylines.empty() ? detria::TriangleLocationMask::All : detria::TriangleLocationMask::Interior;

        std::vector<detria::Triangle<Idx>> triangles;

        tri.forEachTriangleOfLocation([&](const detria::Triangle<Idx>& triangle, const detria::TriangleLocation& /* location */)
        {
            triangles.push_back(triangle);
        }, exportLocation, false);

        // Check triangle orientations - all triangles must be in a CCW orientation
        bool correctTriangleOrientation = true;
        tri.forEachTriangleOfEveryLocation([&](const detria::Triangle<Idx>& triangle)
        {
            if (detria::math::orient2d<true>(points[triangle.x], points[triangle.y], points[triangle.z]) != detria::math::Orientation::CCW)
            {
                correctTriangleOrientation = false;
            }
        }, false);

        if (!correctTriangleOrientation)
        {
            success = false;
            std::cout << "Error - triangulation has flipped triangles: " << fileName << std::endl;
        }
        else
        {
            // Check delaunay condition
            const auto& topology = tri.getTopology();
            using Topology = std::decay_t<decltype(topology)>;
            using TVertex = typename Topology::VertexIndex;
            using TEdge = typename Topology::HalfEdgeIndex;

            size_t vertexCount = topology.vertexCount();
            for (size_t i = 0; i < vertexCount; ++i)
            {
                topology.forEachEdgeOfVertex(TVertex(Idx(i)), [&](TEdge edgeIndex)
                {
                    TEdge e0 = edgeIndex;
                    TEdge e1 = topology.getEdge(topology.getOpposite(e0)).prevEdge;
                    TEdge e2 = topology.getEdge(topology.getOpposite(e1)).prevEdge;

                    if (topology.getEdgeData(e0).isBoundary())
                    {
                        return true;
                    }
                    if (topology.getEdgeData(e1).isBoundary())
                    {
                        return true;
                    }
                    if (topology.getEdgeData(e2).isBoundary())
                    {
                        return true;
                    }

                    TVertex v0 = topology.getEdge(e0).vertex;
                    TVertex v1 = topology.getEdge(e1).vertex;
                    TVertex v2 = topology.getEdge(e2).vertex;

                    Point p0 = points[size_t(v0.index)];
                    Point p1 = points[size_t(v1.index)];
                    Point p2 = points[size_t(v2.index)];

                    auto checkOppositeVertex = [&](TEdge e)
                    {
                        TEdge opposite = topology.getOpposite(e);
                        const auto& oppositeData = topology.getEdgeData(opposite);
                        if (oppositeData.isBoundary() || oppositeData.isConstrained())
                        {
                            return;
                        }

                        TVertex v3 = topology.getEdge(topology.getOpposite(topology.getEdge(e).prevEdge)).vertex;
                        Point p3 = points[size_t(v3.index)];

                        if (detria::math::incircle<true, Point>(p2, p1, p0, p3) == detria::math::CircleLocation::Inside)
                        {
                            success = false;
                            std::cout << "Error - non-delaunay triangle found: circumcircle of triangle { "
                                << v2.index << ", " << v1.index << ", " << v0.index
                                << " } contains point " << v3.index << std::endl;
                        }
                    };

                    checkOppositeVertex(e0);
                    checkOppositeVertex(e1);
                    checkOppositeVertex(e2);

                    return success;
                });

                if (!success)
                {
                    break;
                }
            }
        }

        if (!integerCoordinates)
        {
            std::filesystem::path objFileName = fileName;
            objFileName.replace_extension("obj");
            exportObj<Point, Idx, detria::Triangle<Idx>>(points, triangles, exportFolder / objFileName);
        }

        std::cout << "Triangulation successful" << maybeIntegerCoordinatesText << ": " << fileName << std::endl;
    }
    else
    {
        std::cerr << maybeIntegerCoordinatesText << "Triangulation failed" << maybeIntegerCoordinatesText << ": " << fileName << std::endl
            << "Message: " << tri.getErrorMessage() << std::endl;
    }

    std::cout << "Done in " << DurationToString(end - start) << std::endl;
    return success;
}

using Scalar = double;
using Point = detria::Vec2<Scalar>;
using Idx = uint32_t;

static bool RandomTest()
{
    // Generate some random points, and triangulate them

    std::vector<size_t> randomPointCounts = { 3, 10, 100, 1000, 100000 };

    detria::Triangulation<Point, Idx> tri;

    auto doRandomTest = [&](bool addConstrainedEdges, bool delaunay)
    {
        for (size_t pointCount : randomPointCounts)
        {
            tri.clear();

            std::vector<Point> points;
            std::vector<Idx> outline;

            generateRandomPoints(pointCount, points, outline);
            tri.setPoints(points);

            // If addConstrainedEdges is true, then we "enclose" the points in a box, and add the points as steiner points
            if (addConstrainedEdges)
            {
                tri.addOutline(outline);
            }

            std::cout << "Running random test with " << pointCount << " random points ("
                << (addConstrainedEdges ? "with constrained edges" : "no constrained edges")
                << ", "
                << (delaunay ? "delaunay" : "not delaunay")
                << ")" << std::endl;

            Clock::time_point start = Clock::now();
            bool success = tri.triangulate(delaunay);
            Clock::time_point end = Clock::now();

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

static bool FractalTest()
{
    std::vector<Point> points;
    std::vector<std::pair<std::vector<Idx>, bool>> polylines;

    fractal(6, 0.0, 0.0, 1.0, 0.125, points, polylines);

    detria::Triangulation<Point, Idx> tri;

    tri.setPoints(points);

    for (const auto& polyline : polylines)
    {
        if (polyline.second)
        {
            tri.addOutline(polyline.first);
        }
        else
        {
            tri.addHole(polyline.first);
        }
    }

    auto testConvexHullAndPolylineParent = [&]()
    {
        // Polyline 0 should be top-level, so no parent
        std::optional parentIdx = tri.getParentPolylineIndex(0);
        if (parentIdx.has_value())
        {
            return false;
        }

        std::vector<Idx> convexHullVertices;
        tri.forEachConvexHullVertex([&](Idx idx)
        {
            convexHullVertices.push_back(idx);
        });

        if (convexHullVertices.size() != 4)
        {
            return false;
        }

        // The convex hull should be "0 3 2 1" (or anything which has the same order, e.g. "2 1 0 3" etc.)
        size_t zeroIndex{ };
        for (size_t i = 0; i < convexHullVertices.size(); ++i)
        {
            if (convexHullVertices[i] == 0)
            {
                zeroIndex = i;
                break;
            }
        }

        bool ok =
            convexHullVertices[zeroIndex] == 0 &&
            convexHullVertices[(zeroIndex + 1) % 4] == 3 &&
            convexHullVertices[(zeroIndex + 2) % 4] == 2 &&
            convexHullVertices[(zeroIndex + 3) % 4] == 1;

        if (!ok)
        {
            return false;
        }

        return true;
    };

    Clock::time_point start = Clock::now();
    bool success = tri.triangulate(false);
    Clock::time_point end = Clock::now();

    std::cout << "Fractal test non delaunay done in " << DurationToString(end - start) << std::endl;

    if (!success)
    {
        std::cerr << tri.getErrorMessage() << std::endl;
        return false;
    }

    if (!testConvexHullAndPolylineParent())
    {
        return false;
    }

    start = Clock::now();
    success = tri.triangulate(true);
    end = Clock::now();

    std::cout << "Fractal test delaunay done in " << DurationToString(end - start) << std::endl;

    if (!success)
    {
        std::cerr << tri.getErrorMessage() << std::endl;
        return false;
    }

    if (!testConvexHullAndPolylineParent())
    {
        return false;
    }

    return true;
}

// Mostly just to check if it compiles
static bool testCustomPointGetter()
{
    std::vector<Point> points(100);

    constexpr Idx indexOffset = 20;
    constexpr size_t numPoints = 5;

    points[size_t(indexOffset + 0)] = Point{ 0.0, 0.0 };
    points[size_t(indexOffset + 1)] = Point{ 1.0, 0.0 };
    points[size_t(indexOffset + 2)] = Point{ 1.0, 1.0 };
    points[size_t(indexOffset + 3)] = Point{ 0.0, 1.0 };
    points[size_t(indexOffset + 4)] = Point{ 0.5, 0.5 };

    auto pointGetter = [&](Idx idx) -> const Point&
    {
        return points[size_t(idx + indexOffset)];
    };

    struct Config : detria::DefaultTriangulationConfig<Point, Idx>
    {
        using PointGetter = decltype(pointGetter);
    };

    detria::Triangulation<Point, Idx, Config> tri;
    tri.setPointGetter(pointGetter, numPoints);

    return tri.triangulate(true);
}

int main()
{
    std::cout << std::setprecision(20);

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
    std::vector<std::filesystem::path> p2tFilesThatCanBeUsedWithIntegerCoordinates =
    {
        "2.dat",
        "diamond.dat",
        "dude.dat",
        "e.dat",
        "funny.dat",
        "merge_test.dat",
        "merge_test_2.dat",
        "merge_test_3.dat",
        "stalactite.dat",
        "star.dat",
        "steiner.dat",
        "strange.dat",
        "tank.dat",
        "test.dat"
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

    auto testFile = [&](const std::filesystem::path& filePath, const std::filesystem::path& folder, const std::vector<std::filesystem::path>& filesExpectedToFail,
        const std::vector<std::filesystem::path>& filesThatCanBeUsedWithIntegerCoordinates,
        const std::vector<Point>& allPoints, const std::vector<Polyline<Idx>>& allPolylines, const std::vector<std::pair<Idx, Idx>>& allManuallyConstrainedEdges)
    {
        bool shouldFail = contains(filesExpectedToFail, filePath);
        bool success = Triangulate(filePath, testExportFolder / folder, shouldFail, allPoints, allPolylines, allManuallyConstrainedEdges);

        bool checkIntegerCoordinates = contains(filesThatCanBeUsedWithIntegerCoordinates, filePath);
        bool successWithIntegerCoordinates = true;
        if (checkIntegerCoordinates)
        {
            successWithIntegerCoordinates = Triangulate(filePath, testExportFolder / folder, shouldFail,
                getIntegerPoints<Point, detria::Vec2<int>>(allPoints), allPolylines, { });
        }

        if (!success || !successWithIntegerCoordinates)
        {
            failTest(filePath.string());
        }
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

        testFile(filePath, "p2t", p2tFilesExpectedToFail, p2tFilesThatCanBeUsedWithIntegerCoordinates, allPoints, allPolylines, { });
    }

    auto testFilesFromFolder = [&](const std::filesystem::path& folder,
        const std::vector<std::filesystem::path>& filesExpectedToFail,
        const std::vector<std::filesystem::path>& filesThatCanBeUsedWithIntegerCoordinates)
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
            if (!readFile<Point, Idx>(filePath, allPoints, allPolylines, allManuallyConstrainedEdges))
            {
                std::cerr << "Unable to read file: " << filePath << std::endl;
                continue;
            }

            testFile(filePath, folder, filesExpectedToFail, filesThatCanBeUsedWithIntegerCoordinates, allPoints, allPolylines, allManuallyConstrainedEdges);
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
    std::vector<std::filesystem::path> cdtFilesThatCanBeUsedWithIntegerCoordinates =
    {
        "cdt.txt",
        "gh_issue.txt",
        "Hanging.txt",
        "Hanging2.txt",
        "Letter u.txt",
        "ProblematicCase1.txt",
        "unit square.txt"
    };

    std::vector<std::filesystem::path> detriaFilesExpectedToFail =
    {
        "constrained edge intersection.txt",
        "duplicate point indices.txt",
        "invalid indices.txt"
    };
    std::vector<std::filesystem::path> detriaFilesThatCanBeUsedWithIntegerCoordinates =
    {
    };

    testFilesFromFolder("natural-earth-countries", { "Canada.txt" }, { });

    testFilesFromFolder("CDT", cdtFilesExpectedToFail, cdtFilesThatCanBeUsedWithIntegerCoordinates);
    testFilesFromFolder("detria", detriaFilesExpectedToFail, detriaFilesThatCanBeUsedWithIntegerCoordinates);


#endif

    if (!testCustomPointGetter())
    {
        failTest("Custom point getter");
    }

    if (!RandomTest())
    {
        failTest("Random test");
    }

    if (!FractalTest())
    {
        failTest("Fractal test");
    }

    constexpr char separator[] = "================================================================";

    int exitCode = 0;
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
