#include <utility>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <sstream>

#include "triangulator_benchmark.h"

#include <fileformat.hpp>
#include <generate_inputs.hpp>

using namespace util;


static void processTriangulationResult(std::filesystem::path filename, const TriangulationInput& input, auto&& triangleGetter, size_t numTriangles)
{
    std::ofstream file("test-export" / filename, std::ios::binary);
    file.precision(20);

    for (const Vec2& p : input.points)
    {
        file << "v " << p.x << ' ' << p.y << " 0" << std::endl;
    }

    for (size_t i = 0; i < numTriangles; ++i)
    {
        Triangle tri = triangleGetter(i);
        file << "f " << (tri.x + 1) << ' ' << (tri.y + 1) << ' ' << (tri.z + 1) << std::endl;
    }
}

static void benchmarkRandomPoints(TriangulatorBenchmark& benchmark)
{
    // Generate random points within a square, and triangulate it

    struct RandomPointBenchmark
    {
        size_t numPoints;
        size_t numRuns;
    };

    std::vector<RandomPointBenchmark> numRandomPoints =
    {
        RandomPointBenchmark{ .numPoints = 1'000, .numRuns = 500 },
        RandomPointBenchmark{ .numPoints = 10'000, .numRuns = 200 },
        RandomPointBenchmark{ .numPoints = 100'000, .numRuns = 10 },
        RandomPointBenchmark{ .numPoints = 1'000'000, .numRuns = 3 }
    };

    for (const auto& data : numRandomPoints)
    {
        TriangulationInput input{ };
        generateRandomPoints(data.numPoints, input.points, input.outlines.emplace_back());

        std::stringstream benchmarkName;
        benchmarkName << data.numPoints << " random points";

        benchmark.runBenchmark(benchmarkName.str(), input, data.numRuns);
    }
}

static void benchmarkFractal(TriangulatorBenchmark& benchmark)
{
    struct FractalBenchmark
    {
        size_t depth;
        size_t numRuns;
    };

    std::vector<FractalBenchmark> benchmarkData =
    {
        FractalBenchmark{ .depth = 4, .numRuns = 500 },
        FractalBenchmark{ .depth = 6, .numRuns = 50 },
        FractalBenchmark{ .depth = 8, .numRuns = 5 }
    };

    for (const auto& data : benchmarkData)
    {
        TriangulationInput input{ };
        std::vector<std::pair<std::vector<Idx>, bool>> polylines;
        fractal(data.depth, 0.0, 0.0, 1.0, 0.125, input.points, polylines);

        for (auto& polyline : polylines)
        {
            if (polyline.second)
            {
                input.outlines.emplace_back(std::move(polyline.first));
            }
            else
            {
                input.holes.emplace_back(std::move(polyline.first));
            }
        }

        std::stringstream benchmarkName;
        benchmarkName << "fractal, " << data.depth << " depth";

        benchmark.runBenchmark(benchmarkName.str(), input, data.numRuns);
    }
}

static void benchmarkFile(TriangulatorBenchmark& benchmark, const std::filesystem::path& filePath, size_t numRuns,
    const std::vector<Vec2>& allPoints, const std::vector<Polyline<Idx>>& allPolylines)
{
    TriangulationInput input{ };
    input.points = allPoints;

    for (const auto& polyline : allPolylines)
    {
        if (polyline.type == PolylineType::Outline)
        {
            input.outlines.push_back(polyline.pointIndices);
        }
        else if (polyline.type == PolylineType::Hole)
        {
            input.holes.push_back(polyline.pointIndices);
        }
    }

    benchmark.runBenchmark(filePath.filename().string(), input, numRuns);
}

static void benchmarkFileP2TFormat(TriangulatorBenchmark& benchmark, std::filesystem::path filePath, size_t numRuns)
{
    std::vector<Vec2> allPoints;
    std::vector<Polyline<Idx>> allPolylines;
    if (!ReadFileP2T(filePath, allPoints, allPolylines))
    {
        std::cerr << "Unable to read file: " << filePath << std::endl;
        return;
    }

    benchmarkFile(benchmark, filePath, numRuns, allPoints, allPolylines);
}

static void benchmarkFileDetriaFormat(TriangulatorBenchmark& benchmark, std::filesystem::path filePath, size_t numRuns)
{
    std::vector<Vec2> allPoints;
    std::vector<Polyline<Idx>> allPolylines;
    std::vector<std::pair<Idx, Idx>> allManuallyConstrainedEdges;
    if (!readFile<Vec2, Idx>(filePath, allPoints, allPolylines, allManuallyConstrainedEdges))
    {
        std::cerr << "Unable to read file: " << filePath << std::endl;
        return;
    }

    benchmarkFile(benchmark, filePath, numRuns, allPoints, allPolylines); // Manually constrained edges are not always supported, so those are skipped
}

int main()
{
    TriangulatorBenchmark benchmark;

    benchmark.addTriangulator(std::make_unique<DetriaTriangulator>());
    benchmark.addTriangulator(std::make_unique<TriangleTriangulator>());
    benchmark.addTriangulator(std::make_unique<Poly2triTriangulator>());
    benchmark.addTriangulator(std::make_unique<CdtTriangulator>());

    std::filesystem::path testFolder = "test-data";

    benchmarkFileDetriaFormat(benchmark, testFolder / "detria" / "ne_10m_land.txt", 10);
    benchmarkFileDetriaFormat(benchmark, testFolder / "natural-earth-countries" / "United States of America.txt", 100);
    benchmarkFileDetriaFormat(benchmark, testFolder / "natural-earth-countries" / "Russia.txt", 100);
    benchmarkFileDetriaFormat(benchmark, testFolder / "natural-earth-countries" / "Antarctica.txt", 100);
    benchmarkFileDetriaFormat(benchmark, testFolder / "natural-earth-countries" / "Greenland.txt", 100);
    benchmarkFileDetriaFormat(benchmark, testFolder / "natural-earth-countries" / "Indonesia.txt", 100);
    benchmarkFileDetriaFormat(benchmark, testFolder / "natural-earth-countries" / "Afghanistan.txt", 500);
    benchmarkFileDetriaFormat(benchmark, testFolder / "natural-earth-countries" / "Bolivia.txt", 500);
    benchmarkFileDetriaFormat(benchmark, testFolder / "natural-earth-countries" / "Guinea.txt", 500);
    benchmarkFileDetriaFormat(benchmark, testFolder / "natural-earth-countries" / "Kyrgyzstan.txt", 500);
    benchmarkFileDetriaFormat(benchmark, testFolder / "natural-earth-countries" / "Laos.txt", 500);
    benchmarkFileDetriaFormat(benchmark, testFolder / "natural-earth-countries" / "Poland.txt", 500);
    benchmarkFileDetriaFormat(benchmark, testFolder / "natural-earth-countries" / "Republic of the Congo.txt", 500);
    benchmarkFileDetriaFormat(benchmark, testFolder / "natural-earth-countries" / "Zambia.txt", 500);
    benchmarkFileP2TFormat(benchmark, testFolder / "p2t" / "debug2.dat", 100);
    benchmarkFileP2TFormat(benchmark, testFolder / "p2t" / "city.dat", 200);

    benchmarkFractal(benchmark);
    benchmarkRandomPoints(benchmark);

    return 0;
}
