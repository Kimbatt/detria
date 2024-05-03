#include <sstream>
#include <iostream>
#include <algorithm>

#include "triangulator_benchmark.h"
#include "boxplot.h"

#include <fileformat.hpp>

using namespace util;

void TriangulatorBenchmark::runBenchmark(std::string name, const TriangulationInput& input, size_t runCount)
{
    struct BenchmarkRun
    {
        struct Result
        {
            std::string triangulatorName;
            std::vector<util::Clock::duration> times;
            bool supported;
        };

        std::string name;
        std::vector<Result> results;
        size_t vertexCount;
        size_t triangleCount;
    };

    BenchmarkRun run{ };
    run.name = name;
    run.vertexCount = input.points.size();

    if (runCount < 1)
    {
        runCount = 1;
    }

    std::filesystem::path exportFolder = "test-export";
    exportFolder /= "benchmark";

    bool hasMultipleOutlines = input.outlines.size() > 1;

    for (const auto& triangulator : _triangulators)
    {
        BenchmarkRun::Result& result = run.results.emplace_back();
        result.triangulatorName = triangulator->name();

        TriangulatorCapabilities capabilities = triangulator->getCapabilities();

        auto isSupported = [&](TriangulatorCapabilities capability)
        {
            return (static_cast<uint8_t>(capabilities) & static_cast<uint8_t>(capability)) != 0;
        };

        bool multipleOutlinesSupported = isSupported(TriangulatorCapabilities::MultipleOutlines);

        if (hasMultipleOutlines && !multipleOutlinesSupported)
        {
            result.supported = false;
            continue;
        }

        result.supported = true;

        std::vector<Triangle> resultTriangles;
        for (size_t i = 0; i < runCount; ++i)
        {
            resultTriangles.clear();
            resultTriangles.shrink_to_fit();

            Clock::duration time = triangulator->triangulateAndMeasure(input, resultTriangles);

            result.times.push_back(time);
        }

        run.triangleCount = resultTriangles.size();

        std::string resultFileName;
        resultFileName.append(name);
        resultFileName.append(" - ");
        resultFileName.append(triangulator->name());
        resultFileName.append(".obj");
        exportObj<Vec2, Idx, Triangle>(input.points, resultTriangles, exportFolder / resultFileName);
    }

    // Generate box plot image
    BoxPlotInput boxPlotInput{ };
    {
        std::filesystem::path imagesFolder = "images";
        imagesFolder /= "benchmark";

        std::vector<BoxPlotDatasetValues> boxPlotValues;
        boxPlotValues.reserve(_triangulators.size());

        std::vector<double> sampleTimes;
        for (const auto& runResult : run.results)
        {
            sampleTimes.clear();
            sampleTimes.reserve(runResult.times.size());

            for (const auto& time : runResult.times)
            {
                double timeInMilliseconds = std::chrono::duration<double, std::milli>(time).count();
                sampleTimes.push_back(timeInMilliseconds);
            }

            std::sort(sampleTimes.begin(), sampleTimes.end());
            boxPlotValues.emplace_back(calculateDatasetValues(runResult.triangulatorName, sampleTimes));
        }

        std::stringstream ss;
        ss << run.vertexCount << " vertices, " << run.triangleCount << " triangles"
            << " (measured " << runCount << " times)";

        boxPlotInput = BoxPlotInput
        {
            .name = run.name,
            .description = ss.str(),
            .valueAxisName = "Triangulation time (milliseconds)",
            .values = std::move(boxPlotValues),
            .valueDecimals = 2,
            .valueUnit = "ms",
            .rowNameWidth = 120
        };

        std::string svg = generateBoxPlotSvg(boxPlotInput);

        std::string svgFileName = name;
        svgFileName.append(".svg");
        std::ofstream svgFile(imagesFolder / svgFileName);
        svgFile << svg;
    }

    // Print results
    {
        constexpr size_t triangulatorNameColumn = 0;
        constexpr size_t minTimeColumn = 1;
        constexpr size_t medianTimeColumn = 2;
        constexpr size_t maxTimeColumn = 3;
        constexpr size_t numColumns = 4;
        std::vector<std::vector<std::string>> columns(numColumns);

        columns[triangulatorNameColumn].emplace_back("triangulator");
        columns[minTimeColumn].emplace_back("shortest time");
        columns[medianTimeColumn].emplace_back("median time");
        columns[maxTimeColumn].emplace_back("longest time");

        for (auto& values : boxPlotInput.values)
        {
            columns[triangulatorNameColumn].emplace_back(values.name);

            if (!values.supported)
            {
                columns[minTimeColumn].emplace_back("*not supported*");
                columns[medianTimeColumn].emplace_back();
                columns[maxTimeColumn].emplace_back();
            }
            else
            {
                columns[minTimeColumn].emplace_back(ToStringWithPrecision(values.minValue, 2));
                columns[medianTimeColumn].emplace_back(ToStringWithPrecision(values.median, 2));
                columns[maxTimeColumn].emplace_back(ToStringWithPrecision(values.maxValue, 2));
            }
        }

        std::vector<size_t> columnSizes;
        columnSizes.reserve(columns.size());

        for (const auto& column : columns)
        {
            size_t maxLength = 0;
            for (const auto& text : column)
            {
                maxLength = std::max(maxLength, text.length());
            }

            columnSizes.push_back(maxLength);
        }

        std::stringstream ss;
        ss.precision(20);

        ss << "### " << run.name << " - "
            << run.vertexCount << " vertices, " << run.triangleCount << " triangles"
            << " (measured " << runCount << " times)" << std::endl << std::endl;

        // Write table header
        ss << "|";
        for (size_t i = 0; i < columns.size(); ++i)
        {
            const auto& text = columns[i][0];
            size_t columnSize = columnSizes[i];

            ss << ' ' << text;
            for (size_t j = text.length(); j < columnSize; ++j)
            {
                ss << ' ';
            }

            ss << " |";
        }
        ss << std::endl;

        // Write separator row
        ss << "|";
        for (size_t i = 0; i < columns.size(); ++i)
        {
            size_t columnSize = columnSizes[i];

            // Write table header
            for (size_t j = 0; j < columnSize + 2; ++j) // Add extra characters at start and end
            {
                ss << '-';
            }

            ss << '|';
        }
        ss << std::endl;

        // Write values/rows
        for (size_t i = 0; i < run.results.size(); ++i)
        {
            size_t rowIndex = i + 1; // Row 0 is the header

            ss << "|";
            for (size_t j = 0; j < columns.size(); ++j)
            {
                const auto& text = columns[j][rowIndex];
                size_t columnSize = columnSizes[j];

                ss << ' ' << text;
                for (size_t k = text.length(); k < columnSize; ++k)
                {
                    ss << ' ';
                }

                ss << " |";
            }
            ss << std::endl;
        }
        ss << std::endl;
        ss << "---" << std::endl;

        std::cout << ss.str() << std::endl << std::endl;
    }
}
