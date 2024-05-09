#include "boxplot.h"

#include <array>
#include <cmath>
#include <numeric>
#include <optional>
#include <sstream>
#include <utility>

BoxPlotDatasetValues calculateDatasetValues(const std::string& name, std::span<const double> sortedSamples)
{
    BoxPlotDatasetValues result
    {
        .name = name,
        .supported = sortedSamples.size() >= 3
    };

    if (!result.supported)
    {
        return result;
    }

    result.minValue = sortedSamples.front();
    result.maxValue = sortedSamples.back();

    bool hasEvenNumberOfSamples = sortedSamples.size() % 2 == 0;
    auto getMedianAtIndex = [&](size_t index)
    {
        return hasEvenNumberOfSamples
            ? (sortedSamples[index - 1] + sortedSamples[index]) / 2.0
            : sortedSamples[index];
    };

    result.median = getMedianAtIndex(sortedSamples.size() / 2);
    result.lowerQuartile = getMedianAtIndex(sortedSamples.size() / 4);
    result.upperQuartile = getMedianAtIndex(sortedSamples.size() * 3 / 4);

    constexpr double iqrMultiplier = 1.5;
    double iqrRange = (result.upperQuartile - result.lowerQuartile) * iqrMultiplier;
    result.minValueWithoutOutliers = std::max(result.minValue, result.lowerQuartile - iqrRange);
    result.maxValueWithoutOutliers = std::min(result.maxValue, result.upperQuartile + iqrRange);

    for (double sample : sortedSamples)
    {
        if (sample < result.minValueWithoutOutliers || sample > result.maxValueWithoutOutliers)
        {
            result.outliers.push_back(sample);
        }
    }

    return result;
}

std::string generateBoxPlotSvg(const BoxPlotInput& input)
{
    constexpr int titleHeight = 100;
    constexpr int globalHorizontalPaddingLeft = 10;
    constexpr int globalHorizontalPaddingRight = 40;

    constexpr int rowHeight = 50;
    constexpr int rowWidth = 800;
    constexpr int rowSpacing = 20;
    constexpr int halfRowHeight = rowHeight / 2;
    constexpr int whiskerPadding = 10;

    constexpr int bottomValueNumbersHeight = 80;
    constexpr double bottomValueNumberCountTarget = 10.0;

    constexpr int boxPlotLineThickness = 2;

    constexpr double minMaxValuePaddingPercent = 0.02;

    int totalWidth = input.rowNameWidth + rowWidth + globalHorizontalPaddingLeft + globalHorizontalPaddingRight;

    int totalHeight =
        titleHeight +
        int(input.values.size()) * rowHeight +
        int(input.values.size()) * rowSpacing +
        bottomValueNumbersHeight;

    double globalMin = std::numeric_limits<double>::max();
    double globalMax = std::numeric_limits<double>::min();

    for (const auto& values : input.values)
    {
        if (!values.supported)
        {
            continue;
        }

        globalMin = std::min(globalMin, values.minValue);
        globalMax = std::max(globalMax, values.maxValue);
    }

    // Find best step size, to have around `bottomValueNumberCountTarget` steps,
    // and that the step size is a multiple of 10 or 100 or 1000 or so on
    double globalRange = globalMax - globalMin;

    // Add some padding at the sides
    globalMin -= globalRange * minMaxValuePaddingPercent;
    globalMax += globalRange * minMaxValuePaddingPercent;
    globalRange = globalMax - globalMin;

    double stepSize = globalRange / bottomValueNumberCountTarget;
    double magnitude = std::floor(std::log10(stepSize));
    double roundedToPowerOf10 = std::pow(10.0, magnitude);
    double finalStepSize = std::floor(stepSize / roundedToPowerOf10) * roundedToPowerOf10;
    int valuePrecision = std::max(input.valueDecimals - int(magnitude) - 2, 0);

    globalMin = std::floor(globalMin / finalStepSize) * finalStepSize;
    globalMax = std::ceil(globalMax / finalStepSize) * finalStepSize;
    int finalNumSteps = int(std::round((globalMax - globalMin) / finalStepSize));

    std::stringstream ss;
    ss << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << totalWidth << "\" height=\"" << totalHeight << "\">" << std::endl;

    // Setup non-value related stuff

    auto addAttribute = [&](const std::string& attributeName, const auto& value)
    {
        if (value.has_value())
        {
            ss << attributeName << "=\"" << *value << "\" ";
        }
    };

    std::vector<std::pair<std::string, std::string>> customAttributesForNextElement;

    auto addRect = [&](double x, double y, double width, double height, std::optional<std::string> fill = std::nullopt,
        std::optional<double> strokeWidth = std::nullopt, std::optional<std::string> stroke = std::nullopt)
    {
        ss << "<rect ";
        addAttribute("x", std::optional(x));
        addAttribute("y", std::optional(y));
        addAttribute("width", std::optional(width));
        addAttribute("height", std::optional(height));
        addAttribute("fill", fill);
        addAttribute("stroke", stroke);
        addAttribute("stroke-width", strokeWidth);

        for (const auto& attr : customAttributesForNextElement)
        {
            addAttribute(attr.first, std::optional(attr.second));
        }

        ss << "/>" << std::endl;
    };

    auto addLine = [&](double x1, double y1, double x2, double y2,
        std::optional<double> strokeWidth = std::nullopt, std::optional<std::string> stroke = "white")
    {
        ss << "<line ";
        addAttribute("x1", std::optional(x1));
        addAttribute("y1", std::optional(y1));
        addAttribute("x2", std::optional(x2));
        addAttribute("y2", std::optional(y2));
        addAttribute("stroke", stroke);
        addAttribute("stroke-width", strokeWidth);
        ss << "/>" << std::endl;
    };

    auto addText = [&](const std::string& text, double x, double y, std::optional<double> fontSize = std::nullopt,
        std::optional<std::string> textAnchor = std::nullopt, std::optional<std::string> dominantBaseline = std::nullopt,
        std::optional<std::string> fontStyle = std::nullopt)
    {
        ss << "<text ";
        addAttribute("x", std::optional(x));
        addAttribute("y", std::optional(y));
        addAttribute("font-size", fontSize);
        addAttribute("fill", std::optional("white"));
        addAttribute("text-anchor", textAnchor);
        addAttribute("dominant-baseline", dominantBaseline);
        addAttribute("font-style", fontStyle);
        ss << ">" << text << "</text>" << std::endl;
    };

    // Style + hover tooltips
    ss << R""(
<style>
text, .has-tooltip + *
{
    pointer-events: none;
}

.has-tooltip:not(:hover) + *
{
    display: none;
}
</style>
)"" << std::endl;

    // Background rect
    addRect(0, 0, totalWidth, totalHeight, "#121212");

    // Title
    addText(input.name, globalHorizontalPaddingLeft, 40, 30);

    // Description
    addText(input.description, globalHorizontalPaddingLeft, 80, 20);

    // Vertical line after the names
    addLine(
        globalHorizontalPaddingLeft + input.rowNameWidth + 0.5,
        titleHeight,
        globalHorizontalPaddingLeft + input.rowNameWidth + 0.5,
        totalHeight - bottomValueNumbersHeight,
        1
    );

    // Vertical line at the end
    addLine(
        totalWidth - globalHorizontalPaddingRight + 0.5,
        titleHeight,
        totalWidth - globalHorizontalPaddingRight + 0.5,
        totalHeight - bottomValueNumbersHeight,
        1
    );

    // Bottom horizontal line for the value numbers
    addLine(
        globalHorizontalPaddingLeft,
        totalHeight - bottomValueNumbersHeight - 0.5,
        totalWidth - globalHorizontalPaddingRight,
        totalHeight - bottomValueNumbersHeight - 0.5,
        1
    );

    double boxPlotMinX = globalHorizontalPaddingLeft + input.rowNameWidth;
    double boxPlotMaxX = boxPlotMinX + rowWidth;

    auto mapValueToDoubleSvgX = [&](double value)
    {
        double t = (value - globalMin) / (globalMax - globalMin);
        return (boxPlotMaxX - boxPlotMinX) * t;
    };
    auto mapValueToSvgX = [&](double value)
    {
        return int(std::round(mapValueToDoubleSvgX(value)));
    };

    // Bottom value numbers
    for (int i = 0; i <= finalNumSteps; ++i)
    {
        double t = double(i) / double(finalNumSteps);
        int x = int(std::round(boxPlotMinX + (boxPlotMaxX - boxPlotMinX) * t));
        int y = totalHeight - bottomValueNumbersHeight;

        addLine(
            x + 0.5,
            y,
            x + 0.5,
            y + 10,
            1
        );

        double currentValue = globalMin + finalStepSize * i;

        // Value
        std::stringstream valueSStream;
        valueSStream.precision(valuePrecision);
        valueSStream << std::fixed;
        valueSStream << currentValue;
        valueSStream << input.valueUnit;

        addText(
            valueSStream.str(),
            x + 0.5,
            y + 20,
            10,
            "middle"
        );
    }

    // Value description text
    addText(
        input.valueAxisName,
        std::midpoint(boxPlotMinX, boxPlotMaxX),
        totalHeight - bottomValueNumbersHeight + 55,
        16,
        "middle"
    );

    int startingRowY = titleHeight + rowSpacing / 2;
    int rowY = startingRowY;
    for (const auto& values : input.values)
    {
        // Row name
        addText(
            values.name,
            globalHorizontalPaddingLeft + 10,
            rowY + halfRowHeight,
            20,
            { },
            "middle"
        );

        // Separator horizontal line
        addLine(
            globalHorizontalPaddingLeft,
            rowY - rowSpacing / 2 - 0.5,
            totalWidth - globalHorizontalPaddingRight,
            rowY - rowSpacing / 2 - 0.5,
            1
        );

        ss << "<g transform=\"translate(" << boxPlotMinX << ", " << rowY << ")\">" << std::endl;

        if (values.supported)
        {
            int min = mapValueToSvgX(values.minValueWithoutOutliers);
            int q1 = mapValueToSvgX(values.lowerQuartile);
            int median = mapValueToSvgX(values.median);
            int q3 = mapValueToSvgX(values.upperQuartile);
            int max = mapValueToSvgX(values.maxValueWithoutOutliers);

            // Min value whisker
            addLine(
                min,
                whiskerPadding,
                min,
                rowHeight - whiskerPadding,
                boxPlotLineThickness
            );

            // Max value whisker
            addLine(
                max,
                whiskerPadding,
                max,
                rowHeight - whiskerPadding,
                boxPlotLineThickness
            );

            // Q1 to median box
            addRect(
                q1,
                0,
                median - q1,
                rowHeight,
                "#404040",
                boxPlotLineThickness,
                "white"
            );

            // Median to q3 box
            addRect(
                median,
                0,
                q3 - median,
                rowHeight,
                "#505050",
                boxPlotLineThickness,
                "white"
            );

            // Connecting line - min whisker to q1
            addLine(
                min,
                halfRowHeight,
                q1,
                halfRowHeight,
                boxPlotLineThickness
            );

            // Connecting line - q3 to max whisker
            addLine(
                q3,
                halfRowHeight,
                max,
                halfRowHeight,
                boxPlotLineThickness
            );

            // Outliers
            for (double outlier : values.outliers)
            {
                ss << "<circle cx=\"" << mapValueToDoubleSvgX(outlier)
                    << "\" cy=\"" << halfRowHeight
                    << "\" r=\"2.5\" fill=\"#ff000060\" />" << std::endl;
            }
        }
        else
        {
            addText(
                "Not supported",
                20,
                halfRowHeight,
                20,
                { },
                "middle",
                "italic"
            );
        }

        ss << "</g>" << std::endl;

        rowY += rowHeight + rowSpacing;
    }

    // Tooltips must be after all other elements because of the render order
    rowY = startingRowY;

    for (const auto& values : input.values)
    {
        if (values.supported)
        {
            // Tooltip hover trigger
            // Need to have opacity="0" instead of fill="none" for hover events to trigger
            customAttributesForNextElement = { { "class", "has-tooltip" }, { "opacity", "0" } };
            addRect(
                boxPlotMinX,
                rowY - rowSpacing / 2.0,
                boxPlotMaxX - boxPlotMinX,
                rowHeight + rowSpacing
            );

            ss << "<g transform=\"translate(" << boxPlotMinX + 20 << ", " << rowY - 75 << ")\">" << std::endl;

            constexpr int rowSpacing = 25;
            std::vector<std::pair<std::string, double>> valueRows;
            valueRows.reserve(8);

            if (values.minValue < values.minValueWithoutOutliers)
            {
                valueRows.emplace_back("Min (including outliers):", values.minValue);
            }

            valueRows.emplace_back("Min:", values.minValueWithoutOutliers);
            valueRows.emplace_back("Q1:", values.lowerQuartile);
            valueRows.emplace_back("Median:", values.median);
            valueRows.emplace_back("Q3:", values.upperQuartile);
            valueRows.emplace_back("Max:", values.maxValueWithoutOutliers);

            if (values.maxValue > values.maxValueWithoutOutliers)
            {
                valueRows.emplace_back("Max (including outliers):", values.maxValue);
            }

            customAttributesForNextElement = { { "rx", "5" } };
            addRect(
                0,
                0,
                360,
                rowSpacing * int(valueRows.size()) + 10,
                "black",
                2,
                "#a0a0a0"
            );

            auto addValueText = [&](double value, double y)
            {
                std::stringstream tmpValueSStream;
                tmpValueSStream.precision(input.valueDecimals);
                tmpValueSStream << std::fixed;
                tmpValueSStream << value << input.valueUnit;
                addText(tmpValueSStream.str(), 240, y);
            };

            int currentY = 25;
            for (const auto& [text, value] : valueRows)
            {
                addText(text, 10, currentY);
                addValueText(value, currentY);
                currentY += rowSpacing;
            }

            ss << "</g>" << std::endl;
        }

        rowY += rowHeight + rowSpacing;
    }

    ss << "</svg>" << std::endl;
    return ss.str();
}
