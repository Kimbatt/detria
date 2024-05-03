#pragma once

#include <span>
#include <string>
#include <vector>

struct BoxPlotDatasetValues
{
    std::string name;
    bool supported;
    double minValue;
    double lowerQuartile;
    double median;
    double upperQuartile;
    double maxValue;
    double minValueWithoutOutliers;
    double maxValueWithoutOutliers;
    std::vector<double> outliers;
};

struct BoxPlotInput
{
    std::string name;
    std::string description;
    std::string valueAxisName;
    std::vector<BoxPlotDatasetValues> values;

    int valueDecimals = 2;
    std::string valueUnit = "";
    int rowNameWidth = 200;
};

BoxPlotDatasetValues calculateDatasetValues(const std::string& name, std::span<const double> sortedSamples);
std::string generateBoxPlotSvg(const BoxPlotInput& input);
