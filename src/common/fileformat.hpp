#pragma once

#include <vector>
#include <string>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iterator>

namespace util
{
    template <typename Point, typename Idx, typename Tri>
    void exportObj(const std::vector<Point>& points, const std::vector<Tri>& triangles, const std::filesystem::path& targetFile)
    {
        // Store the entire file in memory, then write it to disk
        // This uses more memory, but faster for large files

        std::stringstream ss;
        ss.precision(20);

        for (const Point& p : points)
        {
            ss << "v " << p.x << ' ' << p.y << " 0" << std::endl;
        }

        for (const Tri& tri : triangles)
        {
            ss << "f " << (tri.x + 1) << ' ' << (tri.y + 1) << ' ' << (tri.z + 1) << std::endl;
        }

        std::ofstream file(targetFile, std::ios::binary);
        file << ss.str();
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
        else if constexpr (std::is_integral_v<Scalar>)
        {
            try
            {
                result = Scalar(std::stoll(str));
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

    // For handling test files from poly2tri
    // From https://github.com/jhasse/poly2tri/blob/master/testbed/main.cc
    template <typename Point>
    static bool ParseFileP2T(const std::filesystem::path& filePath, std::vector<Point>& out_polyline, std::vector<std::vector<Point>>& out_holes,
        std::vector<Point>& out_steiner)
    {
        using Scalar = decltype(Point::x);

        enum class ParserState
        {
            Polyline,
            Hole,
            Steiner
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

                    Point point{ x, y };

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
    static bool ReadFileP2T(const std::filesystem::path& path, std::vector<Point>& allPoints, std::vector<Polyline<Idx>>& resultPolylines)
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

    // For handling test files
    template <typename Point, typename Idx>
    static bool readFile(const std::filesystem::path& path, std::vector<Point>& allPoints, std::vector<Polyline<Idx>>& resultPolylines,
        std::vector<std::pair<Idx, Idx>>& resultManuallyConstrainedEdges)
    {
        using Scalar = decltype(Point::x);

        std::ifstream f(path);
        if (!f.is_open())
        {
            return false;
        }

        // File structure:
        // numVertices numPolylines numManuallyConstrainedEdges
        // for numPolylines: numVerticesInPolyline
        // for numPolylines: polylineType (0 - outline, 1 - hole, 2 - auto detect)
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
            std::vector<PolylineType> polylineTypes;

            polylineLengths.reserve(numPolylines);
            polylineTypes.reserve(numPolylines);

            // Read polyline lengths
            for (size_t i = 0; i < numPolylines; ++i)
            {
                size_t length{ };
                f >> length;
                polylineLengths.push_back(length);
            }

            // Read polyline types
            for (size_t i = 0; i < numPolylines; ++i)
            {
                size_t type{ };
                f >> type;
                polylineTypes.push_back(PolylineType(type));
            }

            // Read vertices
            allPoints.reserve(numVertices);
            for (size_t i = 0; i < numVertices; ++i)
            {
                Scalar x{ };
                Scalar y{ };
                f >> x >> y;
                allPoints.emplace_back(Point{ x, y });
            }

            // Read polylines
            for (size_t i = 0; i < numPolylines; ++i)
            {
                size_t numVerticesInPolyline = polylineLengths[i];
                Polyline<Idx>& currentPolyline = resultPolylines.emplace_back(Polyline<Idx>
                {
                    std::vector<Idx>{ }, polylineTypes[i]
                });
                currentPolyline.pointIndices.reserve(numVerticesInPolyline);

                for (size_t j = 0; j < numVerticesInPolyline; ++j)
                {
                    Idx idx{ };
                    f >> idx;
                    currentPolyline.pointIndices.push_back(idx);
                }
            }

            // Read manually constrained edges
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

    template <typename Point, typename IntPoint>
    static std::vector<IntPoint> getIntegerPoints(const std::vector<Point>& points)
    {
        std::vector<IntPoint> resultPoints;
        resultPoints.reserve(points.size());

        for (const Point& p : points)
        {
            resultPoints.emplace_back(IntPoint{ int(p.x), int(p.y) });
        }

        return resultPoints;
    }
}
