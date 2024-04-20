#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <memory>

#include <misc.hpp>

using Scalar = double;
using Idx = uint32_t;

struct Vec2
{
    Vec2() = default;
    Vec2(Scalar x_, Scalar y_) : x(x_), y(y_)
    {
    }

    Scalar x;
    Scalar y;
};

static_assert(sizeof(Vec2) == sizeof(Scalar) * 2);

struct Triangle
{
    Triangle() = default;
    Triangle(Idx x_, Idx y_, Idx z_) : x(x_), y(y_), z(z_)
    {
    }

    Idx x;
    Idx y;
    Idx z;
};

struct TriangulationInput
{
    std::vector<Vec2> points;
    std::vector<std::vector<Idx>> outlines;
    std::vector<std::vector<Idx>> holes;
};

enum class TriangulatorCapabilities : uint8_t
{
    MultipleOutlines = 1 << 0,
    AutoDetectPolylineType = 1 << 1,
};

class Triangulator
{
public:
    virtual std::string name() const = 0;
    virtual TriangulatorCapabilities getCapabilities() const = 0;
    virtual util::Clock::duration triangulateAndMeasure(const TriangulationInput& input, std::vector<Triangle>& resultTriangles) const = 0;

    virtual ~Triangulator() { }
};

class TriangulatorBenchmark
{
public:
    void addTriangulator(std::unique_ptr<Triangulator>&& triangulator)
    {
        _triangulators.emplace_back(std::move(triangulator));
    }

    void runBenchmark(std::string name, const TriangulationInput& input, size_t runCount);

private:
    std::vector<std::unique_ptr<Triangulator>> _triangulators;
};

class DetriaTriangulator : public Triangulator
{
public:
    virtual std::string name() const override;
    virtual TriangulatorCapabilities getCapabilities() const override;
    virtual util::Clock::duration triangulateAndMeasure(const TriangulationInput& input, std::vector<Triangle>& resultTriangles) const override;
};

class TriangleTriangulator : public Triangulator
{
public:
    virtual std::string name() const override;
    virtual TriangulatorCapabilities getCapabilities() const override;
    virtual util::Clock::duration triangulateAndMeasure(const TriangulationInput& input, std::vector<Triangle>& resultTriangles) const override;
};

class Poly2triTriangulator : public Triangulator
{
public:
    virtual std::string name() const override;
    virtual TriangulatorCapabilities getCapabilities() const override;
    virtual util::Clock::duration triangulateAndMeasure(const TriangulationInput& input, std::vector<Triangle>& resultTriangles) const override;
};

class CdtTriangulator : public Triangulator
{
public:
    virtual std::string name() const override;
    virtual TriangulatorCapabilities getCapabilities() const override;
    virtual util::Clock::duration triangulateAndMeasure(const TriangulationInput& input, std::vector<Triangle>& resultTriangles) const override;
};
