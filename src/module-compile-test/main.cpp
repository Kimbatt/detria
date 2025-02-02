#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>

// Test file to check if the module version compiles, and all exported items are available

import detria;

using Point = detria::PointD;
using Idx = uint32_t;

struct StdSorter
{
    template <typename Iter, typename Cmp>
    inline static void sort(Iter begin, Iter end, Cmp cmp)
    {
        return std::sort(begin, end, cmp);
    }
};

struct TestConfig : public detria::DefaultTriangulationConfig<Point, Idx>
{
    using Sorter = StdSorter;
};

int main()
{
    std::vector<Point> points
    {
        { 0.0, 0.0 },
        { 0.0, 1.0 },
        { 1.0, 1.0 },
        { 1.0, 0.0 },
    };

    std::vector<Idx> outline{ 0, 1, 2, 3 };

    detria::Triangulation<Point, Idx, TestConfig> tri;
    tri.setPoints(points);
    tri.addOutline(outline);

    bool success = tri.triangulate(true);

    if (success)
    {
        if (tri.getError() != detria::TriangulationError::NoError)
        {
            std::cerr << "Unexpected error" << std::endl;
            return 1;
        }

        std::cout << "Triangulation successful" << std::endl;

        tri.forEachTriangleOfLocation([&](const detria::Triangle<Idx>& triangle, detria::TriangleLocation /* location */)
        {
            std::cout << triangle.x << ' ' << triangle.y << ' ' << triangle.z << std::endl;
        }, detria::TriangleLocationMask::Interior);

        return 0;
    }
    else
    {
        std::cerr << "Triangulation failed" << std::endl;
        return 1;
    }
}
