// These types must be the same as in detria.cs
// But since it's not possible with C# to directly use those types, they must be copied here, and all files that use them
using Scalar = float;
using Idx = int;

using detria;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.IO;
using System;

using static detria_test.FileFormat;

namespace detria_test;

public enum PolylineType
{
    Outline,
    Hole,
    AutoDetect
}

public class Polyline
{
    public List<Idx> pointIndices = [];
    public PolylineType type;
}

public class TriangulationInput
{
    public readonly List<Vec2> points = [];
    public readonly List<Polyline> polylines = [];
    public readonly List<(Idx, Idx)> manuallyConstrainedEdges = [];

    public TriangulationInput()
    {
    }

    public TriangulationInput(List<Vec2> points, List<Polyline> polylines, List<(Idx, Idx)> manuallyConstrainedEdges)
    {
        this.points = points;
        this.polylines = polylines;
        this.manuallyConstrainedEdges = manuallyConstrainedEdges;
    }
}

[TestClass]
public sealed class Test
{
    public static void SetupTriangulation(Triangulation triangulation, TriangulationInput input)
    {
        triangulation.Clear();

        triangulation.SetPoints(input.points);

        foreach (Polyline polyline in input.polylines)
        {
            switch (polyline.type)
            {
                case PolylineType.Outline:
                    triangulation.AddOutline(polyline.pointIndices);
                    break;
                case PolylineType.Hole:
                    triangulation.AddHole(polyline.pointIndices);
                    break;
                case PolylineType.AutoDetect:
                    triangulation.AddPolylineAutoDetectType(polyline.pointIndices);
                    break;
                default:
                    Assert.Fail("Unknown polyline type: " + polyline.type);
                    break;
            }
        }

        foreach ((Idx x, Idx y) in input.manuallyConstrainedEdges)
        {
            triangulation.SetConstrainedEdge(x, y);
        }
    }

    [TestMethod]
    public void SimpleTest()
    {
        Triangulation tri = new Triangulation();

        tri.SetPoints([
            new Vec2(0.0f, 0.0f),
            new Vec2(1.0f, 0.0f),
            new Vec2(1.0f, 1.0f),
            new Vec2(0.0f, 1.0f),
        ]);

        bool success = tri.Triangulate(true);
        Assert.IsTrue(success);

        int numTriangles = tri.EnumerateAllTriangles(true).Count();
        Assert.AreEqual(2, numTriangles);
    }

    private static void TestSingleFile(Triangulation triangulation, TriangulationInput input, string name, bool shouldFail)
    {
        SetupTriangulation(triangulation, input);

        Stopwatch sw = Stopwatch.StartNew();
        bool triangulationSuccess = triangulation.Triangulate(true);
        sw.Stop();

        Assert.AreNotEqual(shouldFail, triangulationSuccess, name);

        if (triangulationSuccess)
        {
            Console.WriteLine($"Triangulation success: {name}");
            ExportObj(input.points, triangulation.EnumerateTriangles(false), Path.Combine("test-export", name + ".obj"));
        }
        else
        {
            Console.WriteLine($"Triangulation failed (was expected to fail): {name}");
        }

        Console.WriteLine($"Finished in {sw.Elapsed.TotalMilliseconds} milliseconds");
        Console.WriteLine();

        // Test triangle orientations
        string orientationTestErrorMessage = $"Triangulation has flipped triangles: {name}";
        foreach (Triangle tri in triangulation.EnumerateAllTriangles(false))
        {
            Vec2 a = input.points[(int)tri.x];
            Vec2 b = input.points[(int)tri.y];
            Vec2 c = input.points[(int)tri.z];

            detria.Math.Orientation orientation = detria.Math.Orient2d(triangulation.Predicates, a, b, c);
            Assert.AreEqual(detria.Math.Orientation.CCW, orientation, orientationTestErrorMessage);
        }

        // Test delaunay condition
        {
            Topology topology = triangulation.Topology;
            int vertexCount = topology.VertexCount;
            for (int i = 0; i < vertexCount; ++i)
            {
                topology.ForEachEdgeOfVertex(new VertexIndex((Idx)i), edgeIndex =>
                {
                    HalfEdgeIndex e0 = edgeIndex;
                    HalfEdgeIndex e1 = topology.GetEdge(Topology.GetOpposite(e0)).prevEdge;
                    HalfEdgeIndex e2 = topology.GetEdge(Topology.GetOpposite(e1)).prevEdge;

                    if (topology.GetEdgeData(e0).IsBoundary)
                    {
                        return true;
                    }
                    if (topology.GetEdgeData(e1).IsBoundary)
                    {
                        return true;
                    }
                    if (topology.GetEdgeData(e2).IsBoundary)
                    {
                        return true;
                    }

                    VertexIndex v0 = topology.GetEdge(e0).vertex;
                    VertexIndex v1 = topology.GetEdge(e1).vertex;
                    VertexIndex v2 = topology.GetEdge(e2).vertex;

                    Vec2 p0 = input.points[(int)v0.index.Value];
                    Vec2 p1 = input.points[(int)v1.index.Value];
                    Vec2 p2 = input.points[(int)v2.index.Value];

                    void CheckOppositeVertex(HalfEdgeIndex e)
                    {
                        HalfEdgeIndex opposite = Topology.GetOpposite(e);
                        EdgeData oppositeData = topology.GetEdgeData(opposite);
                        if (oppositeData.IsBoundary || oppositeData.IsConstrained)
                        {
                            return;
                        }

                        VertexIndex v3 = topology.GetEdge(Topology.GetOpposite(topology.GetEdge(e).prevEdge)).vertex;
                        Vec2 p3 = input.points[(int)v3.index.Value];

                        detria.Math.CircleLocation incircleResult = detria.Math.Incircle(triangulation.Predicates, p2, p1, p0, p3);
                        Assert.AreNotEqual(detria.Math.CircleLocation.Inside, incircleResult, "Non-delaunay triangle found");
                    }

                    CheckOppositeVertex(e0);
                    CheckOppositeVertex(e1);
                    CheckOppositeVertex(e2);

                    return true;
                });
            }
        }
    }

    private static void TestAllFilesInFolder(string folder, bool isP2TFormat, IEnumerable<string> filesExpectedToFail)
    {
        DirectoryInfo directoryInfo = new DirectoryInfo(folder);

        string requiredExtension = isP2TFormat ? ".dat" : ".txt";

        Triangulation triangulation = new Triangulation();

        foreach (FileInfo fileInfo in directoryInfo.EnumerateFiles())
        {
            if (!fileInfo.Extension.Equals(requiredExtension, StringComparison.OrdinalIgnoreCase))
            {
                continue;
            }

            FileReaderFunc readerFunc = isP2TFormat ? ReadFileP2T : FileFormat.ReadFile;
            Assert.IsTrue(readerFunc(fileInfo.FullName, out TriangulationInput input));

            TestSingleFile(triangulation, input, fileInfo.Name, filesExpectedToFail.Contains(fileInfo.Name, StringComparer.OrdinalIgnoreCase));
        }
    }

    [TestMethod]
    public void TestFilesP2T()
    {
        List<string> filesExpectedToFail = [
            "custom.dat",
            "collinear.dat",
            "sketchup.dat",
        ];

        TestAllFilesInFolder(Path.Combine("test-data", "p2t"), true, filesExpectedToFail);
    }

    [TestMethod]
    public void TestFilesCDT()
    {
        List<string> filesExpectedToFail = [
            "corner cases.txt",
            "crossing-edges.txt",
            "issue-42-hole-overlaps-bondary.txt",
            "issue-42-multiple-boundary-overlaps-conform-to-edge.txt",
            "issue-42-multiple-boundary-overlaps.txt",
            "overlapping constraints.txt",
            "overlapping constraints2.txt",
            "points_on_constraint_edge.txt",
        ];

        TestAllFilesInFolder(Path.Combine("test-data", "CDT"), false, filesExpectedToFail);
    }

    [TestMethod]
    public void TestFilesDetria()
    {
        List<string> filesExpectedToFail = [
            "constrained edge intersection.txt",
            "duplicate point indices.txt",
            "long collinear.txt",
            "invalid indices.txt",
        ];

        if (typeof(Scalar) == typeof(float))
        {
            // When using 32-bit floats, there is not enough precision to distinguish some of the points in the following files
            // The precision loss causes different points to become equal, so expect them to fail in the tests

            filesExpectedToFail.Add("ne_10m_land.txt");
        }

        TestAllFilesInFolder(Path.Combine("test-data", "detria"), false, filesExpectedToFail);
    }

    [TestMethod]
    public void TestFilesNaturalEarth()
    {
        List<string> filesExpectedToFail = ["Canada.txt"];

        if (typeof(Scalar) == typeof(float))
        {
            // Same as in TestFilesDetria, expect these files to fail in 32-bit float mode

            filesExpectedToFail.AddRange([
                "Antarctica.txt",
                "Belize.txt",
                "Bosnia and Herzegovina.txt",
                "Cuba.txt",
                "Cyprus.txt",
                "Lebanon.txt",
                "North Korea.txt",
                "Saint Martin.txt",
                "Turkey.txt",
                "United States of America.txt",
            ]);
        }

        TestAllFilesInFolder(Path.Combine("test-data", "natural-earth-countries"), false, filesExpectedToFail);
    }
}
