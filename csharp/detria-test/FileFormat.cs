using Scalar = float;
using Idx = int;

using detria;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace detria_test;

public static class FileFormat
{
    public delegate bool FileReaderFunc(string path, out TriangulationInput result);

    private static string[] Tokenize(this string str) => str.Split(' ', System.StringSplitOptions.RemoveEmptyEntries | System.StringSplitOptions.TrimEntries);

    public static void ExportObj(IEnumerable<Vec2> points, IEnumerable<Triangle> triangles, string targetFile)
    {
        // Store the entire file in memory, then write it to disk
        // This uses more memory, but faster for large files

        StringBuilder sb = new StringBuilder();

        foreach (Vec2 p in points)
        {
            sb.Append("v ");
            sb.Append(p.x);
            sb.Append(' ');
            sb.Append(p.y);
            sb.Append(" 0");
            sb.AppendLine();
        }

        foreach (Triangle tri in triangles)
        {
            sb.Append("f ");
            sb.Append(tri.x + 1);
            sb.Append(' ');
            sb.Append(tri.y + 1);
            sb.Append(' ');
            sb.Append(tri.z + 1);
            sb.AppendLine();
        }

        Directory.CreateDirectory(Path.GetDirectoryName(targetFile));
        File.WriteAllText(targetFile, sb.ToString());
    }


    private enum ParserState
    {
        Polyline,
        Hole,
        Steiner
    }

    // For handling test files from poly2tri
    // From https://github.com/jhasse/poly2tri/blob/master/testbed/main.cc
    public static bool ParseFileP2T(string filePath, out List<Vec2> out_polyline, out List<List<Vec2>> out_holes, out List<Vec2> out_steiner)
    {
        ParserState state = ParserState.Polyline;
        List<Vec2> hole = null;

        out_polyline = [];
        out_holes = [];
        out_steiner = [];

        using FileStream fileStream = File.OpenRead(filePath);
        using StreamReader reader = new StreamReader(fileStream);

        while (true)
        {
            string line = reader.ReadLine();
            if (string.IsNullOrEmpty(line))
            {
                break;
            }

            string[] tokens = line.Tokenize();
            if (tokens.Length == 0)
            {
                break;
            }
            else if (tokens.Length == 1)
            {
                switch (tokens[0])
                {
                    case "HOLE":
                        state = ParserState.Hole;
                        hole = [];
                        out_holes.Add(hole);
                        break;

                    case "STEINER":
                        state = ParserState.Steiner;
                        break;

                    default:
                        return false;
                }
            }
            else
            {
                if (!Scalar.TryParse(tokens[0], out Scalar x) || !Scalar.TryParse(tokens[1], out Scalar y))
                {
                    return false;
                }

                Vec2 point = new Vec2(x, y);

                switch (state)
                {
                    case ParserState.Polyline:
                        out_polyline.Add(point);
                        break;

                    case ParserState.Hole:
                        if (hole == null)
                        {
                            return false;
                        }

                        hole.Add(point);
                        break;

                    case ParserState.Steiner:
                        out_steiner.Add(point);
                        break;
                }
            }
        }

        return true;
    }

    public static bool ReadFileP2T(string path, out TriangulationInput result)
    {
        result = new TriangulationInput();

        List<Vec2> allPoints = result.points;
        List<Polyline> resultPolylines = result.polylines;

        if (!ParseFileP2T(path, out List<Vec2> polyline, out List<List<Vec2>> holes, out List<Vec2> steiner))
        {
            return false;
        }

        void AddPolyline(List<Vec2> pl, bool isHole)
        {
            int numPoints = allPoints.Count;
            allPoints.AddRange(pl);

            Polyline currentPolyline = new Polyline
            {
                type = isHole ? PolylineType.Hole : PolylineType.Outline,
            };
            resultPolylines.Add(currentPolyline);

            List<Idx> indices = currentPolyline.pointIndices;
            indices.Capacity = pl.Count;
            for (int i = 0; i < pl.Count; ++i)
            {
                indices.Add((Idx)(numPoints + i));
            }
        }

        AddPolyline(polyline, false);

        foreach (List<Vec2> hole in holes)
        {
            AddPolyline(hole, true);
        }

        foreach (Vec2 steinerPoint in steiner)
        {
            allPoints.Add(steinerPoint);
        }

        return true;
    }

    // For handling test files
    public static bool ReadFile(string path, out TriangulationInput result)
    {
        result = new TriangulationInput();
        List<Vec2> allPoints = result.points;
        List<Polyline> resultPolylines = result.polylines;
        List<(Idx, Idx)> resultManuallyConstrainedEdges = result.manuallyConstrainedEdges;

        using FileStream fileStream = File.OpenRead(path);
        using StreamReader reader = new StreamReader(fileStream);

        // File structure:
        // numVertices numPolylines numManuallyConstrainedEdges
        // for numPolylines: numVerticesInPolyline
        // for numPolylines: polylineType (0 - outline, 1 - hole, 2 - auto detect)
        // list of vertices (x, y)
        // list of vertices in polylines (index)
        // list of manually constrained edges (index, index)

        string[] tokens = reader.ReadLine().Tokenize();

        int numVertices = int.Parse(tokens[0]);
        int numPolylines = int.Parse(tokens[1]);
        int numManuallyConstrainedEdges = int.Parse(tokens[2]);

        List<int> polylineLengths = [];
        List<PolylineType> polylineTypes = [];

        polylineLengths.Capacity = numPolylines;
        polylineTypes.Capacity = numPolylines;

        if (numPolylines > 0)
        {
            // Read polyline lengths
            tokens = reader.ReadLine().Tokenize();
            for (int i = 0; i < numPolylines; ++i)
            {
                polylineLengths.Add(int.Parse(tokens[i]));
            }

            // Read polyline types
            tokens = reader.ReadLine().Tokenize();
            for (int i = 0; i < numPolylines; ++i)
            {
                polylineTypes.Add((PolylineType)int.Parse(tokens[i]));
            }
        }

        // Read vertices
        allPoints.Capacity = numVertices;
        for (int i = 0; i < numVertices; ++i)
        {
            tokens = reader.ReadLine().Tokenize();
            Scalar x = Scalar.Parse(tokens[0]);
            Scalar y = Scalar.Parse(tokens[1]);
            allPoints.Add(new Vec2(x, y));
        }

        // Read polylines
        for (int i = 0; i < numPolylines; ++i)
        {
            int numVerticesInPolyline = polylineLengths[i];

            Polyline currentPolyline = new Polyline
            {
                type = polylineTypes[i],
            };
            resultPolylines.Add(currentPolyline);
            currentPolyline.pointIndices.Capacity = numVerticesInPolyline;

            for (int j = 0; j < numVerticesInPolyline; ++j)
            {
                currentPolyline.pointIndices.Add(Idx.Parse(reader.ReadLine()));
            }
        }

        // Read manually constrained edges
        resultManuallyConstrainedEdges.Capacity = numManuallyConstrainedEdges;
        for (int i = 0; i < numManuallyConstrainedEdges; ++i)
        {
            tokens = reader.ReadLine().Tokenize();
            Idx p0 = Idx.Parse(tokens[0]);
            Idx p1 = Idx.Parse(tokens[1]);
            resultManuallyConstrainedEdges.Add((p0, p1));
        }

        return true;
    }
}
