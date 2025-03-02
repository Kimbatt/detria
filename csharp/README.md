Similar to the C++ version, the C# version is also a single file, which can be found here: [`detria.cs`](detria/detria.cs)

The C# version works with C# 7 or newer.  
Unity engine is also supported, Unity 2018 or later is required.  
In the Unity version, the built-in `UnityEngine.Vector2` type is used (instead of `detria.Vec2`)

The usage is similar to the C++ version:
```csharp
#if UNITY_5_3_OR_NEWER
using Vec2 = UnityEngine.Vector2;
#else
using Vec2 = detria.Vec2;
#endif

// Create a square, and triangulate it

// List of points (positions)
var points = new Vec2[]
{
    new Vec2(0.0f, 0.0f),
    new Vec2(1.0f, 0.0f),
    new Vec2(1.0f, 1.0f),
    new Vec2(0.0f, 1.0f),
};

// List of point indices
var outline = new int[] { 0, 1, 2, 3 };

bool delaunay = true;

var tri = new detria.Triangulation();
tri.SetPoints(points);
tri.AddOutline(outline);

bool success = tri.Triangulate(delaunay);

if (success)
{
    bool cwTriangles = true;

    foreach (detria.Triangle triangle in tri.EnumerateTriangles(cwTriangles))
    {
        // `triangle` contains the point indices

        var firstPointOfTriangle = points[triangle.x];
        var secondPointOfTriangle = points[triangle.y];
        var thirdPointOfTriangle = points[triangle.z];

        // Use the results
        Console.WriteLine($"Triangle: ({firstPointOfTriangle}), ({secondPointOfTriangle}), ({thirdPointOfTriangle})");
    }
}
```

Main differences:
- Less configuration options:
    - No custom point types  
    Since C# generics work differently to C++ templates, we can't use the same style of configurations for specifying a custom point type.  
    At the start of [`detria.cs`](detria/detria.cs), there are some pre-defined types for `Scalar` and `Idx` (index) types. These can be edited manually to support custom point and index types.  
    You can also add a define for `DETRIA_DOUBLE_PRECISION` to use `double` values by default (instead of `float`).  
    Because of this, integer point types are not supported (though you can still use integer values with float types).
    - No allocators  
    In C#, there is no need (and there is no way) to specify custom allocators for managed memory.
    - No custom sorter  
    Using a custom sorter seems to have worse performance.
- There are no memory safety concerns - since C# uses a garbage collector, the caller cannot free the memory while it's still being referenced by the triangulation.
- The result triangles (and other stuff) are retrieved through an `IEnumerable`, instead of a callback function. (So instead of `forEachXXX`, you need to write `EnumerateXXX`)
- The names of the functions and classes start with a capital letter.

The rest is pretty much same as in the [documentation of the C++ version](../DOCS.md).

All public methods and properties in the `detria.Triangulation` class are documented in the source code. These can be found in the `Public API` region.
