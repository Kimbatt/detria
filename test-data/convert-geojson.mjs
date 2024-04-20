// @ts-check

import * as fs from "fs";
import * as process from "process";
import * as path from "path";

const args = process.argv.slice(2);

if (args.length < 2)
{
    console.log("Not enough arguments");
    console.log("Usage: node convert-geojson.mjs <input.json> outfolder [--separate]");
    process.exit(1);
}

const inputFilePath = args[0];
const outFolder = args[1];

const separate = args.includes("--separate"); // Save each polygon separately

if (!fs.existsSync(inputFilePath) || !fs.lstatSync(inputFilePath).isFile())
{
    console.error("Input file doesn't exist");
    process.exit(1);
}

if (!fs.existsSync(outFolder) || !fs.lstatSync(outFolder).isDirectory())
{
    console.error("Output folder doesn't exist");
    process.exit(1);
}

const fileContents = fs.readFileSync(inputFilePath, { encoding: "utf8" });
const data = JSON.parse(fileContents);

const vertices = [];
const outlines = [];
const holes = [];

/**
 * @param {[number, number][]} polygon
 * @param {boolean} isHole
 */
function AddOutline(polygon, isHole)
{
    const polyline = [];
    for (let i = 0; i < polygon.length - 1; ++i) // First and last points are the same
    {
        const point = polygon[i];

        polyline.push(vertices.length.toString());
        vertices.push(point[0].toString() + " " + point[1].toString());
    }

    if (isHole)
    {
        holes.push(polyline);
    }
    else
    {
        outlines.push(polyline);
    }
}

/**
 * @param {string} fileName
 */
function WriteFile(fileName)
{
    const result = [];

    // Num vertices, num outlines, num manually constrained edges
    result.push(vertices.length.toString() + " " + (outlines.length + holes.length).toString() + " 0");

    const polylines = [...outlines, ...holes];

    // Write polyline lengths
    result.push(polylines.map(pl => pl.length).join(" "));

    // Write polyline types
    const polylineTypes = [];
    for (let i = 0; i < outlines.length; ++i)
    {
        polylineTypes.push("0");
    }
    for (let i = 0; i < holes.length; ++i)
    {
        polylineTypes.push("1");
    }

    result.push(polylineTypes.join(" "));

    for (const point of vertices)
    {
        result.push(point);
    }

    for (const polyline of polylines)
    {
        for (const idx of polyline)
        {
            result.push(idx);
        }
    }

    const resultFileContents = result.join("\n");
    fs.writeFileSync(path.join(outFolder, fileName), resultFileContents);
}

for (const feature of data.features)
{
    if (separate)
    {
        vertices.length = 0;
        outlines.length = 0;
        holes.length = 0;
    }

    const coordinates = feature.geometry.coordinates;

    if (feature.geometry.type === "LineString")
    {
        AddOutline(coordinates, false);
    }
    if (feature.geometry.type === "Polygon")
    {
        for (let i = 0; i < coordinates.length; ++i)
        {
            const polygon = coordinates[i];
            AddOutline(polygon, i !== 0);
        }
    }
    else if (feature.geometry.type === "MultiPolygon")
    {
        for (const polyArray of coordinates)
        {
            for (let i = 0; i < polyArray.length; ++i)
            {
                const polygon = polyArray[i];
                AddOutline(polygon, i !== 0);
            }
        }
    }

    if (separate)
    {
        WriteFile(feature.properties.GEOUNIT + ".txt");
    }
}

if (!separate)
{
    WriteFile(path.parse(inputFilePath).name + ".txt");
}
