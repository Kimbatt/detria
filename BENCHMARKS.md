# Benchmarks
The benchmarks compare detria with the following libraries:
- [Triangle](https://www.cs.cmu.edu/~quake/triangle.html)
- [poly2tri](https://github.com/jhasse/poly2tri)
- [CDT](https://github.com/artem-ogre/CDT)

The code to run the benchmarks can be found [here](src/benchmark).

The benchmark only measures the triangulation time, not including the time to set up the inputs (since every triangulator expects input to be in a different format), and not including the time to retrieve the result triangles.  
For CDT, there is no single triangulation method, the calculations start immediately when the points are added, so it is measured accordingly.  
poly2tri doesn't support inputs with multiple outlines, so its runtime could not be measured for those cases.

The benchmarks were compiled with Clang 17.0.3 (with MSVC-like command-line), release build with default compiler flags, on Windows.  
The benchmarks were run on an AMD Ryzen 9 3900X CPU,  Windows 10.

The benchmark data includes:
- Outline of the earth and the "most complex" countries, data is from [Natural Earth](https://www.naturalearthdata.com/)
- Some files from poly2tri
- Fractal test: see [here](images/fractal.png), contains a lot of cocircular points
- Random test: generating uniform random points inside a square - the square has 4 corners and 4 outline edges, the rest of the points are steiner points


### ne_10m_land.txt - 407072 vertices, 398946 triangles (measured 10 times)

| triangulator | average time    | median time | shortest time | total time  |
|--------------|-----------------|-------------|---------------|-------------|
| detria       | 491.73 ms       | 490.61 ms   | 484.19 ms     | 4917.27 ms  |
| triangle     | 422.85 ms       | 421.64 ms   | 414.67 ms     | 4228.53 ms  |
| poly2tri     | *not supported* |             |               |             |
| cdt          | 1398.94 ms      | 1392.35 ms  | 1366.75 ms    | 13989.41 ms |

---


### United States of America.txt - 35708 vertices, 35016 triangles (measured 100 times)

| triangulator | average time    | median time | shortest time | total time |
|--------------|-----------------|-------------|---------------|------------|
| detria       | 26.11 ms        | 25.95 ms    | 24.63 ms      | 2610.53 ms |
| triangle     | 28.25 ms        | 27.28 ms    | 25.57 ms      | 2825.06 ms |
| poly2tri     | *not supported* |             |               |            |
| cdt          | 59.99 ms        | 59.31 ms    | 56.72 ms      | 5998.55 ms |

---


### Russia.txt - 36562 vertices, 36134 triangles (measured 100 times)

| triangulator | average time    | median time | shortest time | total time |
|--------------|-----------------|-------------|---------------|------------|
| detria       | 25.90 ms        | 25.67 ms    | 23.94 ms      | 2590.13 ms |
| triangle     | 27.33 ms        | 27.00 ms    | 25.57 ms      | 2733.08 ms |
| poly2tri     | *not supported* |             |               |            |
| cdt          | 64.54 ms        | 62.37 ms    | 58.08 ms      | 6453.69 ms |

---


### Antarctica.txt - 23657 vertices, 23299 triangles (measured 100 times)

| triangulator | average time    | median time | shortest time | total time |
|--------------|-----------------|-------------|---------------|------------|
| detria       | 14.35 ms        | 14.30 ms    | 13.82 ms      | 1435.37 ms |
| triangle     | 16.67 ms        | 16.46 ms    | 16.02 ms      | 1667.18 ms |
| poly2tri     | *not supported* |             |               |            |
| cdt          | 38.06 ms        | 37.70 ms    | 36.80 ms      | 3806.19 ms |

---


### Greenland.txt - 20007 vertices, 19751 triangles (measured 100 times)

| triangulator | average time    | median time | shortest time | total time |
|--------------|-----------------|-------------|---------------|------------|
| detria       | 13.64 ms        | 13.49 ms    | 13.16 ms      | 1363.98 ms |
| triangle     | 14.28 ms        | 14.21 ms    | 13.87 ms      | 1428.44 ms |
| poly2tri     | *not supported* |             |               |            |
| cdt          | 33.29 ms        | 33.03 ms    | 32.36 ms      | 3329.01 ms |

---


### Indonesia.txt - 19303 vertices, 18775 triangles (measured 100 times)

| triangulator | average time    | median time | shortest time | total time |
|--------------|-----------------|-------------|---------------|------------|
| detria       | 13.18 ms        | 13.08 ms    | 12.72 ms      | 1318.46 ms |
| triangle     | 13.37 ms        | 13.20 ms    | 12.86 ms      | 1336.97 ms |
| poly2tri     | *not supported* |             |               |            |
| cdt          | 29.71 ms        | 29.51 ms    | 28.84 ms      | 2971.30 ms |

---


### debug2.dat - 10000 vertices, 9998 triangles (measured 100 times)

| triangulator | average time | median time | shortest time | total time |
|--------------|--------------|-------------|---------------|------------|
| detria       | 12.24 ms     | 11.97 ms    | 11.70 ms      | 1224.22 ms |
| triangle     | 12.57 ms     | 12.46 ms    | 12.19 ms      | 1257.47 ms |
| poly2tri     | 15.59 ms     | 15.51 ms    | 15.02 ms      | 1559.11 ms |
| cdt          | 28.67 ms     | 28.47 ms    | 28.02 ms      | 2867.33 ms |

---


### city.dat - 3855 vertices, 3853 triangles (measured 200 times)

| triangulator | average time | median time | shortest time | total time |
|--------------|--------------|-------------|---------------|------------|
| detria       | 2.36 ms      | 2.35 ms     | 2.29 ms       | 471.69 ms  |
| triangle     | 2.51 ms      | 2.50 ms     | 2.42 ms       | 501.49 ms  |
| poly2tri     | 3.16 ms      | 3.15 ms     | 3.02 ms       | 631.65 ms  |
| cdt          | 6.09 ms      | 6.09 ms     | 5.91 ms       | 1217.69 ms |

---


### fractal, 2 depth - 84 vertices, 58 triangles (measured 5000 times)

| triangulator | average time    | median time | shortest time | total time |
|--------------|-----------------|-------------|---------------|------------|
| detria       | 0.06 ms         | 0.06 ms     | 0.05 ms       | 284.04 ms  |
| triangle     | 0.06 ms         | 0.06 ms     | 0.06 ms       | 318.09 ms  |
| poly2tri     | *not supported* |             |               |            |
| cdt          | 0.19 ms         | 0.19 ms     | 0.18 ms       | 974.18 ms  |

---


### fractal, 4 depth - 1364 vertices, 954 triangles (measured 500 times)

| triangulator | average time    | median time | shortest time | total time |
|--------------|-----------------|-------------|---------------|------------|
| detria       | 1.69 ms         | 1.69 ms     | 1.62 ms       | 844.18 ms  |
| triangle     | 1.71 ms         | 1.72 ms     | 1.64 ms       | 855.78 ms  |
| poly2tri     | *not supported* |             |               |            |
| cdt          | 3.42 ms         | 3.42 ms     | 3.30 ms       | 1707.98 ms |

---


### fractal, 6 depth - 21844 vertices, 15290 triangles (measured 50 times)

| triangulator | average time    | median time | shortest time | total time |
|--------------|-----------------|-------------|---------------|------------|
| detria       | 59.71 ms        | 59.67 ms    | 58.40 ms      | 2985.39 ms |
| triangle     | 28.06 ms        | 27.56 ms    | 26.91 ms      | 1403.12 ms |
| poly2tri     | *not supported* |             |               |            |
| cdt          | 55.04 ms        | 54.94 ms    | 53.60 ms      | 2752.20 ms |

---


### fractal, 8 depth - 349524 vertices, 244666 triangles (measured 5 times)

| triangulator | average time    | median time | shortest time | total time  |
|--------------|-----------------|-------------|---------------|-------------|
| detria       | 3112.93 ms      | 3115.03 ms  | 3080.27 ms    | 15564.63 ms |
| triangle     | 852.58 ms       | 857.15 ms   | 834.99 ms     | 4262.91 ms  |
| poly2tri     | *not supported* |             |               |             |
| cdt          | 1414.21 ms      | 1417.01 ms  | 1402.61 ms    | 7071.06 ms  |

---


### 100 random points - 104 vertices, 202 triangles (measured 10000 times)

| triangulator | average time | median time | shortest time | total time |
|--------------|--------------|-------------|---------------|------------|
| detria       | 0.03 ms      | 0.03 ms     | 0.03 ms       | 320.59 ms  |
| triangle     | 0.02 ms      | 0.02 ms     | 0.02 ms       | 187.96 ms  |
| poly2tri     | 0.08 ms      | 0.07 ms     | 0.07 ms       | 753.91 ms  |
| cdt          | 0.10 ms      | 0.10 ms     | 0.09 ms       | 991.42 ms  |

---


### 1000 random points - 1004 vertices, 2002 triangles (measured 500 times)

| triangulator | average time | median time | shortest time | total time |
|--------------|--------------|-------------|---------------|------------|
| detria       | 0.49 ms      | 0.49 ms     | 0.47 ms       | 247.35 ms  |
| triangle     | 0.43 ms      | 0.43 ms     | 0.41 ms       | 213.49 ms  |
| poly2tri     | 0.75 ms      | 0.75 ms     | 0.72 ms       | 374.70 ms  |
| cdt          | 1.11 ms      | 1.11 ms     | 1.07 ms       | 555.74 ms  |

---


### 10000 random points - 10004 vertices, 20002 triangles (measured 200 times)

| triangulator | average time | median time | shortest time | total time |
|--------------|--------------|-------------|---------------|------------|
| detria       | 6.86 ms      | 6.86 ms     | 6.56 ms       | 1372.93 ms |
| triangle     | 5.24 ms      | 5.16 ms     | 4.98 ms       | 1048.81 ms |
| poly2tri     | 7.77 ms      | 7.74 ms     | 7.50 ms       | 1554.81 ms |
| cdt          | 11.57 ms     | 11.52 ms    | 11.24 ms      | 2313.16 ms |

---


### 100000 random points - 100004 vertices, 200002 triangles (measured 10 times)

| triangulator | average time | median time | shortest time | total time |
|--------------|--------------|-------------|---------------|------------|
| detria       | 117.70 ms    | 117.85 ms   | 113.95 ms     | 1177.00 ms |
| triangle     | 67.00 ms     | 66.02 ms    | 64.98 ms      | 670.04 ms  |
| poly2tri     | 118.00 ms    | 116.54 ms   | 114.69 ms     | 1180.03 ms |
| cdt          | 170.07 ms    | 168.57 ms   | 160.89 ms     | 1700.72 ms |

---


### 1000000 random points - 1000004 vertices, 2000002 triangles (measured 2 times)

| triangulator | average time | median time | shortest time | total time |
|--------------|--------------|-------------|---------------|------------|
| detria       | 1833.96 ms   | 1833.96 ms  | 1828.57 ms    | 3667.92 ms |
| triangle     | 926.05 ms    | 926.05 ms   | 922.41 ms     | 1852.10 ms |
| poly2tri     | 2398.75 ms   | 2398.75 ms  | 2384.93 ms    | 4797.50 ms |
| cdt          | 2799.49 ms   | 2799.49 ms  | 2742.33 ms    | 5598.98 ms |

---
