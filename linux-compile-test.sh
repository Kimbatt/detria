mkdir build-linux
cd build-linux

set -e

# clang++ debug
rm -f CMakeCache.txt
cmake .. -DCMAKE_BUILD_TYPE=Debug -DDETRIA_BUILD_BENCHMARKS=ON -DDETRIA_BUILD_CXXMODULE=ON -DCMAKE_CXX_COMPILER=/usr/bin/clang++ -G Ninja
ninja
ninja clean

# g++ debug
rm -f CMakeCache.txt
cmake .. -DCMAKE_BUILD_TYPE=Debug -DDETRIA_BUILD_BENCHMARKS=ON -DCMAKE_CXX_COMPILER=/usr/bin/g++ -G Ninja
ninja
ninja clean

# g++ release
rm -f CMakeCache.txt
cmake .. -DCMAKE_BUILD_TYPE=Release -DDETRIA_BUILD_BENCHMARKS=ON -DCMAKE_CXX_COMPILER=/usr/bin/g++ -G Ninja
ninja
ninja clean

# clang++ release
rm -f CMakeCache.txt
cmake .. -DCMAKE_BUILD_TYPE=Release -DDETRIA_BUILD_BENCHMARKS=ON -DDETRIA_BUILD_CXXMODULE=ON -DCMAKE_CXX_COMPILER=/usr/bin/clang++ -G Ninja
ninja

# run tests
cd ..
./bin/detria-test
cd build-linux

ninja clean

cd ..
