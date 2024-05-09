mkdir build-linux
cd build-linux

set -e

# clang++ debug
rm CMakeCache.txt
cmake .. -DCMAKE_BUILD_TYPE=Debug -DDETRIA_BUILD_BENCHMARKS=ON -DCMAKE_CXX_COMPILER=/usr/bin/clang++
make -j
make clean

# g++ debug
rm CMakeCache.txt
cmake .. -DCMAKE_BUILD_TYPE=Debug -DDETRIA_BUILD_BENCHMARKS=ON -DCMAKE_CXX_COMPILER=/usr/bin/g++
make -j
make clean

# g++ release
rm CMakeCache.txt
cmake .. -DCMAKE_BUILD_TYPE=Release -DDETRIA_BUILD_BENCHMARKS=ON -DCMAKE_CXX_COMPILER=/usr/bin/g++
make -j
make clean

# clang++ release
rm CMakeCache.txt
cmake .. -DCMAKE_BUILD_TYPE=Release -DDETRIA_BUILD_BENCHMARKS=ON -DCMAKE_CXX_COMPILER=/usr/bin/clang++
make -j

# run tests
cd ..
./bin/detria-test
cd build-linux

make clean

cd ..
