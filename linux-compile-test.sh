mkdir build-linux
cd build-linux

# g++ debug
rm CMakeCache.txt
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=/usr/bin/g++
make
make clean

# g++ release
rm CMakeCache.txt
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/g++
make
make clean

# clang++ debug
rm CMakeCache.txt
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=/usr/bin/clang++
make
make clean

# clang++ release
rm CMakeCache.txt
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/clang++
make

# run tests
cd ..
./bin/detria-test
cd build-linux

make clean

cd ..
