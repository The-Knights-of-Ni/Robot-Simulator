mkdir "../osx_build"
pushd "../osx_build"

rm "simulator.exe"
clang++ -Wall -O0 -o "simulator" -D DEBUG "../code/sdl_main.cpp" -I "../libraries/SDL2-2.0.3/include" -L "../libraries/SDL2-2.0.3/i686-w64-mingw32/lib" -lmingw32 -lSDL2main -lSDL2

"./simulator"
popd
