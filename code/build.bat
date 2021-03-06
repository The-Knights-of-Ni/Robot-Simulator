@echo off

mkdir "../win32_build"
pushd "../win32_build"

del "simulator.exe"
clang++ -Wall -Wno-missing-braces -Wno-unused-function -O0 -o "simulator" -D DEBUG "../code/sdl_main.cpp" -I "../libraries/stb" -I "../libraries/SDL2-2.0.3/include" -L "../libraries/SDL2-2.0.3/i686-w64-mingw32/lib" -lopengl32 -lmingw32 -lSDL2main -lSDL2

cd ../resources
"../win32_build/simulator"
popd
