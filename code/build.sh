mkdir "../osx_build"
pushd "../osx_build"

rm "simulator"
clang++ -Wall -Wno-missing-braces -O0 -o "simulator" -D DEBUG "../code/sdl_main.cpp" -I "../libraries/SDL2-2.0.3/include" -F "./" -framework SDL2 -rpath "@loader_path/"

"./simulator"
popd
