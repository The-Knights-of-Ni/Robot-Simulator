mkdir "../osx_build"
pushd "../osx_build"

rm "simulator"
clang++ -Wall -Wno-missing-braces -Wno-unused-function -Wno-deprecated-writable-strings -O0 -o "simulator" -D DEBUG "../code/sdl_main.cpp" -I "../libraries/stb" -I "../libraries/SDL2-2.0.3/include" -F "./" -framework SDL2 -framework opengl -rpath "@loader_path/"

cd ../resources
"../osx_build/simulator"
popd
