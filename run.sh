cmake_if=${1:-"false"}
stoprun=${2:-"false"}
xmlpath=${3:-""}


if [ $cmake_if = "r" ]; then
    rm -r build/
    mkdir build
fi

cd build

if [ "$cmake_if" = "r" ] || [ "$cmake_if" = "c" ]; then
    cmake ..
fi

make
cd ..

if [ "$stoprun" = "x" ]; then
catchsegv ./bin/LoboFEM -o hai $xmlpath
fi