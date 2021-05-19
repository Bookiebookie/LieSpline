#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

set -x
set -e

BUILD_TYPE=Release

if [ -n "$1" ]; then
BUILD_TYPE=$1
fi

# https://stackoverflow.com/a/45181694
NUM_CORES=`getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu || echo 1`

NUM_PARALLEL_BUILDS=$NUM_CORES

# Important note on Eigen alignment and the arch flag. TLDR: Passing
# arch=native for all build types is currently the only viable option
# to avoid suble bugs with Eigen.
#
# Eigen uses 16 byte alignemnt by default, but switches to 32 byte
# alignment if AVX instructions are enabled. This is the case on
# modern Intel hardware if we pass arch=native. It is vital to ensure
# that all translation units, including all thirdparty libraries, use
# the same value for EIGEN_MAX_ALIGN_BYTES (see
# https://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html),
# since the aliged-malloc functions provided by Eigen might not be
# inlined and thus be taken from any of the translation units. Our
# current approach is to ensure arch=native everywhere. A possible
# alternative would be to explicitly pass -DEIGEN_MAX_ALIGN_BYTES=32
# everywhere, then 32 bytes would be used regardless of whether avx is
# enabled or not.
#
# Note that Ceres might override with arch=native in Release mode no
# matter what we pass it, so any other value for CXX_MARCH
# might not work as expected. Also, even though we explicitly pass
# -O3, it might be overwritten e.g. with -O2 if we don't also set the
# build type to Release.

CXX_MARCH=native

COMMON_CMAKE_ARGS=(
    -DCMAKE_C_COMPILER_LAUNCHER=ccache
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
    -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
    -DCMAKE_CXX_FLAGS="-march=$CXX_MARCH -O3 -Wno-deprecated-declarations -Wno-null-pointer-arithmetic -Wno-unknown-warning-option -Wno-unused-function" #  -Wno-int-in-bool-context
)

BUILD_CERES="$SCRIPT_DIR/thirdparty/build-ceres-solver"

git submodule sync --recursive
git submodule update --init --recursive

rm -rf "$BUILD_CERES"

mkdir -p "$BUILD_CERES"
pushd "$BUILD_CERES"
cmake ../ceres-solver "${COMMON_CMAKE_ARGS[@]}" \
    -DCMAKE_PREFIX_PATH="$SCRIPT_DIR/cmake_modules/eigen3" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF \
    -DEXPORT_BUILD_DIR=ON \
    -DCERES_THREADING_MODEL=CXX_THREADS

make -j$NUM_PARALLEL_BUILDS ceres
popd

