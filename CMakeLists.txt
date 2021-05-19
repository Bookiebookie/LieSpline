cmake_minimum_required(VERSION 3.10)
include("thirdparty/basalt/thirdparty/basalt-headers/cmake_modules/PreProjectWorkarounds.cmake")

project(spline-experiments)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

set(CERES_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build-ceres-solver")

# Set default build type if not specified otherwise.
# See https://cmake.org/pipermail/cmake/2012-May/050243.html
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  message(STATUS "Setting build type to '${CMAKE_BUILD_TYPE}' as none was specified.")
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

# Configure CCache if available
if (NOT CMAKE_C_COMPILER_LAUNCHER AND NOT CMAKE_CXX_COMPILER_LAUNCHER)
  find_program(CCACHE_PROGRAM ccache)
  if(CCACHE_PROGRAM)
    message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  else()
    message(STATUS "Dind't find ccache")
  endif()
else()
  message(STATUS "Compiler launcher already set. Not configuring ccache.")
  message(STATUS "CMAKE_C_COMPILER_LAUNCHER: ${CMAKE_C_COMPILER_LAUNCHER}")
  message(STATUS "CMAKE_CXX_COMPILER_LAUNCHER: ${CMAKE_CXX_COMPILER_LAUNCHER}")
endif()

# See comment in ./build_submodules.sh on the relation of arch=native
# and Eigen's alignment. TLDR: Passing arch=native for all build types
# is currently the only viable option to avoid suble bugs with Eigen.
SET(CXX_MARCH native)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# for some reason clang doesn't include them in Wall whereas gcc does, so include them explicitly.
SET(EXTRA_WARNING_FLAGS "-Wsign-compare")

# clang-specific compile flags to avoid warnings from thirdpary includes
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
  set(EXTRA_WARNING_FLAGS "${EXTRA_WARNING_FLAGS} -Wno-exceptions -frelaxed-template-template-args")
endif()

set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DBASALT_USE_CHOLMOD")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DBASALT_USE_CHOLMOD -DBASALT_DISABLE_ASSERTS -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DBASALT_USE_CHOLMOD -DBASALT_DISABLE_ASSERTS -DNDEBUG")
SET(CMAKE_CXX_FLAGS " -ftemplate-backtrace-limit=0 -Wall ${EXTRA_WARNING_FLAGS} -march=${CXX_MARCH} ${CMAKE_CXX_FLAGS}")

# Set platform / compiler specific compile flags and checks
if(APPLE)
  # Among others, setting CMAKE_FIND_FRAMEWORK to LAST fixed issues
  # with installed Mono that contains old headers (libpng, ...).
  # See: https://github.com/openMVG/openMVG/issues/1349#issuecomment-401492811
  set(CMAKE_FIND_FRAMEWORK LAST)

  if(CMAKE_SYSTEM_VERSION VERSION_LESS 19.0.0)
      # use brewed llvm's libc++
      include_directories("/usr/local/opt/llvm/include/c++/v1")
      link_directories("/usr/local/opt/llvm/lib")
      add_compile_options("-nostdinc++")
      set(STD_CXX_FS c++fs)

      # Workaround for cmake not to filter the manually added standard include path
      # See: https://gitlab.kitware.com/cmake/cmake/issues/19227#note_669894
      list(REMOVE_ITEM CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES "/usr/local/opt/llvm/include/c++/v1")
  endif()


  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(STATUS "Detected macOS with non-Apple clang")

  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    message(STATUS "Detected macOS with Apple clang")
    # Apple clang on macOS < 10.14 Mojave is too old
    if(CMAKE_SYSTEM_VERSION VERSION_LESS 18.0.0)
      message(WARNING "Detected Darwin version ${CMAKE_SYSTEM_VERSION}, which is earlier than macos 10.14 Mojave. Apple clang is too old and not supported. Use clang from homebrew.")
    endif()

  else()
    message(WARNING "Detected macOS with unsupported compiler ${CMAKE_CXX_COMPILER_ID}")
  endif()

elseif(UNIX)
  set(USE_OPENMP_DEFAULT OFF)

  # assume libstdc++
  set(STD_CXX_FS stdc++fs)

  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(LINUX_CLANG 1)
    message(STATUS "Detected Linux with clang.")

  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(STATUS "Detected Linux with gcc.")

  else()
    message(WARNING "Detected Linux with unsupported compiler ${CMAKE_CXX_COMPILER_ID}")
  endif()

else()
  message(WARNING "Only Linux and macOS are currently supported")
endif()

find_package(Eigen3 3.3.9 EXACT REQUIRED CONFIG)
message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIRS}")

find_package(TBB REQUIRED)
message(STATUS "TBB_INTERFACE_VERSION: ${TBB_INTERFACE_VERSION}")
if (TBB_INTERFACE_VERSION LESS 11004)
  # enable global_control header for earlier TBB versions (Ubuntu 16.04, 18.04)
  add_definitions(-DTBB_PREVIEW_GLOBAL_CONTROL)
endif()

find_package(Ceres 2.0 REQUIRED CONFIG HINTS "${CERES_ROOT}" NO_DEFAULT_PATH)

find_package(SuiteSparse REQUIRED)
message(STATUS "Found suitesparse headers in: ${SUITESPARSE_INCLUDE_DIRS}")

find_package(OpenCV REQUIRED core imgproc calib3d highgui)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "Found OpenCV headers in: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Found OpenCV_LIBS: ${OpenCV_LIBS}")


include_directories(thirdparty/basalt/thirdparty/basalt-headers/thirdparty/Sophus)
include_directories(thirdparty/basalt/thirdparty/basalt-headers/thirdparty/cereal/include)
include_directories(thirdparty/basalt/thirdparty/basalt-headers/include)
include_directories(thirdparty/basalt/thirdparty/CLI11/include)


include_directories(thirdparty/basalt/include)
include_directories(include)



add_executable(eval_lie_spline src/eval_lie_spline.cpp)
target_link_libraries(eval_lie_spline Eigen3::Eigen Ceres::ceres)

add_executable(eval_calib src/eval_calib.cpp thirdparty/basalt/src/calibration/aprilgrid.cpp)
target_link_libraries(eval_calib ${OpenCV_LIBS} ${STD_CXX_FS} Eigen3::Eigen Ceres::ceres)

# Note: Add suitesparse after ceres, since on macos it sets include directories /usr/local/include,
# such that the wrong ceres version is picked up, if installed from homebrew.
target_link_libraries(eval_calib SuiteSparse::SuiteSparse)

enable_testing()
include_directories(${EIGEN3_INCLUDE_DIRS})  # Needed for basalt-headers tests
add_subdirectory(thirdparty/basalt/thirdparty/basalt-headers/test)
add_subdirectory(test)
