cmake_minimum_required(VERSION 3.10...3.18)

project(multitargetcalib)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})

if(NOT EIGEN_ROOT)
  set(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/multitargetcalib-headers/thirdparty/eigen")
endif()

find_program(LSB_RELEASE_PROGRAM lsb_release DOC "lsb_release program of Debian-based systems")

if(LSB_RELEASE_PROGRAM)
  execute_process(COMMAND ${LSB_RELEASE_PROGRAM} -rs
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  if(${LSB_RELEASE_ID_SHORT} EQUAL "20.04")
    set(DEBIAN_DEPENDS "libtbb2, liblz4-1, libbz2-1.0, libboost-filesystem1.71.0, libboost-date-time1.71.0, libboost-program-options1.71.0, libboost-regex1.71.0, libopencv-dev, libglew2.1, libjpeg8, libpng16-16, librealsense2, librealsense2-dkms, librealsense2-gl, librealsense2-utils")

  elseif(${LSB_RELEASE_ID_SHORT} EQUAL "18.04")
    set(DEBIAN_DEPENDS "libtbb2, liblz4-1, libbz2-1.0, libboost-filesystem1.65.1, libboost-date-time1.65.1, libboost-program-options1.65.1, libboost-regex1.65.1, libopencv-dev, libglew2.0, libjpeg8, libpng16-16, librealsense2, librealsense2-dkms, librealsense2-gl, librealsense2-utils")

  elseif(${LSB_RELEASE_ID_SHORT} EQUAL "16.04")
    set(DEBIAN_DEPENDS "libtbb2, liblz4-1, libbz2-1.0, libboost-filesystem1.58.0, libboost-date-time1.58.0, libboost-program-options1.58.0, libboost-regex1.58.0, libopencv-dev, libglew1.13, libjpeg8, libpng12-0, libstdc++6, librealsense2, librealsense2-dkms, librealsense2-gl, librealsense2-utils")
  endif()
endif(LSB_RELEASE_PROGRAM)

string(TIMESTAMP PROJECT_VERSION_REVISION "%Y%m%d%H%M")

# Configure CCache if available
if(NOT CMAKE_C_COMPILER_LAUNCHER AND NOT CMAKE_CXX_COMPILER_LAUNCHER)
  find_program(CCACHE_PROGRAM ccache)

  if(CCACHE_PROGRAM)
    message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  else()
    message(STATUS "Dind't find ccache")
  endif()
else()
  message(STATUS "Compiler launcher already set. Not configuring ccache.")
  message(STATUS "CMAKE_C_COMPILER_LAUNCHER: ${CMAKE_C_COMPILER_LAUNCHER}")
  message(STATUS "CMAKE_CXX_COMPILER_LAUNCHER: ${CMAKE_CXX_COMPILER_LAUNCHER}")
endif()

set(CMAKE_BUILD_TYPE Release)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

if(NOT CXX_MARCH)
  set(CXX_MARCH native)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Flags used for CHECK_CXX_SOURCE_COMPILES
set(CMAKE_REQUIRED_FLAGS "-Wno-error")

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN") # cmake default: "-g"
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN") # cmake default: "-O2 -g -DNDEBUG"
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG") # cmake default: "-O3 -DNDEBUG"
set(CMAKE_CXX_FLAGS_CIDEBUG "-O0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN") # CI version with no debug symbols
set(CMAKE_CXX_FLAGS_CIRELWITHDEBINFO "-O3 -DEIGEN_INITIALIZE_MATRICES_BY_NAN") # CI version with no debug symbols

# base set of compile flags
set(MULTITARGETCALIB_CXX_FLAGS "-Wall -Wextra -Werror -Wno-error=unused-parameter -ftemplate-backtrace-limit=0")

# clang-specific compile flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
  set(MULTITARGETCALIB_CXX_FLAGS "${MULTITARGETCALIB_CXX_FLAGS} -Wno-exceptions -fcolor-diagnostics -frelaxed-template-template-args -Wno-error=deprecated-declarations")

  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10)
    # These are disabled to avoid lot's of warnings in Eigen code with clang 10
    set(MULTITARGETCALIB_CXX_FLAGS "${MULTITARGETCALIB_CXX_FLAGS} -Wno-misleading-indentation -Wno-error=deprecated-copy")
  endif()

  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 15)
    # These are disabled to avoid lot's of warnings in Eigen code with clang 15
    set(MULTITARGETCALIB_CXX_FLAGS "${MULTITARGETCALIB_CXX_FLAGS} -Wno-error=unused-but-set-variable")
  endif()

  # - Added TBB_USE_GLIBCXX_VERSION macro to specify the version of GNU
  # libstdc++ when it cannot be properly recognized, e.g. when used
  # with Clang on Linux* OS. Adopted from https://github.com/wjakob/tbb
  if(NOT TBB_USE_GLIBCXX_VERSION AND UNIX AND NOT APPLE)
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
      string(REPLACE "." "0" TBB_USE_GLIBCXX_VERSION ${CMAKE_CXX_COMPILER_VERSION})
    endif()

    add_definitions(-DTBB_USE_GLIBCXX_VERSION=${TBB_USE_GLIBCXX_VERSION})
  endif()
else()
  set(MULTITARGETCALIB_CXX_FLAGS "${MULTITARGETCALIB_CXX_FLAGS} -Wno-error=maybe-uninitialized -Wno-error=implicit-fallthrough")

  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 9)
    # These are disabled to avoid lot's of warnings in Eigen code with gcc-9
    set(MULTITARGETCALIB_CXX_FLAGS "${MULTITARGETCALIB_CXX_FLAGS} -Wno-error=deprecated-copy")
  endif()

  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.0)
    # These are disabled due to warnings in pangolin
    # set(MULTITARGETCALIB_CXX_FLAGS "${MULTITARGETCALIB_CXX_FLAGS} -Wno-error=parentheses")
    set(MULTITARGETCALIB_CXX_FLAGS "${MULTITARGETCALIB_CXX_FLAGS} -Wno-parentheses")
  endif()
endif()

# Set platform / compiler specific compile flags and checks
if(APPLE)
  # Need to investigate how to reliably detect and use OpenMP on macOS...
  # set(USE_OPENMP_DEFAULT OFF)

  # Among others, setting CMAKE_FIND_FRAMEWORK to LAST fixed issues
  # with installed Mono that contains old headers (libpng, ...).
  # See: https://github.com/openMVG/openMVG/issues/1349#issuecomment-401492811
  set(CMAKE_FIND_FRAMEWORK LAST)

  if(CMAKE_SYSTEM_VERSION VERSION_LESS 19.0.0)
    # use brewed llvm's libc++
    include_directories("/usr/local/opt/llvm/include/c++/v1")
    link_directories("/usr/local/opt/llvm/lib")
    add_compile_options("-nostdinc++")

    # set(STD_CXX_FS c++fs)

    # Workaround for cmake not to filter the manually added standard include path
    # See: https://gitlab.kitware.com/cmake/cmake/issues/19227#note_669894
    list(REMOVE_ITEM CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES "/usr/local/opt/llvm/include/c++/v1")
  endif()

  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(STATUS "Detected macOS with non-Apple clang")
    add_library(multitargetcalib SHARED)

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
  # set(USE_OPENMP_DEFAULT ON)

  # assume libstdc++
  set(STD_CXX_FS stdc++fs)

  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(STATUS "Detected Linux with clang.")

  # Note: Whatever the issue was, this seems to not be an issue any more, at least with more recent clang and Ubuntu versions (tested clang-12 and Ubuntu 18.04)
  # message(WARNING "Clang on Linux is currently not fully supported. You'll likely need to get a recent version of TBB.")
  # set(LINUX_CLANG 1)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(STATUS "Detected Linux with gcc.")

  else()
    message(WARNING "Detected Linux with unsupported compiler ${CMAKE_CXX_COMPILER_ID}")
  endif()

else()
  message(WARNING "Only Linux and macOS are currently supported")
endif()

# OpenMP option and compile flags
#
# Note: OpenMP and TBB don't mix well, so we disable Eigen's parallelization.
# It's trying to paralellize matrix products during SC, which we run in a parallel_reduce using TBB.
# Turns out using OpenMP can slow down the computby factor 10-100x! So for now we discable it completely.
# One place where Eigen's parallelization could still have been useful is the CG solver in the mapper.
# We could in the future investiagte other implementations (paralellized with TBB) or selectively enabling
# Eigen's parallelization just for CG, setting number of threads to 1 everywhere else.
# Another way to ensure Eigen doesn't use OpenMP regardless of how it was built is setting the environment
# variable OMP_NUM_THREADS=1 beofre running the application.
#
# See: https://eigen.tuxfamily.org/dox/TopicMultiThreading.html
#
# If we enable BLAS / LAPACK either directly or via thirdparty libs like ceres,
# make sure to disable OpenMP for the linked BLAS library. In particular on Ubuntu it seems OpenBLAS is often installed,
# and it can have similar issues in multithreaded applications if it's own parallelization with OpenMP is enabled.
# You can set the environment varaibles OPENBLAS_NUM_THREADS=1 or OMP_NUM_THREADS=1. This is also mentioned in the ceres
# installation documentation.
#
# See also: https://github.com/xianyi/OpenBLAS/wiki/faq#multi-threaded
#
# Set EIGEN_DONT_PARALLELIZE to be sure it doesn't use openmp,
# just in case some dependency enables openmp without us noticing.
set(MULTITARGETCALIB_CXX_FLAGS "${MULTITARGETCALIB_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE")

# option(USE_OPENMP "Use OpenMP (e.g. for parallel computation in Eigen)" ${USE_OPENMP_DEFAULT})
# if(USE_OPENMP)
# message(STATUS "OpenMP Enabled")
# set(MULTITARGETCALIB_CXX_FLAGS "${MULTITARGETCALIB_CXX_FLAGS} -fopenmp")
# else()
# message(STATUS "OpenMP Disabled")
# endif()

# Make specialization for double / float optional.
# Disabling them can be useful for better compile times during development.
option(MULTITARGETCALIB_INSTANTIATIONS_DOUBLE "Instatiate templates for Scalar=double." ON)
option(MULTITARGETCALIB_INSTANTIATIONS_FLOAT "Instatiate templates for Scalar=float." ON)

if(MULTITARGETCALIB_INSTANTIATIONS_DOUBLE)
  list(APPEND MULTITARGETCALIB_COMPILE_DEFINITIONS MULTITARGETCALIB_INSTANTIATIONS_DOUBLE)
endif()

if(MULTITARGETCALIB_INSTANTIATIONS_FLOAT)
  list(APPEND MULTITARGETCALIB_COMPILE_DEFINITIONS MULTITARGETCALIB_INSTANTIATIONS_FLOAT)
endif()

# setup combined compiler flags
set(CMAKE_CXX_FLAGS "${MULTITARGETCALIB_CXX_FLAGS} ${MULTITARGETCALIB_MARCH_FLAGS} ${MULTITARGETCALIB_PASSED_CXX_FLAGS}")

# catkin_package()
set(EIGEN_INCLUDE_DIR_HINTS ${EIGEN_ROOT})
find_package(Eigen3 3.4.0 EXACT REQUIRED MODULE)
message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")

if(NOT EIGEN3_INCLUDE_DIR MATCHES "^${EIGEN_ROOT}")
  message(WARNING "Found Eigen headers are outside of specified EIGEN_ROOT '${EIGEN_ROOT}'")
endif()

find_package(TBB REQUIRED)
message(STATUS "Found TBB ${TBB_VERSION_MAJOR}.${TBB_VERSION_MINOR} (interface version ${TBB_INTERFACE_VERSION}) headers in: ${TBB_INCLUDE_DIRS}")

if(TBB_INTERFACE_VERSION LESS 11004)
  # enable global_control header for earlier TBB versions (Ubuntu 16.04, 18.04)
  add_definitions(-DTBB_PREVIEW_GLOBAL_CONTROL)
endif()

# NOTE: not specifying version, since 2, 3 or 4 is fine
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d highgui)
message(STATUS "Found OpenCV ${OpenCV_VERSION} headers in: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Found OpenCV_LIBS: ${OpenCV_LIBS}")

# Add our own custom scoped opencv target since none is provided by OpenCV itself
add_library(multitargetcalib::opencv INTERFACE IMPORTED)
set_property(TARGET multitargetcalib::opencv PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${OpenCV_INCLUDE_DIRS})
set_property(TARGET multitargetcalib::opencv PROPERTY INTERFACE_LINK_LIBRARIES ${OpenCV_LIBS})

find_package(catkin REQUIRED COMPONENTS rosbag cv_bridge)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

find_package(fmt REQUIRED)
message(STATUS "Found {fmt} ${fmt_VERSION} in: ${fmt_DIR}")

add_subdirectory(thirdparty)

# main library
add_library(multitargetcalib SHARED)

# List all header and source files with target_sources.
# This ensures, e.g., that QtCreator >= 6.0 properly recognize these headers to belong to the project.
# To support cmake < 3.13, use absolute paths (see: https://crascit.com/2016/01/31/enhanced-source-file-handling-with-target_sources/)
target_sources(multitargetcalib
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/multitargetcalib/calibration/calibration_helper.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/multitargetcalib/calibration/cam_calib.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/multitargetcalib/hash_bow/hash_bow.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/multitargetcalib/optimization/accumulator.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/multitargetcalib/optimization/linearize.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/multitargetcalib/optimization/poses_linearize.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/multitargetcalib/optimization/poses_optimize.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/multitargetcalib/threadpool/threadpool.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/multitargetcalib/utils/common_types.h
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/calibration/calibraiton_helper.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/calibration/cam_calib.cpp
)

target_link_libraries(multitargetcalib
  PUBLIC ${STD_CXX_FS} TBB::tbb multitargetcalib::multitargetcalib-headers multitargetcalib::opencv ${catkin_LIBRARIES} apriltag
  PRIVATE opengv fmt::fmt)
target_include_directories(multitargetcalib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(multitargetcalib PUBLIC ${multitargetcalib_COMPILE_DEFINITIONS})

# target_compile_definitions(multitargetcalib PUBLIC multitargetcalib_DISABLE_ASSERTS)
add_executable(multitargetcalib_calibrate src/calibrate.cpp)
target_link_libraries(multitargetcalib_calibrate multitargetcalib)
