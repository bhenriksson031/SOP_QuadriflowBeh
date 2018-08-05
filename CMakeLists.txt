#BEH CONFIG
list(APPEND CMAKE_PREFIX_PATH "C:/dev/glm/glm")
list(APPEND CMAKE_PREFIX_PATH "C:/dev/Eigen/Eigen")
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOUDINI_INSTALL_DIR}/toolkit")
#set(EIGEN_INCLUDE_DIR_HINTS "C:/dev/eigen/Eigen")
set(EIGEN_INCLUDE_DIR "C:/dev/eigen/eigen-eigen-b3f3d4950030")

message(STATUS "Using houdini dir: $ENV{HOUDINI_INSTALL_DIR}")
set(GLM_INCLUDE_DIRS "C:/dev/glm/glm")
#END BEH CONFIG

#HOUDINI CONFIG
# Specify the minimum required version of CMake to build the project.
cmake_minimum_required( VERSION 3.6 )
project( SOP_QuadriflowBeh )
# Locate Houdini's libraries and header files.
# Registers an imported library target named 'Houdini'.
find_package( Houdini REQUIRED )
# Add a library with two source files.
set( library_name SOP_QuadriflowBeh )
##END HOUDINI CONFIG


##quadriflow stuff
#cmake_minimum_required(VERSION 3.1)

option(BUILD_PERFORMANCE_TEST "More subdivisition for performance test" OFF)
option(BUILD_OPENMP "Enable support for OpenMP" OFF)
option(BUILD_LOG "Enable verbose log" OFF)
option(BUILD_GUROBI "Enable GUROBI for comparison ONLY" OFF)
message(STATUS "CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
#find_package(TBB REQUIRED) #BEH EDIT, part of Houdini 
find_package(Eigen REQUIRED)
#find_package(glm REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

if (BUILD_GUROBI)
    find_package(GUROBI REQUIRED)
endif(BUILD_GUROBI)

set(LEMON_3RD_PATH 3rd/lemon-1.3.1)

find_path(
    LEMON_SOURCE_ROOT_DIR
    CMakeLists.txt
    PATHS ${CMAKE_SOURCE_DIR}/${LEMON_3RD_PATH} NO_DEFAULT_PATH
    DOC "Location of LEMON source as a CMAKE subproject"
)

if (EXISTS ${LEMON_SOURCE_ROOT_DIR})
    message(STATUS "LEMON_SOURCE_ROOT_DIR = ${LEMON_SOURCE_ROOT_DIR}")
    add_subdirectory(${LEMON_SOURCE_ROOT_DIR})
    set(LEMON_INCLUDE_DIRS ${LEMON_SOURCE_ROOT_DIR} ${PROJECT_BINARY_DIR}/${LEMON_3RD_PATH})
    set(LEMON_LIBRARIES lemon)
    unset(LEMON_ROOT_DIR CACHE)
    unset(LEMON_DIR CACHE)
    unset(LEMON_INCLUDE_DIR CACHE)
    unset(LEMON_LIBRARY CACHE)
else()
    find_package(LEMON NO_MODULE)
    find_package(LEMON REQUIRED)
    message(STATUS "here")
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")  # enable assert
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")  # enable assert
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS}")
set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG}")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -Wno-int-in-bool-context")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
    set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS}")
    set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fsanitize=address")
endif()


include_directories(src)
include_directories(3rd/pcg32)
include_directories(3rd/pss)
include_directories(3rd/libigl/include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${GLM_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${LEMON_INCLUDE_DIRS})
#include_directories(${TBB_INCLUDE_DIRS}) BEH EDIT, part of Houdini
include_directories(${GUROBI_INCLUDE_DIRS})

if (BUILD_PERFORMANCE_TEST)
    add_definitions(-DPERFORMANCE_TEST)
endif(BUILD_PERFORMANCE_TEST)

if (BUILD_OPENMP)
    add_definitions(-DWITH_OMP)
endif(BUILD_OPENMP)

if (BUILD_LOG)
    add_definitions(-DLOG_OUTPUT)
endif(BUILD_LOG)

if (BUILD_GUROBI)
    add_definitions(-DWITH_GUROBI)
endif(BUILD_GUROBI)


set(
    SOP_quadriflow_SRC
    src/adjacent-matrix.cpp
    src/adjacent-matrix.hpp
    src/compare-key.hpp
    src/config.hpp
    src/dedge.cpp
    src/dedge.hpp
    src/disajoint-tree.hpp
    src/dset.hpp
    src/field-math.hpp
    src/flow.hpp
    src/hierarchy.cpp
    src/hierarchy.hpp
    src/loader.cpp
    src/loader.hpp
    src/localsat.cpp
    src/localsat.hpp
    #src/main.cpp
    src/merge-vertex.cpp
    src/merge-vertex.hpp
    src/optimizer.cpp
    src/optimizer.hpp
    src/parametrizer.cpp
    src/parametrizer-flip.cpp
    src/parametrizer-int.cpp
    src/parametrizer-mesh.cpp
    src/parametrizer-scale.cpp
    src/parametrizer-sing.cpp
    src/parametrizer.hpp
    src/serialize.hpp
    src/subdivide.cpp
    src/subdivide.hpp
    src/parametrizer-houdini.cpp
)

#add_executable(
#    quadriflow
#    ${quadriflow_SRC}
#)

#from earcut WIN32
add_definitions(
    -DVERSION=\"${SOP_TRIANGULATE_EAR_CUT_VERSION_FULL}\"
    -DWIN32
    -DI386
    -nologo
    -TP
    -Zc:forScope
    -DSWAP_BITFIELDS
    -D_WIN32_WINNT=0x0502
    -DNOMINMAX
    -DSTRICT
    -DWIN32_LEAN_AND_MEAN
    -D_USE_MATH_DEFINES
    -D_CRT_SECURE_NO_DEPRECATE
    -D_CRT_NONSTDC_NO_DEPRECATE
    -D_SCL_SECURE_NO_WARNINGS
    -DBOOST_ALL_NO_LIB
	-DHBOOST_ALL_NO_LIB
    -wd4355
    -w14996
    #-O2
    -DNDEBUG
    -MD
    -EHsc
    -GR
    -bigobj
)
#From earcut
add_definitions(
    -DMAKING_DSO
)

##CONFIG OUTPUT aproach 1
add_library( ${library_name} SHARED
    ${library_name}.C
    ${library_name}.h
	${SOP_quadriflow_SRC}
)

##CONFIG OUTPUT aproach 2
#include_directories(src)
#add_library( ${library_name} SHARED
#    ${library_name}.C
#    ${library_name}.h
#)

#
# Link against the Houdini libraries, and add required include directories and compile definitions.


target_link_libraries( ${library_name} 
    Houdini 
    #${TBB_LIBRARIES} #comes with Houdini?
    ${Boost_LIBRARIES}
    ${LEMON_LIBRARIES}
    ${GUROBI_LIBRARIES}
)
# Configure several common target properties, such as its output directory.
houdini_configure_target( ${library_name} )

##END CONFIG OUTPUT