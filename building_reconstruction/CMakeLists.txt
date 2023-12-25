# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.23)

project(Polygonal_surface_reconstruction_Examples)

#set(CMAKE_CXX_COMPILER "g++-13")
#set(CMAKE_CXX_COMPILER "clang")

# OPENMP variables
set(OpenMP_C_FLAGS "-fopenmp=lomp")
set(OpenMP_CXX_FLAGS "-fopenmp=lomp")
set(OpenMP_C_LIB_NAMES "libomp")
set(OpenMP_CXX_LIB_NAMES "libomp")
set(OpenMP_libomp_LIBRARY "/usr/local/Cellar/libomp/17.0.2/lib/libomp.dylib")
set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp /usr/local/Cellar/libomp/17.0.2/lib/libomp.dylib -I/usr/local/Cellar/libomp/17.0.2/include")
set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp /usr/local/Cellar/libomp/17.0.2/lib/libomp.dylib -I/usr/local/Cellar/libomp/17.0.2/include")

#add_compile_options(-std=c++11)

find_package(Vulkan)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

# PCL and its components
find_package(PCL 1.12.1 REQUIRED common io search kdtree octree surface filters)
list(APPEND ADDITIONAL_OPTIONS
        -VTK_MODULE_ENABLE_VTK_ViewsQt=YES
        -VTK_MODULE_ENABLE_VTK_GuiSupportQt=YES
        )

# VTK
find_package(VTK)

# OPENCV
find_package(OpenCV REQUIRED)

# CGAL and its components
find_package(CGAL REQUIRED)

# Boost and its components
find_package(Boost REQUIRED)

if (NOT Boost_FOUND)

    message(
            STATUS
            "NOTICE: This project requires the Boost library, and will not be compiled."
    )

    return()

endif ()

# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OPENCV
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

# Creating entries for all C++ files with "main" routine
# ##########################################################

find_package(Eigen3 3.1.0) #(requires 3.1.0 or greater)
include(CGAL_Eigen3_support)
if (NOT TARGET CGAL::Eigen3_support)
    message(
            STATUS
            "NOTICE: This project requires Eigen 3.1 (or greater) and will not be compiled."
    )
    return()
endif ()

find_package(SCIP QUIET)
include(CGAL_SCIP_support)
if (NOT TARGET CGAL::SCIP_support)
    find_package(GLPK QUIET)
    include(CGAL_GLPK_support)
    if (NOT TARGET CGAL::GLPK_support)
        message(
                STATUS
                "NOTICE: This project requires either SCIP or GLPK, and will not be compiled."
        )
        return()
    endif ()
endif ()


add_library(iocgal STATIC IO/Io_cgal.h IO/Io_cgal.cpp)
add_library(iopcl STATIC IO/Io_pcl.h IO/Io_pcl.cpp)
add_library(geometry STATIC Geometry/Geometry_pcl.h Geometry/Geometry_pcl.cpp)
add_library(algo_reconstruction STATIC Algo_reconstruction.h Algo_reconstruction.cpp)
add_library(shift STATIC PointClouds/Shift.h PointClouds/Shift.cpp)
add_library(string_utils STATIC StringUtils/PathUtils.h StringUtils/PathUtils.cpp)
add_library(texturing_mapping STATIC Texturing/Texturing_mapping.h Texturing/Texturing_mapping.cpp
        Texturing/IO/Image_txt_reading.h Texturing/IO/Image_txt_reading.cpp
        Texturing/Image/Panorama2cubemap.h Texturing/Image/Panorama2cubemap.cpp
        Texturing/Image/Lidar2depth.h Texturing/Image/Lidar2depth.cpp)
add_library(metrics STATIC Metrics/Metrics.h Metrics/Metrics.cpp
            Metrics/BlurEstimation.h Metrics/BlurEstimation.cpp)
create_single_source_cgal_program("texturing_map_example.cpp")
create_single_source_cgal_program("metric_image_compare_example.cpp")
#add_library(surface_reconstruction STATIC surface_reconstruction.cpp)
add_library(building_reconstruction STATIC building_reconstruction.cpp)
#create_single_source_cgal_program("surface_reconstruction_example.cpp")
create_single_source_cgal_program("building_reconstruction_example.cpp")

foreach (
        target
        metrics
        texturing_mapping
)
    target_link_libraries(${target} PUBLIC ${OpenCV_LIBS})
endforeach ()

foreach (
        target
        iocgal
        iopcl
        geometry
        algo_reconstruction
        shift
        string_utils
        texturing_mapping
        metrics
        #        surface_reconstruction
        building_reconstruction
)
    target_link_libraries(${target} PUBLIC CGAL::Eigen3_support ${PCL_LIBRARIES})
    if (TARGET CGAL::SCIP_support)
        target_link_libraries(${target} PUBLIC CGAL::SCIP_support)
    else ()
        target_link_libraries(${target} PUBLIC CGAL::GLPK_support)
    endif ()
endforeach ()

target_link_libraries(metric_image_compare_example PUBLIC metrics)

foreach (
        target
        texturing_map_example
)
    target_link_libraries(${target} PUBLIC iocgal)
    target_link_libraries(${target} PUBLIC iopcl)
    target_link_libraries(${target} PUBLIC geometry)
    target_link_libraries(${target} PUBLIC algo_reconstruction)
    target_link_libraries(${target} PUBLIC shift)
    target_link_libraries(${target} PUBLIC string_utils)
    #  target_link_libraries(${target} PUBLIC surface_reconstruction)
    target_link_libraries(${target} PUBLIC building_reconstruction)
    target_link_libraries(${target} PUBLIC texturing_mapping)
    target_link_libraries(${target} PUBLIC CGAL::Eigen3_support ${PCL_LIBRARIES})
    if (TARGET CGAL::SCIP_support)
        target_link_libraries(${target} PUBLIC CGAL::SCIP_support)
    else ()
        target_link_libraries(${target} PUBLIC CGAL::GLPK_support)
    endif ()
endforeach ()

foreach (
        target
        building_reconstruction_example
        #        surface_reconstruction_example
)
    target_link_libraries(${target} PUBLIC iocgal)
    target_link_libraries(${target} PUBLIC iopcl)
    target_link_libraries(${target} PUBLIC geometry)
    target_link_libraries(${target} PUBLIC algo_reconstruction)
    target_link_libraries(${target} PUBLIC shift)
    target_link_libraries(${target} PUBLIC string_utils)
    #  target_link_libraries(${target} PUBLIC surface_reconstruction)
    target_link_libraries(${target} PUBLIC building_reconstruction)
    target_link_libraries(${target} PUBLIC texturing_mapping)
    target_link_libraries(${target} PUBLIC CGAL::Eigen3_support ${PCL_LIBRARIES})
    if (TARGET CGAL::SCIP_support)
        target_link_libraries(${target} PUBLIC CGAL::SCIP_support)
    else ()
        target_link_libraries(${target} PUBLIC CGAL::GLPK_support)
    endif ()
endforeach ()

