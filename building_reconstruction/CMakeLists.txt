# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

project(Polygonal_surface_reconstruction_Examples)

cmake_minimum_required(VERSION 3.1...3.23)

# PCL and its components

find_package(PCL 1.3 REQUIRED common io search kdtree octree surface filters)
list(APPEND ADDITIONAL_OPTIONS
        -VTK_MODULE_ENABLE_VTK_ViewsQt=YES
        -VTK_MODULE_ENABLE_VTK_GuiSupportQt=YES
        )

# OPENCV

#find_package(OpenCV REQUIRED)


# CGAL and its components
find_package(CGAL REQUIRED)

# Boost and its components
find_package(Boost REQUIRED)

if(NOT Boost_FOUND)

  message(
    STATUS
      "NOTICE: This project requires the Boost library, and will not be compiled."
  )

  return()

endif()

# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OPENCV

#include_directories( ${OpenCV_INCLUDE_DIRS} )
#link_directories( ${OpenCV_LIB_DIR})


# Creating entries for all C++ files with "main" routine
# ##########################################################

find_package(Eigen3 3.1.0) #(requires 3.1.0 or greater)
include(CGAL_Eigen3_support)
if(NOT TARGET CGAL::Eigen3_support)
  message(
    STATUS
      "NOTICE: This project requires Eigen 3.1 (or greater) and will not be compiled."
  )
  return()
endif()

find_package(SCIP QUIET)
include(CGAL_SCIP_support)
if(NOT TARGET CGAL::SCIP_support)
  find_package(GLPK QUIET)
  include(CGAL_GLPK_support)
  if(NOT TARGET CGAL::GLPK_support)
    message(
      STATUS
        "NOTICE: This project requires either SCIP or GLPK, and will not be compiled."
    )
    return()
  endif()
endif()


add_library(iocgal IO/Io_cgal.cpp)
target_include_directories(iocgal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/IO)
add_library(iopcl IO/Io_pcl.cpp)
target_include_directories(iopcl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/IO)
add_library(geometry Geometry/Geometry_pcl.cpp)
target_include_directories(iopcl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/IO)
add_library(surface_reconstruction surface_reconstruction.cpp)
target_include_directories(surface_reconstruction PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_library(building_reconstruction building_reconstruction.cpp)
target_include_directories(building_reconstruction PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
#create_single_source_cgal_program("polyfit_example_without_input_planes.cpp")
create_single_source_cgal_program("surface_reconstruction_example.cpp")
create_single_source_cgal_program("building_reconstruction_example.cpp")
#create_single_source_cgal_program("polyfit_example_with_region_growing.cpp")

foreach(
        target
        iocgal
        iopcl
        geometry
        surface_reconstruction
        building_reconstruction
) # polyfit_example_with_region_growing polyfit_example_without_input_planes
  target_link_libraries(${target} PUBLIC CGAL::Eigen3_support ${PCL_LIBRARIES}) #${OpenCV_LIBS}
  if(TARGET CGAL::SCIP_support)
    target_link_libraries(${target} PUBLIC CGAL::SCIP_support)
  else()
    target_link_libraries(${target} PUBLIC CGAL::GLPK_support)
  endif()
endforeach()

foreach(
  target
        building_reconstruction_example
        surface_reconstruction_example
        ) # polyfit_example_with_region_growing polyfit_example_without_input_planes
  target_link_libraries(${target} PUBLIC iocgal)
  target_link_libraries(${target} PUBLIC iopcl)
  target_link_libraries(${target} PUBLIC geometry)
  target_link_libraries(${target} PUBLIC surface_reconstruction)
  target_link_libraries(${target} PUBLIC building_reconstruction)
  target_link_libraries(${target} PUBLIC CGAL::Eigen3_support ${PCL_LIBRARIES}) #${OpenCV_LIBS}
  if(TARGET CGAL::SCIP_support)
    target_link_libraries(${target} PUBLIC CGAL::SCIP_support)
  else()
    target_link_libraries(${target} PUBLIC CGAL::GLPK_support)
  endif()
endforeach()
