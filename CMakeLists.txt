cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(Viewer
	LANGUAGES CXX C
	)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/CMakeOptions.txt)
	include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeOptions.txt)
else()
	message(FATAL_ERROR "No local options file specifying CGOGN directory: CMakeOptions.txt (see CMakeOptions.txt.sample)")
endif()

#### Default build type
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "release")
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

list(APPEND CMAKE_PREFIX_PATH "${CGOGN_RELEASE_BUILD_DIR}")

find_package(Qt5 5.4.0 COMPONENTS Widgets REQUIRED)
find_package(QOGLViewer REQUIRED)
find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(cgogn_io REQUIRED)
find_package(cgogn_rendering REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++11")

set(HEADER_FILES
	viewer.h

	cgogn/dijkstra.h
	cgogn/directed_graph.h
	cgogn/graph_render.h
	cgogn/helper_functions.h

	cgogn/differential_topology/pl_functions.h
	cgogn/differential_topology/reeb_graph.h

	cgogn/selection/collector_criterion.h
	cgogn/selection/collector.h

	cgogn/geometry/algos/curvature.h

	gui/volume.h
	gui/surface.h
	gui/feature_points.h
	gui/graph.h
	)

set(SOURCE_FILES
	viewer.cpp
	)

set(CGOGN_TEST_MESHES_PATH ${CGOGN_ROOT_DIR}/data/meshes)
add_definitions("-DCGOGN_TEST_MESHES_PATH=${CGOGN_TEST_MESHES_PATH}")

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} 
	${cgogn_core_LIBRARIES} ${cgogn_geometry_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_rendering_LIBRARIES} 
	${Qt5Widgets_LIBRARIES} ${QOGLViewer_LIBRARIES})
