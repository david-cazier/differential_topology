cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(Viewer
	LANGUAGES CXX C
	)

find_package(Qt5Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


set(CGOGN_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../CGoGN2)
set(CGOGN_BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../CGoGN2-build/lib/)
#set(CGOGN_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../CGoGN_2)
#set(CGOGN_BUILD_DIR ${CGOGN_ROOT_DIR}/build/debug/lib/)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++11")

set(HEADER_FILES
	viewer.h

	cgogn/dijkstra.h
		cgogn/directed_graph.h
		cgogn/graph_render.h
		cgogn/helper_functions.h
		cgogn/graph_render.h
		cgogn/helper_functions.h

		cgogn/differential_topology/pl_functions.h
		cgogn/differential_topology/reeb_graph.h

	cgogn/selection/collector_criterion.h
	cgogn/selection/collector.h

	cgogn/geometry/algos/curvature.h

	gui/surface.h
		gui/feature_points.h
		gui/graph.h
	)

set(SOURCE_FILES
	viewer.cpp
	)

# qt5_wrap_ui(SOURCE_UI
# 	viewer.ui
# 	)

# qt5_wrap_cpp(SOURCE_MOC
# 	viewer.h
# 	)

include_directories(
		${CGOGN_ROOT_DIR}
		${CGOGN_ROOT_DIR}/thirdparty/libQGLViewer/QOGLViewer
		${CGOGN_ROOT_DIR}/thirdparty/eigen-3.2.8
		${CGOGN_ROOT_DIR}/thirdparty/TinyXml2
		${CGOGN_ROOT_DIR}/thirdparty/ply
		${CGOGN_ROOT_DIR}/thirdparty/lm6)

link_directories(${CGOGN_BUILD_DIR})  #${CMAKE_BUILD_TYPE} in place of debug

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})# ${SOURCE_MOC} ${SOURCE_UI})
target_link_libraries(${PROJECT_NAME} cgogn_core_d cgogn_geometry_d cgogn_io_d cgogn_rendering_d QOGLViewer Qt5::Widgets tinyxml2_d lm6_d ply_d)
