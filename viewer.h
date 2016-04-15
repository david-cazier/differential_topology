/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France       *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Web site: http://cgogn.unistra.fr/                                           *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/

#include <ui_viewer.h>

#include <QApplication>
#include <QMatrix4x4>

#include <qoglviewer.h>
#include <QKeyEvent>

#include <gui/surface.h>
#include <gui/feature_points.h>
#include <gui/graph.h>

#include <cgogn/geometry/algos/bounding_box.h>
#include <cgogn/rendering/drawer.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

class Viewer : public QOGLViewer
{
public:
	using Vec3 = Eigen::Vector3d;
	using Scalar = Eigen::Vector3d::Scalar;

public:
	Viewer();
	Viewer(const Viewer&) = delete;
	Viewer& operator=(const Viewer&) = delete;

	virtual ~Viewer();

	virtual void draw();
	virtual void init();

	virtual void keyPressEvent(QKeyEvent *);
	virtual void closeEvent(QCloseEvent *e);

	void import(const std::string& surfaceMesh);

private:
	Surface<Vec3> surface_;
	Surface<Vec3>::Vertex dglobal_;

	cgogn::geometry::BoundingBox<Vec3> bb_;
	cgogn::rendering::Drawer* drawer_;

	FeaturePoints<Vec3> feature_points_;
	Graph reeb_graph_;

	bool surface_rendering_;
	bool surface_phong_rendering_;
	bool surface_flat_rendering_;
	bool surface_vertices_rendering_;
	bool surface_edge_rendering_;
	bool surface_normal_rendering_;

	bool bb_rendering_;

	bool graph_vertices_rendering_;
	bool graph_edges_rendering_;

	bool feature_points_rendering_;

};
