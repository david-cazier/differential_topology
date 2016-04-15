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

#include "viewer.h"

Viewer::Viewer() :
	surface_(),
	bb_(),
	feature_points_(this),
	reeb_graph_(this),
	drawer_(nullptr),
	surface_rendering_(true),
	surface_phong_rendering_(true),
	surface_flat_rendering_(false),
	surface_vertices_rendering_(false),
	surface_edge_rendering_(false),
	surface_normal_rendering_(false),
	graph_vertices_rendering_(false),
	graph_edges_rendering_(false),
	feature_points_rendering_(true),
	bb_rendering_(true)
{}

Viewer::~Viewer()
{}

void Viewer::draw()
{
	QMatrix4x4 proj;
	QMatrix4x4 view;
	camera()->getProjectionMatrix(proj);
	camera()->getModelViewMatrix(view);

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f, 2.0f);

	if(surface_rendering_)
	{
		if (surface_flat_rendering_)
			surface_.draw_flat(proj,view);

		if (surface_phong_rendering_)
			surface_.draw_phong(proj,view);
	}
	glDisable(GL_POLYGON_OFFSET_FILL);

	if (surface_vertices_rendering_)
		surface_.draw_vertices(proj,view);

	//if(graph_vertices_rendering_)

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	if (surface_edge_rendering_)
		surface_.draw_edges(proj,view);

//	if(graph_edges_rendering_)
//		reeb_graph_.draw(proj,view);

	glDisable(GL_BLEND);

	if (surface_normal_rendering_)
		surface_.draw_normals(proj, view);

	if(feature_points_rendering_)
		feature_points_.draw(proj, view);

	if (bb_rendering_)
		drawer_->call_list(proj,view);
}

void Viewer::init()
{
	glClearColor(0.1f,0.1f,0.3f,0.0f);

	surface_.init();
	feature_points_.init(bb_);
//	reeb_graph_.init();

	// drawer for simple old-school g1 rendering
	drawer_ = new cgogn::rendering::Drawer(this);
	drawer_->new_list();
	drawer_->line_width_aa(2.0);
	drawer_->begin(GL_LINE_LOOP);
	drawer_->color3f(1.0,1.0,1.0);
	drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.min()[2]);
	drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.min()[2]);
	drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.min()[2]);
	drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.min()[2]);
	drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.max()[2]);
	drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.max()[2]);
	drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.max()[2]);
	drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.max()[2]);
	drawer_->end();
	drawer_->begin(GL_LINES);
	drawer_->color3f(1.0,1.0,1.0);
	drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.min()[2]);
	drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.min()[2]);
	drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.max()[2]);
	drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.max()[2]);
	drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.min()[2]);
	drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.max()[2]);
	drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.min()[2]);
	drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.max()[2]);
	drawer_->end();
	drawer_->end_list();

}

void Viewer::keyPressEvent(QKeyEvent *ev)
{
	switch (ev->key()) {
		case Qt::Key_S:
			surface_rendering_ = !surface_rendering_;
			break;
		case Qt::Key_P:
			surface_phong_rendering_ = true;
			surface_flat_rendering_ = false;
			break;
		case Qt::Key_F:
			surface_flat_rendering_ = true;
			surface_phong_rendering_ = false;
			break;
		case Qt::Key_V:
			surface_vertices_rendering_ = !surface_vertices_rendering_;
			break;
		case Qt::Key_E:
			surface_edge_rendering_ = !surface_edge_rendering_;
			break;
		case Qt::Key_B:
			bb_rendering_ = !bb_rendering_;
			break;
		case Qt::Key_G:
			graph_vertices_rendering_ = !graph_vertices_rendering_;
			graph_edges_rendering_ = !graph_edges_rendering_;
			break;
		case Qt::Key_A:
			feature_points_rendering_ = !feature_points_rendering_;
			break;
		case Qt::Key_0:
		{
			feature_points_.clear();
			feature_points_.begin_draw();
			surface_.height_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_1:
		{
			feature_points_.clear();
			feature_points_.begin_draw();
			surface_.geodesic_distance_function(feature_points_);
//			reeb_graph_.set(surface_.reeb_graph_, surface_.vertex_position_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_2:
		{
			feature_points_.clear();
			feature_points_.begin_draw();
			surface_.edge_length_weighted_morse_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_3:
		{
			feature_points_.clear();
			feature_points_.begin_draw();
			surface_.curvature_weighted_morse_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		default:
			break;
	}

	// enable QGLViewer keys
	QOGLViewer::keyPressEvent(ev);
	//update drawing
	update();
}

void Viewer::closeEvent(QCloseEvent*)
{
	delete drawer_;
}

void Viewer::import(const std::string& surfaceMesh)
{
	bb_ = surface_.import(surfaceMesh);

	setSceneRadius(bb_.diag_size()/2.0);
	Vec3 center = bb_.center();
	setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
	showEntireScene();
}

int main(int argc, char** argv)
{
	std::string surfaceMesh;
	if (argc < 2)
	{
		std::cout << "USAGE: " << argv[0] << " [filename]" << std::endl;
		surfaceMesh = std::string(DEFAULT_MESH_PATH) + std::string("aneurysm3D_1.off");
		std::cout << "Using default mesh : " << surfaceMesh << std::endl;
	}
	else
		surfaceMesh = std::string(argv[1]);

	QApplication application(argc, argv);
	qoglviewer::init_ogl_context();

	// Instantiate the viewer.
	Viewer viewer;
	viewer.setWindowTitle("simpleViewer");
	viewer.import(surfaceMesh);
	viewer.show();

	// Run main loop.
	return application.exec();
}
