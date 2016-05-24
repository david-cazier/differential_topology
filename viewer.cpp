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

#include <cgogn/geometry/algos/picking.h>

Viewer::Viewer() :
	volume_(this),
	bb_(),
	feature_points_(this),
	reeb_graph_(this),
	level_line_drawer_(nullptr),
	level_line_renderer_(nullptr),
	topo_drawer_(nullptr),
	topo_renderer_(nullptr),
	surface_rendering_(true),
	surface_phong_rendering_(false),
	surface_flat_rendering_(true),
	surface_vertices_rendering_(false),
	surface_edge_rendering_(false),
	surface_topo_rendering_(false),
	graph_vertices_rendering_(false),
	graph_edges_rendering_(false),
	feature_points_rendering_(true),
	bb_rendering_(true),
	expl_(0.8f)
{}

Viewer::~Viewer()
{
}

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
			volume_.draw_flat(proj,view);

		if (surface_phong_rendering_)
			volume_.draw_volume(proj,view);
	}
	glDisable(GL_POLYGON_OFFSET_FILL);

//	if (surface_vertices_rendering_)
//		volume_.draw_vertices(proj,view);

	//if(graph_vertices_rendering_)

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	if (surface_edge_rendering_)
		volume_.draw_edges(proj,view);

	//	if(graph_edges_rendering_)
	//		reeb_graph_.draw(proj,view);

    glDisable(GL_BLEND);

	if(feature_points_rendering_)
		feature_points_.draw(proj, view);

	if (bb_rendering_ && level_line_drawer_)
		level_line_renderer_->draw(proj, view, this);

	if(surface_topo_rendering_)
		topo_renderer_->draw(proj,view, this);
}

void Viewer::init()
{
	glClearColor(0.1f,0.1f,0.3f,0.0f);

	volume_.init();
	feature_points_.init(bb_);
	//	reeb_graph_.init();

	// drawer for simple old-school g1 rendering
	level_line_drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
	level_line_renderer_ = level_line_drawer_->generate_renderer();

	topo_drawer_ = cgogn::make_unique<cgogn::rendering::TopoDrawer>();
	topo_renderer_ = topo_drawer_->generate_renderer();

	topo_drawer_->update<Vec3>(volume_.map_,volume_.vertex_position_);
}

void Viewer::mousePressEvent(QMouseEvent* e)
{
	if (e->modifiers() & Qt::ShiftModifier)
	{
		qoglviewer::Vec P;
		qoglviewer::Vec Q;

		P = camera()->unprojectedCoordinatesOf(qoglviewer::Vec(e->x(),e->y(),0.0));
		Q = camera()->unprojectedCoordinatesOf(qoglviewer::Vec(e->x(),e->y(),1.0));

		Vec3 A(P[0],P[1],P[2]);
		Vec3 B(Q[0],Q[1],Q[2]);

		cgogn::geometry::picking_vertices<Vec3>(volume_.map_,volume_.vertex_position_,A,B,selected_vertices_);
		std::cout << "Selected vertices: "<< selected_vertices_.size() << std::endl;

		if(volume_.scalar_field_.is_valid())
			std::cout << volume_.scalar_field_[selected_vertices_.front()] << std::endl;

	}
	QOGLViewer::mousePressEvent(e);
}

void Viewer::keyPressEvent(QKeyEvent *e)
{
	switch (e->key()) {
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
		case Qt::Key_T:
			surface_topo_rendering_ = !surface_topo_rendering_;
			break;
		case Qt::Key_Plus:
			expl_ += 0.05f;
			break;
		case Qt::Key_Minus:
			expl_ -= 0.05f;
			break;
		case Qt::Key_0:
		{
			feature_points_.begin_draw();
			//volume_.height_function(feature_points_);
			volume_.distance_to_boundary_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_1:
		{
			feature_points_.begin_draw();
			volume_.distance_to_center_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_2:
		{
			feature_points_.begin_draw();
			volume_.edge_length_weighted_geodesic_distance_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_3:
		{
			feature_points_.begin_draw();
			volume_.curvature_weighted_geodesic_distance_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_4:
		{
			feature_points_.begin_draw();
			volume_.edge_length_weighted_morse_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_5:
		{
			feature_points_.begin_draw();
			volume_.curvature_weighted_morse_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_6:
		{
			using namespace cgogn;
			level_line_drawer_->new_list();

			using Vertex = VolumeMesh<Vec3>::Vertex;
			using Edge = VolumeMesh<Vec3>::Edge;
			using Face = VolumeMesh<Vec3>::Face;
			using Volume = VolumeMesh<Vec3>::Volume;
			using uint32 = numerics::uint32;

			Vec3 center = volume_.map_centroid<Vec3>(volume_.map_, volume_.vertex_position_);

			std::vector<Vertex> tab_vertices;
			volume_.map_.foreach_cell([&] (Vertex v)
			{
				if(cgogn::volume_critical_vertex_type<Vec3::Scalar>(volume_.map_, v, volume_.scalar_field_).v_ == cgogn::CriticalVertexType::SADDLE)
				{
					tab_vertices.push_back(v);
				}
			});


			std::vector<Vertex> inside_vertices;
			std::vector<Face> inside_faces;
			for(const auto& v : tab_vertices)
			{
				// 1 . sub-level set
				CellMarker<VolumeMesh<Vec3>::CMap3, Vertex::ORBIT> vm(volume_.map_);
				CellMarker<VolumeMesh<Vec3>::CMap3, Face::ORBIT> fm(volume_.map_);

				inside_vertices.push_back(v);
				const Scalar v_value = volume_.scalar_field_[v];

				for(uint32 i = 0 ; i < inside_vertices.size() ; ++i)
				{
					Vertex end = inside_vertices[i];

					volume_.map_.foreach_adjacent_vertex_through_edge(end, [&](Vertex e)
					{
						if(volume_.scalar_field_[e] > v_value)
						{
							if(!vm.is_marked(e))
							{
								inside_vertices.push_back(e);
								vm.mark(e);

								volume_.map_.foreach_incident_face(e, [&](Face f)
								{
									if(!fm.is_marked(f))
									{
										inside_faces.push_back(f);
										fm.mark(f);
									}
								});
							}
						}
					});
				}

				// 2. discrete level line
				std::vector<Edge> level_line_edges;

				for(const auto& f : inside_faces)
				{
					volume_.map_.foreach_incident_edge(f, [&](Edge e)
					{
						std::pair<Vertex, Vertex> v = volume_.map_.vertices(e);
						if(!vm.is_marked(v.first) && !vm.is_marked(v.second))
							level_line_edges.push_back(e);
					});
				}

				level_line_drawer_->line_width(200.2f*bb_.max_size()/50.0f);
				level_line_drawer_->begin(GL_LINES);
				level_line_drawer_->color3f(1.0,0.0,0.0);
				for (const auto& it : level_line_edges)
				{
					std::pair<Vertex, Vertex> v = volume_.map_.vertices(it);
					level_line_drawer_->vertex3fv(volume_.vertex_position_[v.first]);
					level_line_drawer_->vertex3fv(volume_.vertex_position_[v.second]);
				}
				level_line_drawer_->end();

//				surface_.map_.cut_surface(level_line_edges);

				inside_vertices.clear();
				inside_faces.clear();
			}

			level_line_drawer_->end_list();

			Vec3 current_center(0.0,0.0,0.0);
			uint32 count = 0;

			volume_.map_.foreach_cell([&](Volume w)
			{
				volume_.map_.foreach_incident_vertex(w, [&](Vertex v)
				{
					current_center += volume_.vertex_position_[v];
					++count;
				});

				current_center /= count;
				current_center -= center;

				volume_.map_.foreach_incident_vertex(w, [&](Vertex v)
				{
					volume_.vertex_position_[v] += current_center * 0.5 ;
				});
			});

			std::cout << "nb cc = " << volume_.map_.nb_connected_components() << std::endl;
			topo_drawer_->update<Vec3>(volume_.map_,volume_.vertex_position_);
			volume_.update_geometry();
			volume_.update_topology();

			break;
		}
		case Qt::Key_7:
		{
			feature_points_.begin_draw();
			volume_.show_level_sets(feature_points_, volume_.scalar_field_);
			feature_points_.end_draw();
			break;
		}
		default:
			break;
	}

	// enable QGLViewer keys
	QOGLViewer::keyPressEvent(e);
	//update drawing
	update();
}

void Viewer::closeEvent(QCloseEvent*)
{
//	drawer_.reset();
//	renderer_.reset();
	topo_drawer_.reset();
	topo_renderer_.reset();
}

void Viewer::import(const std::string& filename)
{
	bb_ = volume_.import(filename);

	setSceneRadius(bb_.diag_size()/2.0);
	Vec3 center = bb_.center();
	setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
	showEntireScene();
}

int main(int argc, char** argv)
{
	std::string filename;
	if (argc < 2)
	{
		std::cout << "USAGE: " << argv[0] << " [filename]" << std::endl;
		filename = std::string("D:/Dev/CGoGN_2/data/meshes/tet/hand.tet");
		std::cout << "Using default mesh : " << filename << std::endl;
	}
	else
		filename = std::string(argv[1]);

	QApplication application(argc, argv);
	qoglviewer::init_ogl_context();

	// Instantiate the viewer.
	Viewer viewer;
	viewer.setWindowTitle("Differential Topology");
	viewer.import(filename);
	viewer.show();

	// Run main loop.
	return application.exec();
}
