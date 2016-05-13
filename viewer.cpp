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
	surface_(),
	bb_(),
	feature_points_(this),
	reeb_graph_(this),
	drawer_(nullptr),
	renderer_(nullptr),
	topo_drawer_(nullptr),
	topo_renderer_(nullptr),
	surface_rendering_(true),
	surface_phong_rendering_(true),
	surface_flat_rendering_(false),
	surface_vertices_rendering_(false),
	surface_edge_rendering_(false),
	surface_normal_rendering_(false),
	surface_topo_rendering_(false),
	graph_vertices_rendering_(false),
	graph_edges_rendering_(false),
	feature_points_rendering_(true),
	bb_rendering_(true)
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

	if (bb_rendering_ && drawer_)
		renderer_->draw(proj, view, this);

	if(surface_topo_rendering_)
		topo_renderer_->draw(proj,view, this);
}

void Viewer::init()
{
	glClearColor(0.1f,0.1f,0.3f,0.0f);

	surface_.init();
	feature_points_.init(bb_);
	//	reeb_graph_.init();

	// drawer for simple old-school g1 rendering
	drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
	renderer_ = drawer_->generate_renderer();

	topo_drawer_ = cgogn::make_unique<cgogn::rendering::TopoDrawer>();
	topo_renderer_ = topo_drawer_->generate_renderer();

	topo_drawer_->update<Vec3>(surface_.map_,surface_.vertex_position_);
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

		cgogn::geometry::picking_vertices<Vec3>(surface_.map_,surface_.vertex_position_,A,B,selected_vertices_);
		std::cout << "Selected vertices: "<< selected_vertices_.size() << std::endl;

		if(surface_.scalar_field_.is_valid())
			std::cout << surface_.scalar_field_[selected_vertices_.front()] << std::endl;

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
		case Qt::Key_0:
		{
			feature_points_.begin_draw();
			surface_.height_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_1:
		{
			feature_points_.begin_draw();
			surface_.distance_to_center_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_2:
		{
			feature_points_.begin_draw();
			surface_.edge_length_weighted_geodesic_distance_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_3:
		{
			feature_points_.begin_draw();
			surface_.curvature_weighted_geodesic_distance_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_4:
		{
			feature_points_.begin_draw();
			surface_.edge_length_weighted_morse_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_5:
		{
			feature_points_.begin_draw();
			surface_.curvature_weighted_morse_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_6:
		{
			using namespace cgogn;
			drawer_->new_list();

			using Vertex = typename Surface<Vec3>::Vertex;
			using Edge = typename Surface<Vec3>::Edge;
			using Face = typename Surface<Vec3>::Face;
			using Volume = typename Surface<Vec3>::Volume;
			using uint32 = numerics::uint32;

			Vec3 center = surface_.surface_centroid<Vec3>(surface_.map_, surface_.vertex_position_);

			std::vector<Vertex> tab_vertices;
			surface_.map_.foreach_cell([&] (Vertex v)
			{
				if(cgogn::critical_vertex_type<Vec3::Scalar>(surface_.map_, v, surface_.scalar_field_).v_ == cgogn::CriticalVertexType::SADDLE)
				{
					tab_vertices.push_back(v);
				}
			});


			std::vector<Vertex> inside_vertices;
			std::vector<Face> inside_faces;
			for(const auto& v : tab_vertices)
			{
				// 1 . sub-level set
				CellMarker<Surface<Vec3>::CMap2, Vertex::ORBIT> vm(surface_.map_);
				CellMarker<Surface<Vec3>::CMap2, Face::ORBIT> fm(surface_.map_);

				inside_vertices.push_back(v);
				const Scalar v_value = surface_.scalar_field_[v];

				for(uint32 i = 0 ; i < inside_vertices.size() ; ++i)
				{
					Vertex end = inside_vertices[i];

					surface_.map_.foreach_adjacent_vertex_through_edge(end, [&](Vertex e)
					{
						if(surface_.scalar_field_[e] > v_value)
						{
							if(!vm.is_marked(e))
							{
								inside_vertices.push_back(e);
								vm.mark(e);

								surface_.map_.foreach_incident_face(e, [&](Face f)
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
					surface_.map_.foreach_incident_edge(f, [&](Edge e)
					{
						std::pair<Vertex, Vertex> v = surface_.map_.vertices(e);
						if(!vm.is_marked(v.first) && !vm.is_marked(v.second))
							level_line_edges.push_back(e);
					});
				}

				drawer_->line_width(200.2f*bb_.max_size()/50.0f);
				drawer_->begin(GL_LINES);
				drawer_->color3f(1.0,0.0,0.0);
				for (const auto& it : level_line_edges)
				{
					std::pair<Vertex, Vertex> v = surface_.map_.vertices(it);
					drawer_->vertex3fv(surface_.vertex_position_[v.first]);
					drawer_->vertex3fv(surface_.vertex_position_[v.second]);
				}
				drawer_->end();

//				surface_.map_.cut_surface(level_line_edges);

				inside_vertices.clear();
				inside_faces.clear();
			}

			drawer_->end_list();

			Vec3 current_center(0.0,0.0,0.0);
			uint32 count = 0;

			surface_.map_.foreach_cell([&](Volume w)
			{
				surface_.map_.foreach_incident_vertex(w, [&](Vertex v)
				{
					current_center += surface_.vertex_position_[v];
					++count;
				});

				current_center /= count;
				current_center -= center;

				surface_.map_.foreach_incident_vertex(w, [&](Vertex v)
				{
					surface_.vertex_position_[v] += current_center * 0.5 ;
				});
			});

			std::cout << "nb cc = " << surface_.map_.nb_connected_components() << std::endl;
			topo_drawer_->update<Vec3>(surface_.map_,surface_.vertex_position_);
			surface_.update_geometry();
			surface_.update_topology();

			break;
		}
		case Qt::Key_7:
		{
			feature_points_.begin_draw();
			surface_.show_level_sets(feature_points_, surface_.scalar_field_);
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
	drawer_.reset();
	renderer_.reset();
	topo_drawer_.reset();
	topo_renderer_.reset();
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
