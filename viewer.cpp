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
	surface_(this),
	volume_(this),
	dimension_(0u),
	bb_(),
	feature_points_(this),
	reeb_graph_(this),
	level_line_drawer_(nullptr),
	level_line_renderer_(nullptr),
	topo_drawer_(nullptr),
	topo_renderer_(nullptr),
	map_rendering_(true),
	flat_rendering_(false),
	vertices_rendering_(false),
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

	if (map_rendering_)
	{
		if (flat_rendering_)
			if (dimension_ == 2u)
				surface_.draw_flat(proj, view);
			else
				volume_.draw_flat(proj, view);
		else
			if (dimension_ == 2u)
				surface_.draw(proj, view);
			else
				volume_.draw(proj, view);
	}
	glDisable(GL_POLYGON_OFFSET_FILL);

	if (vertices_rendering_)
		if (dimension_ == 2u)
			surface_.draw_vertices(proj, view);
		else
			volume_.draw_vertices(proj, view);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	if (surface_edge_rendering_)
		volume_.draw_edges(proj,view);
	glDisable(GL_BLEND);

	//	if(graph_edges_rendering_)
	//		reeb_graph_.draw(proj,view);

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

	if (dimension_ == 2u)
		surface_.init();
	else
		volume_.init();

	feature_points_.init(bb_);

	// drawer for simple old-school g1 rendering
	level_line_drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
	level_line_renderer_ = level_line_drawer_->generate_renderer();

	topo_drawer_ = cgogn::make_unique<cgogn::rendering::TopoDrawer>();
	topo_renderer_ = topo_drawer_->generate_renderer();

	if (dimension_ == 2u)
		topo_drawer_->update<Vec3>(surface_.map_,surface_.vertex_position_);
	else
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

		//		cgogn::geometry::picking_vertices<Vec3>(volume_.map_,volume_.vertex_position_,A,B,selected_vertices_);
		//		std::cout << "Selected vertices: "<< selected_vertices_.size() << std::endl;

		//		if(volume_.scalar_field_.is_valid())
		//			std::cout << volume_.scalar_field_[selected_vertices_.front()] << std::endl;

	}
	QOGLViewer::mousePressEvent(e);
}

void Viewer::keyPressEvent(QKeyEvent *e)
{
	switch (e->key()) {
		case Qt::Key_M:
			map_rendering_ = !map_rendering_;
			break;
		case Qt::Key_F:
			flat_rendering_ = !flat_rendering_;
			break;
		case Qt::Key_V:
			vertices_rendering_ = !vertices_rendering_;
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
			if (dimension_ == 2u)
				surface_.height_function(feature_points_);
			else
				volume_.distance_to_boundary_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_1:
		{
			feature_points_.begin_draw();
			if (dimension_ == 2u)
				surface_.distance_to_center_function(feature_points_);
			else
				volume_.distance_to_center_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_2:
		{
			feature_points_.begin_draw();
			if (dimension_ == 2u)
				surface_.edge_length_weighted_geodesic_distance_function(feature_points_);
			else
				volume_.edge_length_weighted_geodesic_distance_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_3:
		{
			feature_points_.begin_draw();
			if (dimension_ == 2u)
				surface_.curvature_weighted_geodesic_distance_function(feature_points_);
			else
				volume_.curvature_weighted_geodesic_distance_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_4:
		{
			feature_points_.begin_draw();
			if (dimension_ == 2u)
				surface_.edge_length_weighted_morse_function(feature_points_);
			else
				volume_.edge_length_weighted_morse_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_5:
		{
			feature_points_.begin_draw();
			if (dimension_ == 2u)
				surface_.curvature_weighted_morse_function<CMap2>(feature_points_);
			else
				volume_.curvature_weighted_morse_function<CMap3>(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_6:
		{
			using namespace cgogn;
			level_line_drawer_->new_list();

			using Vertex = MorseSmallComplex<Vec3, CMap3>::Vertex;
			using Edge = MorseSmallComplex<Vec3, CMap3>::Edge;
			using Face = MorseSmallComplex<Vec3, CMap3>::Face;
			using Volume = MorseSmallComplex<Vec3, CMap3>::Volume;
			using uint32 = numerics::uint32;

			Vec3 center = cgogn::geometry::centroid<Vec3, CMap3>(volume_.map_, volume_.vertex_position_);

			cgogn::topology::ScalarField<Scalar, CMap3> scalar_field(volume_.map_, volume_.scalar_field_);
			scalar_field.differential_analysis();
			std::vector<Vertex> tab_vertices = scalar_field.get_saddles();

			std::vector<Vertex> inside_vertices;
			std::vector<Face> inside_faces;
			for(const auto& v : tab_vertices)
			{
				// 1 . sub-level set
				CellMarker<CMap3, Vertex::ORBIT> vm(volume_.map_);
				CellMarker<CMap3, Face::ORBIT> fm(volume_.map_);

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
			if (dimension_ == 2u)
				surface_.show_level_sets(feature_points_, surface_.scalar_field_);
			else
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
	if(filename.rfind(".tet") == filename.size()-4)
	{
		volume_.import(filename);
		dimension_ = 3u;
		cgogn::geometry::compute_AABB(volume_.vertex_position_, bb_);
		volume_.bb_ = bb_;
	}
	else
	{
		surface_.import(filename);
		dimension_ = 2u;
		cgogn::geometry::compute_AABB(surface_.vertex_position_, bb_);
		surface_.bb_ = bb_;
	}

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
		cgogn_log_info("viewer") << "USAGE: " << argv[0] << " [filename]";
		filename = std::string(DEFAULT_MESH_PATH) + std::string("tet/hand.tet");
		cgogn_log_info("viewer") << "Using default mesh \"" << filename << "\".";
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
