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
	topo_render_(nullptr),
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
	delete topo_render_;
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
		drawer_->call_list(proj,view);

	if(surface_topo_rendering_)
		topo_render_->draw(proj,view);
}

void Viewer::init()
{
	glClearColor(0.1f,0.1f,0.3f,0.0f);

	surface_.init();
	feature_points_.init(bb_);
	//	reeb_graph_.init();

	topo_render_ = new cgogn::rendering::TopoRender(this);

	// drawer for simple old-school g1 rendering
	drawer_ = new cgogn::rendering::Drawer(this);
	topo_render_->update_map2<Vec3>(surface_.map_,surface_.vertex_position_);

	//	drawer_->new_list();
	//	drawer_->line_width_aa(2.0);
	//	drawer_->begin(GL_LINE_LOOP);
	//	drawer_->color3f(1.0,1.0,1.0);
	//	drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.min()[2]);
	//	drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.min()[2]);
	//	drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.min()[2]);
	//	drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.min()[2]);
	//	drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.max()[2]);
	//	drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.max()[2]);
	//	drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.max()[2]);
	//	drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.max()[2]);
	//	drawer_->end();
	//	drawer_->begin(GL_LINES);
	//	drawer_->color3f(1.0,1.0,1.0);
	//	drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.min()[2]);
	//	drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.min()[2]);
	//	drawer_->vertex3f(bb_.min()[0],bb_.min()[1],bb_.max()[2]);
	//	drawer_->vertex3f(bb_.min()[0],bb_.max()[1],bb_.max()[2]);
	//	drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.min()[2]);
	//	drawer_->vertex3f(bb_.max()[0],bb_.min()[1],bb_.max()[2]);
	//	drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.min()[2]);
	//	drawer_->vertex3f(bb_.max()[0],bb_.max()[1],bb_.max()[2]);
	//	drawer_->end();
	//	drawer_->end_list();

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
			surface_.distance_to_center_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_2:
		{
			feature_points_.clear();
			feature_points_.begin_draw();
			surface_.edge_length_weighted_geodesic_distance_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_3:
		{
			feature_points_.clear();
			feature_points_.begin_draw();
			surface_.curvature_weighted_geodesic_distance_function(feature_points_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_4:
		{
			feature_points_.clear();
			feature_points_.begin_draw();
			surface_.edge_length_weighted_morse_function(feature_points_);
//			reeb_graph_.set(surface_.reeb_graph_, surface_.vertex_position_);
			feature_points_.end_draw();
			break;
		}
		case Qt::Key_5:
		{
			feature_points_.clear();
			feature_points_.begin_draw();
			surface_.curvature_weighted_morse_function(feature_points_);
//			reeb_graph_.set(surface_.reeb_graph_, surface_.vertex_position_);
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

			//			typename Surface<Vec3>::template VertexAttribute<uint32> vindices = surface_.map_.template add_attribute<uint32, Vertex::ORBIT>("indices");
			//			uint32 count = 0;
			//			surface_.map_.foreach_cell([&] (Vertex v) { vindices[v] = count++; });

			//			std::map<uint32,Vertex> min;
			std::vector<Vertex> tab_vertices;
			surface_.map_.foreach_cell([&] (Vertex v)
			{
				if(cgogn::critical_vertex_type<Vec3::Scalar>(surface_.map_, v, surface_.scalar_field_).v_ == cgogn::CriticalVertexType::SADDLE)
				{
					//					min.insert(std::pair<uint32,Vertex>(vindices[v], v));
					tab_vertices.push_back(v);
				}
			});

			//			//			for(auto min_it : min)
			//			//			{

			//			Vertex v = cgogn::argmin<Vec3::Scalar, typename Surface<Vec3>::CMap2>(min, surface_.fpo_);

			if(selected_vertices_.empty())
				break;
			Vertex v = selected_vertices_.front();

			//			for(auto v : tab_vertices)
			{

				const Scalar v_value = surface_.scalar_field_[v];


				// 1 . sub-level set
				//

				std::vector<Vertex> inside_vertices;
				std::vector<Face> inside_faces;
				CellMarker<Surface<Vec3>::CMap2, Vertex::ORBIT> vm(surface_.map_);
				CellMarker<Surface<Vec3>::CMap2, Face::ORBIT> fm(surface_.map_);

				inside_vertices.push_back(v);
				//			vm.mark(v);

				for(uint32 i = 0 ; i < inside_vertices.size() ; ++i)
				{
					Vertex end = inside_vertices[i];

					surface_.map_.foreach_adjacent_vertex_through_edge(end, [&](Vertex e)
					{
						if(surface_.scalar_field_[e] < v_value)
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
				//
				std::vector<Edge> level_line_edges;
				CellMarker<Surface<Vec3>::CMap2, Face::ORBIT> fm2(surface_.map_);

				for(auto f : inside_faces)
				{
					surface_.map_.foreach_incident_edge(f, [&](Edge e)
					{
						std::pair<Vertex, Vertex> v = surface_.map_.vertices(e);
						if(!vm.is_marked(v.first) && !vm.is_marked(v.second))
							level_line_edges.push_back(e);
					});
				}

//				drawer_->line_width(200.2f*bb_.max_size()/50.0f);
//				drawer_->begin(GL_LINES);
//				drawer_->color3f(1.0,0.0,0.0);
//				for (auto it : level_line_edges)
//				{
//					std::pair<Vertex, Vertex> v = surface_.map_.vertices(it);
//					drawer_->vertex3fv(surface_.vertex_position_[v.first]);
//					drawer_->vertex3fv(surface_.vertex_position_[v.second]);
//				}
//				drawer_->end();

				std::cout << "nb cc = " << surface_.map_.nb_connected_components() << std::endl;

				std::pair<Face,Face> f = surface_.map_.cut_surface(level_line_edges);


				for(auto e : level_line_edges)
				surface_.map_.foreach_incident_vertex(Volume(f.first.dart), [&] (Vertex v){
					surface_.vertex_position_[v] += Vec3(-0.2f, 0.0f, 0.0f);
				});

				topo_render_->update_map2<Vec3>(surface_.map_,surface_.vertex_position_);

				std::cout << "nb cc = " << surface_.map_.nb_connected_components() << std::endl;
				surface_.update_geometry();
				surface_.update_topology();


//				cgogn::numerics::float32 c =cgogn::numerics::scale_and_clamp_to_0_1(cgogn::numerics::float32(v.dart.index),cgogn::numerics::float32(0.),cgogn::numerics::float32(10.));
//				drawer_->ball_size(0.2f*bb_.max_size()/50.0f);
//				drawer_->begin(GL_POINTS);
//				drawer_->color3f(0.5,0.5,0.0);
//				drawer_->vertex3fv(surface_.vertex_position_[v]);
//				drawer_->end();
//				drawer_->begin(GL_POINTS);
//				drawer_->color3f(0.5,c,0.0);
//				for (auto v : inside_vertices)
//				{
//					drawer_->vertex3fv(surface_.vertex_position_[v]);
//				}
//				drawer_->end();



//				drawer_->begin(GL_TRIANGLES);
//				drawer_->color3f(0.5,0.2,0.0);
//				for (auto it : inside_faces)
//				{
//					drawer_->vertex3fv(surface_.vertex_position_[Vertex(it.dart)]);
//					drawer_->vertex3fv(surface_.vertex_position_[Vertex(surface_.map_.phi1(it.dart))]);
//					drawer_->vertex3fv(surface_.vertex_position_[Vertex(surface_.map_.phi_1(it.dart))]);
//				}
//				drawer_->end();

			}


			////			std::vector<cgogn::EquivalenceClass<Vec3::Scalar, Surface<Vec3>> vec;

			//			typename Surface<Vec3>::template VertexAttribute<uint32> vindices = surface_.map_.template add_attribute<uint32, Vertex::ORBIT>("indices");
			//			uint32 count = 0;
			//			surface_.map_.foreach_cell([&] (Vertex v) { vindices[v] = count++; });

			//			std::map<uint32,Vertex> min;
			//			surface_.map_.foreach_cell([&] (Vertex v)
			//			{
			//				if(cgogn::critical_vertex_type<Vec3::Scalar>(surface_.map_, v, surface_.fpo_).v_ == cgogn::CriticalVertexType::MINIMUM)
			//					min.insert(std::pair<uint32,Vertex>(vindices[v], v));
			//			});

			//			Vertex vm = cgogn::argmin<Vec3::Scalar>(surface_.map_, surface_.fpo_);
			//			min.erase(min.find(vindices[vm]));

			//			std::map<uint32, Vertex> sub_level_set;
			//			std::map<uint32, Vertex> level_set;

			//			level_set.insert(std::pair<uint32,Vertex>(vindices[vm], vm));

			//			unsigned int stop = 0;
			//			do
			//			{

			//				if(stop == 1) //3022
			//					break;

			//				Vertex vt = cgogn::argmin<Vec3::Scalar, Surface<Vec3>>(level_set, surface_.fpo_);
			//				Vertex vmin = cgogn::argmin<Vec3::Scalar, Surface<Vec3>>(min, surface_.fpo_);

			//				if(surface_.fpo_[vmin] < surface_.fpo_[vt])
			//				{
			//					vt = vmin;
			//					min.erase(min.find(vindices[vmin]));
			//					level_set.insert(std::pair<uint32, Vertex>(vindices[vt], vt));
			//				}

			//				drawer_->ball_size(0.2f*bb_.max_size()/50.0f);
			//				drawer_->begin(GL_POINTS);
			//				drawer_->color3f(0.0,1.0,0.0);
			//				for (auto it : level_set)
			//				{
			//					drawer_->vertex3fv(surface_.vertex_position_[it.second]);
			//				}
			//				drawer_->end();

			////				//compute discrete contour

			////				cgogn::EquivalenceClass ec;




			////				vec.push_back(discrete_contour(vt, level_set));

			//				level_set.erase(level_set.find(vindices[vt]));
			//				sub_level_set.insert(std::pair<uint32, Vertex>(vindices[vt], vt));

			//				Scalar center = surface_.fpo_[vt];
			//				surface_.map_.foreach_adjacent_vertex_through_edge(vt, [&] (Vertex vn)
			//				{
			//					Scalar current = surface_.fpo_[vn];
			//					if(current > center)
			//						level_set.insert(std::pair<uint32, Vertex>(vindices[vn], vn));

			//				});

			////				drawer_->ball_size(0.2f*bb_.max_size()/50.0f);
			////				drawer_->begin(GL_POINTS);
			////				cgogn::numerics::float32 c =cgogn::numerics::scale_and_clamp_to_0_1(cgogn::numerics::float32(stop),cgogn::numerics::float32(0.),cgogn::numerics::float32(10.));
			////				drawer_->color3f(0.5,c,0.5);
			////				for (auto it : sub_level_set)
			////				{
			////					drawer_->vertex3fv(surface_.vertex_position_[it.second]);
			////				}
			////				drawer_->end();

			//				drawer_->line_width_aa(3.0);
			//				drawer_->begin(GL_LINE_LOOP);
			//				drawer_->color3f(0.0,1.0,0.0);
			//				for (auto it : level_set)
			//				{
			//					drawer_->vertex3fv(surface_.vertex_position_[it.second]);
			//				}
			//				drawer_->end();

			//				stop++;
			//			}while(!level_set.empty());


			drawer_->end_list();
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
