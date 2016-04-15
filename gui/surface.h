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


#ifndef SURFACE_H
#define SURFACE_H

#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/core/utils/definitions.h>
#include <cgogn/io/map_import.h>
#include <cgogn/io/map_export.h>


#include <cgogn/rendering/map_render.h>

#include <cgogn/geometry/algos/normal.h>

#include <cgogn/rendering/shaders/shader_simple_color.h>
#include <cgogn/rendering/shaders/shader_flat.h>
#include <cgogn/rendering/shaders/shader_phong.h>
#include <cgogn/rendering/shaders/shader_vector_per_vertex.h>
#include <cgogn/rendering/shaders/vbo.h>

#include <cgogn/geometry/algos/angle.h>
#include <cgogn/geometry/algos/area.h>
#include <cgogn/geometry/algos/length.h>
#include <cgogn/geometry/algos/bounding_box.h>
#include <cgogn/geometry/algos/ear_triangulation.h>

#include <cgogn/core/utils/numerics.h>


// perso functions

#include <cgogn/geometry/algos/curvature.h>
#include <cgogn/helper_functions.h>

#include <gui/feature_points.h>

#include <cgogn/differential_topology/reeb_graph.h>

template <typename VEC3>
class Surface
{
public:
	using Vec3 = VEC3;
	using Scalar = typename Vec3::Scalar;

	using CMap2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
	using Vertex = CMap2::Vertex;
	using Edge = CMap2::Edge;
	template<typename T>
	using VertexAttribute = CMap2::VertexAttribute<T>;
	template<typename T>
	using EdgeAttribute = CMap2::EdgeAttribute<T>;

public:
	CMap2 map_;

	VertexAttribute<Vec3> vertex_position_;
	VertexAttribute<Vec3> vertex_normal_;
	VertexAttribute<Scalar> fpo_;


	EdgeAttribute<Scalar> edge_metric_;

	cgogn::geometry::BoundingBox<Vec3> bb_;

	cgogn::rendering::MapRender* render_;

	cgogn::rendering::VBO* vbo_pos_;
	cgogn::rendering::VBO* vbo_norm_;
	cgogn::rendering::VBO* vbo_color_;
	cgogn::rendering::VBO* vbo_sphere_sz_;

	cgogn::rendering::ShaderBoldLine* shader_edge_;
	cgogn::rendering::ShaderFlat* shader_flat_;
	cgogn::rendering::ShaderVectorPerVertex* shader_normal_;
	cgogn::rendering::ShaderPhong* shader_phong_;
	cgogn::rendering::ShaderPointSprite* shader_point_sprite_;


	cgogn::ReebGraph<Vec3, CMap2>* reeb_graph_;

public:

	Surface():
		map_(),
		vertex_position_(),
		vertex_normal_(),
		fpo_(),
		edge_metric_(),
		bb_(),
		render_(nullptr),
		vbo_pos_(nullptr),
		vbo_norm_(nullptr),
		vbo_color_(nullptr),
		vbo_sphere_sz_(nullptr),
		shader_edge_(nullptr),
		shader_flat_(nullptr),
		shader_normal_(nullptr),
		shader_phong_(nullptr),
		shader_point_sprite_(nullptr)
	{
		reeb_graph_ = new cgogn::ReebGraph<Vec3, CMap2>(map_);
	}

	~Surface()
	{
		delete render_;
		delete vbo_pos_;
		delete vbo_norm_;
		delete vbo_color_;
		delete vbo_sphere_sz_;
		delete shader_edge_;
		delete shader_flat_;
		delete shader_normal_;
		delete shader_phong_;
		delete shader_point_sprite_;
		delete reeb_graph_;
	}


	template <typename T, typename MAP>
	inline T surface_centroid(MAP& map, VertexAttribute<T>& attribute)
	{
		T result;
		result.setZero();
		unsigned int count = 0;

		//		for(auto& v : attribute)
		//		{
		//			result += v;
		//			++count;
		//		}
		map.foreach_cell([&](typename MAP::Vertex v)
		{
			result += attribute[v];
			++count;
		});

		result /= typename T::Scalar(count);

		return result;
	}


	void init()
	{
		vbo_pos_ = new cgogn::rendering::VBO(3);
		cgogn::rendering::update_vbo(vertex_position_, *vbo_pos_);

		vbo_norm_ = new cgogn::rendering::VBO(3);
		cgogn::rendering::update_vbo(vertex_normal_, *vbo_norm_);

		// fill a color vbo with abs of normals
		vbo_color_ = new cgogn::rendering::VBO(3);
		cgogn::rendering::update_vbo(vertex_normal_, *vbo_color_,[] (const Vec3& n) -> std::array<float,3>
		{
			return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
		});

		// fill a sphere size vbo
		vbo_sphere_sz_ = new cgogn::rendering::VBO(1);
		cgogn::rendering::update_vbo(vertex_normal_, *vbo_sphere_sz_,[&] (const Vec3& n) -> float
		{
			return bb_.diag_size()/1000.0 * (1.0 + 2.0*std::abs(n[2]));
		});

		render_ = new cgogn::rendering::MapRender();
		render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS, vertex_position_);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::LINES, vertex_position_);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, vertex_position_);

		shader_point_sprite_ = new cgogn::rendering::ShaderPointSprite(true,true);
		shader_point_sprite_->add_vao();
		shader_point_sprite_->set_vao(0, vbo_pos_,vbo_color_,vbo_sphere_sz_);
		shader_point_sprite_->bind();
		shader_point_sprite_->set_size(bb_.diag_size()/100.0);
		shader_point_sprite_->set_color(QColor(255,0,0));
		shader_point_sprite_->release();

		shader_edge_ = new cgogn::rendering::ShaderBoldLine() ;
		shader_edge_->add_vao();
		shader_edge_->set_vao(0, vbo_pos_);
		shader_edge_->bind();
		shader_edge_->set_color(QColor(255,255,0));
		shader_edge_->release();

		shader_flat_ = new cgogn::rendering::ShaderFlat;
		shader_flat_->add_vao();
		shader_flat_->set_vao(0, vbo_pos_);
		shader_flat_->bind();
		shader_flat_->set_front_color(QColor(0,200,0));
		shader_flat_->set_back_color(QColor(0,0,200));
		shader_flat_->set_ambiant_color(QColor(5,5,5));
		shader_flat_->release();

		shader_normal_ = new cgogn::rendering::ShaderVectorPerVertex;
		shader_normal_->add_vao();
		shader_normal_->set_vao(0, vbo_pos_, vbo_norm_);
		shader_normal_->bind();
		shader_normal_->set_color(QColor(200,0,200));
		shader_normal_->set_length(bb_.diag_size()/50);
		shader_normal_->release();

		shader_phong_ = new cgogn::rendering::ShaderPhong(true);
		shader_phong_->add_vao();
		shader_phong_->set_vao(0, vbo_pos_, vbo_norm_, vbo_color_);
		shader_phong_->bind();
//		shader_phong_->set_ambiant_color(QColor(5,5,5));
//		shader_phong_->set_double_side(true);
//		shader_phong_->set_specular_color(QColor(255,255,255));
//		shader_phong_->set_specular_coef(10.0);
		shader_phong_->release();
	}

	void update_topology()
	{
		render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS, vertex_position_);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::LINES, vertex_position_);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, vertex_position_);
	}

	void update_color(VertexAttribute<Scalar> scalar)
	{
		double min = std::numeric_limits<double>::max();
		double max = std::numeric_limits<double>::min();
		for(auto& v : scalar)
		{
			min = std::min(min, v);
			max = std::max(max, v);
		}

		cgogn::rendering::update_vbo(scalar, *vbo_color_,
									 [min, max] (const Scalar& n) -> std::array<float,3>
		{
			return cgogn::color_map_hash(cgogn::numerics::scale_to_0_1(n, min, max));
		});
	}

	void draw_phong(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		shader_phong_->bind();
		shader_phong_->set_matrices(proj,view);
		shader_phong_->bind_vao(0);
		render_->draw(cgogn::rendering::TRIANGLES);
		shader_phong_->release_vao(0);
		shader_phong_->release();
	}

	void draw_flat(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		shader_flat_->bind();
		shader_flat_->set_matrices(proj,view);
		//		shader_flat_->set_local_light_position(QVector3D(bb.max()[0],bb.max()[1],bb.max()[2]), view);
		shader_flat_->bind_vao(0);
		render_->draw(cgogn::rendering::TRIANGLES);
		shader_flat_->release_vao(0);
		shader_flat_->release();
	}

	void draw_vertices(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		shader_point_sprite_->bind();
		shader_point_sprite_->set_matrices(proj,view);
		shader_point_sprite_->bind_vao(0);
		render_->draw(cgogn::rendering::POINTS);
		shader_point_sprite_->release_vao(0);
		shader_point_sprite_->release();
	}

	void draw_edges(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		shader_edge_->bind();
		shader_edge_->set_matrices(proj,view);
		shader_edge_->bind_vao(0);
		shader_edge_->set_width(2.5f);
		render_->draw(cgogn::rendering::LINES);
		shader_edge_->release_vao(0);
		shader_edge_->release();

	}

	void draw_normals(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		shader_normal_->bind();
		shader_normal_->set_matrices(proj,view);
		shader_normal_->bind_vao(0);
		render_->draw(cgogn::rendering::POINTS);
		shader_normal_->release_vao(0);
		shader_normal_->release();
	}

	cgogn::geometry::BoundingBox<Vec3> import(const std::string& filename)
	{
		cgogn::io::import_surface<Vec3>(map_, filename);

		vertex_position_ = map_.get_attribute<Vec3, Vertex::ORBIT>("position");
		vertex_normal_ = map_.add_attribute<Vec3, Vertex::ORBIT>("normal");

		cgogn::geometry::compute_normal_vertices<Vec3>(map_, vertex_position_, vertex_normal_);
		cgogn::geometry::compute_bounding_box(vertex_position_, bb_);

		return bb_;
	}

	Vertex central_vertex()
	{
		Vec3 barycenter = surface_centroid<Vec3>(map_, vertex_position_);

		Scalar min_distance = std::numeric_limits<Scalar>::max();
		Vertex min_vertex;

		map_.foreach_cell([&](Vertex v)
		{
			Vec3 diffenre = vertex_position_[v] - barycenter;
			Scalar distance = diffenre.norm();

			if(distance < min_distance)
			{
				min_distance = distance;
				min_vertex = v;
			}
		});
		return min_vertex;
	}

	Vertex farthest_extremity(std::vector<Vertex> vertices,
							  const EdgeAttribute<Scalar>& weight,
							  VertexAttribute<Scalar>& scalar_field)
	{
		VertexAttribute<Vertex> path_to_source = map_.add_attribute<Vertex, Vertex::ORBIT>("path_to_source");
		cgogn::dijkstra_compute_paths<Scalar>(map_, weight, vertices, scalar_field, path_to_source);

		Vertex extremity;
		Scalar max_distance(0);
		map_.foreach_cell([&](Vertex v)
		{
			Scalar distance = scalar_field[v];

			if(distance > max_distance)
			{
				max_distance = distance;
				extremity = v;
			}
		});
		map_.remove_attribute(path_to_source);
		return extremity;
	}

	void height_function(FeaturePoints<VEC3>& fp)
	{
		VertexAttribute<Scalar> scalar = map_.add_attribute<Scalar, Vertex::ORBIT>("scalar");

		cgogn::height_pl_function<Vec3>(map_, vertex_position_, scalar);

		update_color(scalar);

		fp.extract(map_, scalar, vertex_position_);

		reeb_graph_->compute(scalar);

		map_.remove_attribute(scalar);
	}

	void geodesic_distance_function(FeaturePoints<VEC3>& fp, int n)
	{
		VertexAttribute<Scalar> scalar = map_.add_attribute<Scalar, Vertex::ORBIT>("scalar");
		EdgeAttribute<Scalar> weight = map_.add_attribute<Scalar, Edge::ORBIT>("weight");
		map_.foreach_cell([&](Edge e)
		{
			weight[e] = cgogn::geometry::edge_length<Vec3>(map_, e, vertex_position_);
		});

		std::vector<Vertex> vertices;

		Vertex v0 = central_vertex();
		Vertex v1 = farthest_extremity({v0}, weight, scalar);
		vertices.push_back(v1);
		while (n>1) {
			Vertex v = farthest_extremity(vertices, weight, scalar);
			vertices.push_back(v);
			--n;
		}

		cgogn::geodesic_distance_pl_function<Scalar>(map_, vertices, weight, scalar);
		update_color(scalar);

		fp.extract(map_, scalar, vertex_position_);

		map_.remove_attribute(scalar);
		map_.remove_attribute(weight);
	}

	void edge_length_weighted_morse_function(FeaturePoints<VEC3>& fp)
	{

		EdgeAttribute<Scalar> weight = map_.add_attribute<Scalar, Edge::ORBIT>("weight");
		map_.foreach_cell([&](Edge e)
		{
			weight[e] = cgogn::geometry::edge_length<Vec3>(map_, e, vertex_position_);
		});

		morse_function(fp,weight);

		map_.remove_attribute(weight);
	}

	void curvature_weighted_morse_function(FeaturePoints<VEC3>& fp)
	{
		compute_curvature();
		morse_function(fp, edge_metric_);
	}

	/********************/

	void morse_function(FeaturePoints<VEC3>& fp, EdgeAttribute<Scalar>& weight)
	{
		//1. compute v0: the vertex whose distance to the barycenter of map is minimal
		Vertex v0 = central_vertex();

		//2. map the vertices to their geodesic distance to v0: find the vertex v1 that maximizes f0
		VertexAttribute<Scalar> f0 = map_.add_attribute<Scalar, Vertex::ORBIT>("f0");
		Vertex v1 = farthest_extremity({v0}, weight, f0);

		//3. map the vertices to their geodesic distance to v1: find the vertex v2  that maximizes f1
		VertexAttribute<Scalar> f1 = map_.add_attribute<Scalar, Vertex::ORBIT>("f1");
		Vertex v2 = farthest_extremity({v1}, weight, f1);

		//4. map the vertices to their geodesic distance to v2
		VertexAttribute<Scalar> f2 = map_.add_attribute<Scalar, Vertex::ORBIT>("f2");
		VertexAttribute<Vertex> prev_v2 = map_.add_attribute<Vertex, Vertex::ORBIT>("prev_v2");
		cgogn::dijkstra_compute_paths<Scalar>(map_, weight, {v2}, f2, prev_v2);
		map_.remove_attribute(prev_v2);

		//5 check critical vertices of the intersection of f1 and f2
		fp.extract_intersection(map_, f1, f2, vertex_position_, weight);


		//ETape 2:

		//1 initial function with Feature vertices as seeds
		VertexAttribute<Scalar> min_dist = map_.add_attribute<Scalar, Vertex::ORBIT>("min_dist");
		VertexAttribute<Vertex> min_source = map_.add_attribute<Vertex, Vertex::ORBIT>("min_source");
		cgogn::dijkstra_compute_normalized_paths<Scalar>(map_, weight, fp.vertices_, min_dist, min_source);

		VertexAttribute<Scalar> fI = map_.add_attribute<Scalar, Vertex::ORBIT>("fI");
		map_.foreach_cell([&] (Vertex v)
		{
			fI[v] = 1 - min_dist[v];
		});

		// Show the dual of the Voronoi built from the found extrema
		std::vector<Vertex> vertices_dual;
		std::vector<Vertex> saddles;
		cgogn::extract_critical_points<Scalar>(map_, min_dist, vertices_dual, saddles);
		fp.draw(vertices_dual, vertex_position_, 1.0f, 0.2f, 0.2f, 0.8f);
		fp.draw(saddles, vertex_position_, 1.0f, 0.8f, 0.2f, 0.6f);

		//2. function perturbation

		//sort the vertices of map_ by increasing values of fI
		std::vector<std::pair<float, Vertex>> sorted_v;
		map_.foreach_cell([&] (Vertex v)
		{
			sorted_v.push_back(std::pair<float,Vertex>(fI[v] ,v));
		});

		std::sort(sorted_v.begin(), sorted_v.end(), [] (const std::pair<float, Vertex>& pair1, const std::pair<float, Vertex>& pair2) {
			return pair1.first < pair2.first;
		});

		std::uint32_t nb_v = map_.nb_cells<Vertex::ORBIT>();
		fpo_ = map_.add_attribute<Scalar, Vertex::ORBIT>("fpo");

		for(unsigned int i = 0 ; i < sorted_v.size() ; ++i)
		{
			Vertex vit = sorted_v[i].second;
			fpo_[vit] = cgogn::numerics::float64(i) / cgogn::numerics::float64(nb_v);
		}

		update_color(fpo_);

//		cgogn::io::export_vtp<Vec3>(map_, vertex_position_, fI, "test.vtp");

		map_.remove_attribute(f0);
		map_.remove_attribute(f1);
		map_.remove_attribute(f2);
		map_.remove_attribute(min_dist);
		map_.remove_attribute(min_source);
		map_.remove_attribute(fI);
	}


	void compute_curvature()
	{
		EdgeAttribute<Scalar> edgeangle = map_.add_attribute<Scalar, Edge::ORBIT>("edgeangle");
		EdgeAttribute<Scalar> edgeaera = map_.add_attribute<Scalar, Edge::ORBIT>("edgeaera");

		VertexAttribute<Scalar> kmax = map_.add_attribute<Scalar, Vertex::ORBIT>("kmax");
		VertexAttribute<Scalar> kmin = map_.add_attribute<Scalar, Vertex::ORBIT>("kmin");
		VertexAttribute<Vec3> Kmax = map_.add_attribute<Vec3, Vertex::ORBIT>("Kmax");
		VertexAttribute<Vec3> Kmin = map_.add_attribute<Vec3, Vertex::ORBIT>("Kmin");
		VertexAttribute<Vec3> knormal = map_.add_attribute<Vec3, Vertex::ORBIT>("knormal");


		cgogn::geometry::angle_between_face_normals<Vec3>(map_, vertex_position_, edgeangle);
		cgogn::geometry::incident_faces_area<Vec3>(map_, vertex_position_, edgeaera);

		Scalar meanEdgeLength = cgogn::geometry::mean_edge_length<Vec3>(map_, vertex_position_);

		Scalar radius = Scalar(2.0) * meanEdgeLength;

		cgogn::geometry::curvature_normal_cycles_projected<Vec3>(map_,radius, vertex_position_, vertex_normal_,edgeangle,edgeaera,kmax,kmin,Kmax,Kmin,knormal);


		//compute kmean
		VertexAttribute<Scalar> kmean = map_.add_attribute<Scalar, Vertex::ORBIT>("kmean");

		double min = std::numeric_limits<double>::max();
		double max = std::numeric_limits<double>::min();

		map_.foreach_cell([&](Vertex v)
		{
			kmean[v] = (kmin[v] + kmax[v]) / Scalar(2.0);
			min = std::min(min, kmean[v]);
			max = std::max(max, kmean[v]);
		});

		//compute kgaussian
		VertexAttribute<Scalar> kgaussian = map_.add_attribute<Scalar, Vertex::ORBIT>("kgaussian");

		min = std::numeric_limits<double>::max();
		max = std::numeric_limits<double>::min();

		map_.foreach_cell([&](Vertex v)
		{
			kgaussian[v] = (kmin[v] * kmax[v]);
			min = std::min(min, kgaussian[v]);
			max = std::max(max, kgaussian[v]);
		});

		//compute kindex
		VertexAttribute<Scalar> k1 = map_.add_attribute<Scalar, Vertex::ORBIT>("k1");
		VertexAttribute<Scalar> k2 = map_.add_attribute<Scalar, Vertex::ORBIT>("k2");
		VertexAttribute<Scalar> kI = map_.add_attribute<Scalar, Vertex::ORBIT>("kI");

		map_.foreach_cell([&](Vertex v)
		{
			k1[v] = kmean[v] + std::sqrt(kmean[v] * kmean[v] - kgaussian[v]);
			k2[v] = kmean[v] - std::sqrt(kmean[v] * kmean[v] - kgaussian[v]);

			if(k1[v] == k2[v])
				kI[v] = 0.0;
			else
				kI[v] = (2 / M_PI) * std::atan((k1[v] + k2[v]) / (k1[v] - k2[v]));
		});

		update_color(kI);

		//build a metric to feed dijkstra
		Scalar avg_e(0);
		Scalar avg_ki(0);
		cgogn::numerics::uint32 nbe = 0;

		map_.foreach_cell([&](Edge e)
		{
			avg_e = cgogn::geometry::edge_length<Vec3>(map_,e,vertex_position_);
			avg_ki = kI[Vertex(e.dart)] + kI[Vertex(map_.phi1(e.dart))];
			++nbe;
		});
		avg_e /= nbe;
		avg_ki /= nbe;

		edge_metric_ = map_.add_attribute<Scalar, Edge::ORBIT>("edge_metric");

		map_.foreach_cell([&](Edge e)
		{
			Scalar diffKI = kI[Vertex(e.dart)] - kI[Vertex(map_.phi1(e.dart))];

			Scalar w(0.0);
			if(kI[Vertex(e.dart)] < 0.0 && kI[Vertex(map_.phi1(e.dart))] < 0.0)
				w = 0.05;

			edge_metric_[e] = (cgogn::geometry::edge_length<Vec3>(map_,e,vertex_position_) / avg_e) + (w * (diffKI / avg_ki));
		});

		map_.remove_attribute(edgeangle);
		map_.remove_attribute(edgeaera);
		map_.remove_attribute(kmax);
		map_.remove_attribute(kmin);
		map_.remove_attribute(Kmax);
		map_.remove_attribute(Kmin);
		map_.remove_attribute(knormal);

		map_.remove_attribute(kmean);
		map_.remove_attribute(kgaussian);
		map_.remove_attribute(k1);
		map_.remove_attribute(k2);
		map_.remove_attribute(kI);
		map_.remove_attribute(edge_metric_);
	}
};

//	//PL morse function
//	//vertex_f -> feature vertices
//	CVertexAttribute<Scalar> fI = map_.add_attribute<Scalar, CVertex::ORBIT>("fI");

//	map_.foreach_cell([&](Vertex v)
//	{
//		cgogn::dijkstra_compute_normalized_paths<Scalar>(map_, weight, v, dist, prev);

//		Scalar dist_min = std::numeric_limits<Scalar>::max();

//		//find min in vertices_f
//		for(unsigned int i = 0 ; i < vertices_f.size() ; ++i)
//		{
//			if(dist[Vertex(vertices_f[i])] < dist_min)
//			{
//				dist_min = dist[Vertex(vertices_f[i])];
//			}

//		}

//		fI[v] = 1 - dist_min;
//	});

//	//pertubation of fI
//	CVertexAttribute<Scalar> f = map_.add_attribute<Scalar, CVertex::ORBIT>("f");
//	cgogn::dijkstra_compute_perturbated_function<Scalar>(map_, fI, f);


//	std::vector<cgogn::Dart> vertices_f_morse;
//	//7.1 check critical vertices for f and draw them
//	map_.foreach_cell([&](Vertex v)
//	{
//		cgogn::CriticalVertex i = cgogn::critical_vertex_type<Scalar>(map_,v,fI);
//		if(i.v_ == cgogn::CriticalVertexType::MAXIMUM || i.v_ == cgogn::CriticalVertexType::MINIMUM)
//			vertices_f_morse.push_back(v);
//	});


//	Scalar min = std::numeric_limits<Scalar>::max();
//	Scalar max = std::numeric_limits<Scalar>::min();
//	for(auto& v : fI)
//	{
//		min = std::min(min, v);
//		max = std::max(max, v);
//	}

//	cgogn::rendering::update_vbo(fI, *vbo_color_,[&] (const Scalar& n) -> std::array<float,3>
//	{
//		return cgogn::color_map_blue_green_red(cgogn::scale_to_0_1(n, min, max));
//	});


//	drawer_->new_list();
//	drawer_->color3f(1.0f, 1.0f, 1.0f);
//	drawer_->begin(GL_POINTS);
//	for (std::vector<cgogn::Dart>::iterator it = vertices_f_morse.begin(); it != vertices_f_morse.end(); ++it)
//	{
//		drawer_->vertex3fv(vertex_position_[Vertex(*it)]);
//	}
//	drawer_->end();
//	drawer_->end_list();


#endif // SURFACE_H
