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

#include <core/cmap/cmap2.h>
#include <core/utils/definitions.h>
#include <io/map_import.h>

#include <rendering/map_render.h>

#include <geometry/algos/normal.h>

#include <rendering/shaders/shader_simple_color.h>
#include <rendering/shaders/shader_flat.h>
#include <rendering/shaders/shader_phong.h>
#include <rendering/shaders/shader_vector_per_vertex.h>
#include <rendering/shaders/vbo.h>

#include <geometry/algos/bounding_box.h>
#include <geometry/algos/ear_triangulation.h>

// perso functions
#include <cgogn/geometry/algos/angle2.h>
#include <cgogn/geometry/algos/area2.h>
#include <cgogn/geometry/algos/length2.h>
#include <cgogn/geometry/algos/curvature.h>
#include <cgogn/helper_functions.h>

#include <gui/feature_points.h>


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
	using VertexAttributeHandler = CMap2::VertexAttributeHandler<T>;
	template<typename T>
	using EdgeAttributeHandler = CMap2::EdgeAttributeHandler<T>;

public:
	CMap2 map_;

	VertexAttributeHandler<Vec3> vertex_position_;
	VertexAttributeHandler<Vec3> vertex_normal_;

	EdgeAttributeHandler<Scalar> edge_metric_;

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

public:

	Surface():
		map_(),
		vertex_position_(),
		vertex_normal_(),
		edge_metric_(),
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
	{}

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
	}


	template <typename T, typename MAP>
	inline T surface_centroid(MAP& map, VertexAttributeHandler<T>& attribute)
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


	void init(cgogn::geometry::BoundingBox<Vec3>& bb)
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
				return bb.diag_size()/1000.0 * (1.0 + 2.0*std::abs(n[2]));
		});

		render_ = new cgogn::rendering::MapRender();
		render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS, vertex_position_);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::LINES, vertex_position_);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, vertex_position_);

		shader_point_sprite_ = new cgogn::rendering::ShaderPointSprite(true,true);
		shader_point_sprite_->add_vao();
		shader_point_sprite_->set_vao(0, vbo_pos_,vbo_color_,vbo_sphere_sz_);
		shader_point_sprite_->bind();
		shader_point_sprite_->set_size(bb.diag_size()/100.0);
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
		shader_normal_->set_length(bb.diag_size()/50);
		shader_normal_->release();


		shader_phong_ = new cgogn::rendering::ShaderPhong(true);
		shader_phong_->add_vao();
		shader_phong_->set_vao(0, vbo_pos_, vbo_norm_, vbo_color_);
		shader_phong_->bind();
		//	shader_phong_->set_ambiant_color(QColor(5,5,5));
		//shader_phong_->set_double_side(true);
		//	shader_phong_->set_specular_color(QColor(255,255,255));
		//	shader_phong_->set_specular_coef(10.0);
		shader_phong_->release();
	}

	void update_topology()
	{
		render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS, vertex_position_);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::LINES, vertex_position_);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, vertex_position_);
	}

	void update_color(VertexAttributeHandler<Scalar> scalar, Scalar min, Scalar max)
	{
		cgogn::rendering::update_vbo(scalar, *vbo_color_,[min, max] (const Scalar& n) -> std::array<float,3>
		{
				return cgogn::color_map_blue_green_red(cgogn::scale_to_0_1(n, min, max));
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

	void import(const std::string& filename)
	{
		cgogn::io::import_surface<Vec3>(map_, filename);

		vertex_position_ = map_.get_attribute<Vec3, Vertex::ORBIT>("position");
		vertex_normal_ = map_.add_attribute<Vec3, Vertex::ORBIT>("normal");

		cgogn::geometry::compute_normal_vertices<Vec3>(map_, vertex_position_, vertex_normal_);
	}

	void height_function(FeaturePoints& fp)
	{
		VertexAttributeHandler<Scalar> scalar = map_.add_attribute<Scalar, Vertex::ORBIT>("scalar");

		cgogn::height_pl_function<Vec3>(map_, vertex_position_, scalar);

		double min = std::numeric_limits<double>::max();
		double max = std::numeric_limits<double>::min();
		for(auto& v : scalar)
		{
			min = std::min(min, v);
			max = std::max(max, v);
		}
		update_color(scalar, min, max);

		fp.extract<Vec3>(map_, scalar, vertex_position_);

		map_.remove_attribute(scalar);
	}

	void geodesic_distance_function(FeaturePoints& fp, Vertex d)
	{
		VertexAttributeHandler<Scalar> scalar = map_.add_attribute<Scalar, Vertex::ORBIT>("scalar");

		//init cost function for edges
		EdgeAttributeHandler<Scalar> weight = map_.add_attribute<Scalar, Edge::ORBIT>("weight");
		map_.foreach_cell([&](Edge e)
		{
			weight[e] = cgogn::geometry::edge_length<Vec3>(map_, e, vertex_position_);
		});

		cgogn::geodesic_distance_pl_function<Scalar>(map_, d, weight, scalar);

		double min = std::numeric_limits<double>::max();
		double max = std::numeric_limits<double>::min();
		for(auto& v : scalar)
		{
			min = std::min(min, v);
			max = std::max(max, v);
		}
		update_color(scalar, min, max);

		fp.extract<Vec3>(map_, scalar, vertex_position_);

		map_.remove_attribute(scalar);
		map_.remove_attribute(weight);
	}


	void edge_length_weighted_morse_function(FeaturePoints& fp)
	{

		EdgeAttributeHandler<Scalar> weight = map_.add_attribute<Scalar, Edge::ORBIT>("weight");
		map_.foreach_cell([&](Edge e)
		{
			weight[e] = cgogn::geometry::edge_length<Vec3>(map_, e, vertex_position_);
		});

		morse_function(fp,weight);
	}

	void curvature_weighted_morse_function(FeaturePoints& fp)
	{
		compute_curvature();
		morse_function(fp, edge_metric_);
	}

	void morse_function(FeaturePoints& fp, EdgeAttributeHandler<Scalar>& weight)
	{
		Vec3 barycenter = surface_centroid<Vec3>(map_, vertex_position_);

		//1. compute v0: the vertex whose distance to the barycenter of map is minimal
		Scalar dist_v0 = std::numeric_limits<Scalar>::max();
		Vertex v0;

		map_.foreach_cell([&](Vertex v)
		{
			Vec3 origin = vertex_position_[v];
			origin -= barycenter;
			Scalar dist = origin.norm();

			if(dist < dist_v0)
			{
				dist_v0 = dist;
				v0 = v;
			}
		});

		//2. map the vertices to their geodesic distance to v0: find the vertex v1 that maximizes f0
		VertexAttributeHandler<Scalar> f0 = map_.add_attribute<Scalar, Vertex::ORBIT>("f0");
		VertexAttributeHandler<cgogn::Dart> prev_v0 = map_.add_attribute<cgogn::Dart, Vertex::ORBIT>("prev_v0");
		cgogn::dijkstra_compute_paths<Scalar>(map_, weight, v0, f0, prev_v0);
		Scalar dist_v1 = 0.0;
		Vertex v1;
		map_.foreach_cell([&](Vertex v)
		{
			Scalar dist = f0[v];

			if(dist > dist_v1)
			{
				dist_v1 = dist;
				v1 = v;
			}
		});

		//3. map the vertices to their geodesic distance to v1: find the vertex v2  that maximizes f1
		VertexAttributeHandler<Scalar> f1 = map_.add_attribute<Scalar, Vertex::ORBIT>("f1");
		VertexAttributeHandler<cgogn::Dart> prev_v1 = map_.add_attribute<cgogn::Dart, Vertex::ORBIT>("prev_v1");
		cgogn::dijkstra_compute_paths<Scalar>(map_, weight, v1, f1, prev_v1);
		Scalar dist_v2 = 0.0;
		Vertex v2;
		map_.foreach_cell([&](Vertex v)
		{
			Scalar dist = f1[v];

			if(dist > dist_v2)
			{
				dist_v2 = dist;
				v2 = v;
			}
		});

		//4. map the vertices to their geodesic distance to v2
		VertexAttributeHandler<Scalar> f2 = map_.add_attribute<Scalar, Vertex::ORBIT>("f2");
		VertexAttributeHandler<cgogn::Dart> prev_v2 = map_.add_attribute<cgogn::Dart, Vertex::ORBIT>("prev_v2");
		cgogn::dijkstra_compute_paths<Scalar>(map_, weight, v2, f2, prev_v2);
		Scalar dist_v3 = 0.0;
		map_.foreach_cell([&](Vertex v)
		{
			Scalar dist = f2[v];

			if(dist > dist_v3)
				dist_v3 = dist;
		});

		//5 check critical vertices of the intersection of f1 and f2
		fp.extract_intersection<Vec3>(map_, f1, f2, vertex_position_, weight);

		map_.remove_attribute(weight);
		map_.remove_attribute(f0);
		map_.remove_attribute(prev_v0);
		map_.remove_attribute(f1);
		map_.remove_attribute(prev_v1);
		map_.remove_attribute(f2);
		map_.remove_attribute(prev_v2);

	}


	void compute_curvature()
	{
		EdgeAttributeHandler<Scalar> edgeangle = map_.add_attribute<Scalar, Edge::ORBIT>("edgeangle");
		EdgeAttributeHandler<Scalar> edgeaera = map_.add_attribute<Scalar, Edge::ORBIT>("edgeaera");

		VertexAttributeHandler<Scalar> kmax = map_.add_attribute<Scalar, Vertex::ORBIT>("kmax");
		VertexAttributeHandler<Scalar> kmin = map_.add_attribute<Scalar, Vertex::ORBIT>("kmin");
		VertexAttributeHandler<Vec3> Kmax = map_.add_attribute<Vec3, Vertex::ORBIT>("Kmax");
		VertexAttributeHandler<Vec3> Kmin = map_.add_attribute<Vec3, Vertex::ORBIT>("Kmin");
		VertexAttributeHandler<Vec3> knormal = map_.add_attribute<Vec3, Vertex::ORBIT>("knormal");


		cgogn::geometry::angle_between_face_normals<Vec3>(map_, vertex_position_, edgeangle);
		cgogn::geometry::incident_faces_area<Vec3>(map_, vertex_position_, edgeaera);

		Scalar meanEdgeLength = cgogn::geometry::mean_edge_length<Vec3>(map_, vertex_position_);

		Scalar radius = Scalar(2.0) * meanEdgeLength;

		cgogn::geometry::curvature_normal_cycles_projected<Vec3>(map_,radius, vertex_position_, vertex_normal_,edgeangle,edgeaera,kmax,kmin,Kmax,Kmin,knormal);


		//compute kmean
		VertexAttributeHandler<Scalar> kmean = map_.add_attribute<Scalar, Vertex::ORBIT>("kmean");

		double min = std::numeric_limits<double>::max();
		double max = std::numeric_limits<double>::min();

		map_.foreach_cell([&](Vertex v)
		{
			kmean[v] = (kmin[v] + kmax[v]) / Scalar(2.0);
			min = std::min(min, kmean[v]);
			max = std::max(max, kmean[v]);
		});

		//compute kgaussian
		VertexAttributeHandler<Scalar> kgaussian = map_.add_attribute<Scalar, Vertex::ORBIT>("kgaussian");

		min = std::numeric_limits<double>::max();
		max = std::numeric_limits<double>::min();

		map_.foreach_cell([&](Vertex v)
		{
			kgaussian[v] = (kmin[v] * kmax[v]);
			min = std::min(min, kgaussian[v]);
			max = std::max(max, kgaussian[v]);
		});

		//compute kindex
		VertexAttributeHandler<Scalar> k1 = map_.add_attribute<Scalar, Vertex::ORBIT>("k1");
		VertexAttributeHandler<Scalar> k2 = map_.add_attribute<Scalar, Vertex::ORBIT>("k2");
		VertexAttributeHandler<Scalar> kI = map_.add_attribute<Scalar, Vertex::ORBIT>("kI");

		map_.foreach_cell([&](Vertex v)
		{
			k1[v] = kmean[v] + std::sqrt(kmean[v] * kmean[v] - kgaussian[v]);
			k2[v] = kmean[v] - std::sqrt(kmean[v] * kmean[v] - kgaussian[v]);

			if(k1[v] == k2[v])
				kI[v] = 0.0;
			else
				kI[v] = (2 / M_PI) * std::atan((k1[v] + k2[v]) / (k1[v] - k2[v]));
		});

		min = std::numeric_limits<double>::max();
		max = std::numeric_limits<double>::min();

		map_.foreach_cell([&](Vertex v)
		{
			min = std::min(min, kI[v]);
			max = std::max(max, kI[v]);
		});

		update_color(kI, min, max);

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
	}
};

//	//PL morse function
//	//vertex_f -> feature vertices
//	CVertexAttributeHandler<Scalar> fI = map_.add_attribute<Scalar, CVertex::ORBIT>("fI");

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
//	CVertexAttributeHandler<Scalar> f = map_.add_attribute<Scalar, CVertex::ORBIT>("f");
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
