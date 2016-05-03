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
#include <cgogn/rendering/shaders/shader_point_sprite.h>
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
	using Face = CMap2::Face;
	using Volume = CMap2::Volume;
	template<typename T>
	using VertexAttribute = CMap2::VertexAttribute<T>;
	template<typename T>
	using EdgeAttribute = CMap2::EdgeAttribute<T>;

public:
	CMap2 map_;

	VertexAttribute<Vec3> vertex_position_;
	VertexAttribute<Vec3> vertex_normal_;
	VertexAttribute<Scalar> scalar_field_;


	EdgeAttribute<Scalar> edge_metric_;

	cgogn::geometry::BoundingBox<Vec3> bb_;

	cgogn::rendering::MapRender* render_;

	cgogn::rendering::VBO* vbo_pos_;
	cgogn::rendering::VBO* vbo_norm_;
	cgogn::rendering::VBO* vbo_color_;
	cgogn::rendering::VBO* vbo_sphere_sz_;

	cgogn::rendering::ShaderBoldLine::Param* param_edge_;
	cgogn::rendering::ShaderFlatColor::Param* param_flat_;
	cgogn::rendering::ShaderVectorPerVertex::Param* param_normal_;
	cgogn::rendering::ShaderPhongColor::Param* param_phong_;
	cgogn::rendering::ShaderPointSpriteColorSize::Param* param_point_sprite_;

	cgogn::ReebGraph<Vec3, CMap2>* reeb_graph_;

public:

	Surface():
		map_(),
		vertex_position_(),
		vertex_normal_(),
		scalar_field_(),
		edge_metric_(),
		bb_(),
		render_(nullptr),
		vbo_pos_(nullptr),
		vbo_norm_(nullptr),
		vbo_color_(nullptr),
		vbo_sphere_sz_(nullptr)
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
		cgogn::rendering::update_vbo(vertex_position_, vbo_pos_);

		vbo_norm_ = new cgogn::rendering::VBO(3);
		cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_);

		// fill a color vbo with abs of normals
		vbo_color_ = new cgogn::rendering::VBO(3);
		cgogn::rendering::update_vbo(vertex_normal_, vbo_color_,[] (const Vec3& n) -> std::array<float,3>
		{
			return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
		});

		// fill a sphere size vbo
		vbo_sphere_sz_ = new cgogn::rendering::VBO(1);
		cgogn::rendering::update_vbo(vertex_normal_, vbo_sphere_sz_,[&] (const Vec3& n) -> float
		{
			return bb_.diag_size()/1000.0 * (1.0 + 2.0*std::abs(n[2]));
		});

		render_ = new cgogn::rendering::MapRender();
		render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::LINES);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);

		param_point_sprite_ = cgogn::rendering::ShaderPointSpriteColorSize::generate_param();
		param_point_sprite_->set_all_vbos(vbo_pos_,vbo_color_,vbo_sphere_sz_);

		param_edge_ = cgogn::rendering::ShaderBoldLine::generate_param();
		param_edge_->set_position_vbo(vbo_pos_);
		param_edge_->color_ = QColor(255,255,0);
		param_edge_->width_= 2.5f;

		param_flat_ = cgogn::rendering::ShaderFlatColor::generate_param();
		param_flat_->set_position_vbo(vbo_pos_);
		param_flat_->ambiant_color_ = QColor(5,5,5);

		param_normal_ = cgogn::rendering::ShaderVectorPerVertex::generate_param();
		param_normal_->set_all_vbos(vbo_pos_, vbo_norm_);
		param_normal_->color_ = QColor(200,0,200);
		param_normal_->length_ = bb_.diag_size()/50;

		param_phong_ = cgogn::rendering::ShaderPhongColor::generate_param();
		param_phong_->set_all_vbos(vbo_pos_, vbo_norm_, vbo_color_);
	}

	void update_geometry()
	{
		cgogn::rendering::update_vbo(vertex_position_, vbo_pos_);
		cgogn::geometry::compute_normal_vertices<Vec3>(map_, vertex_position_, vertex_normal_);
		cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_);
	}

	void update_topology()
	{

		render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::LINES);
		render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES);
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

		cgogn::rendering::update_vbo(scalar, vbo_color_,
									 [min, max] (const Scalar& n) -> std::array<float,3>
		{
			return cgogn::color_map_blue_green_red(cgogn::numerics::scale_to_0_1(n, min, max));
//			return cgogn::color_map_hash(cgogn::numerics::scale_to_0_1(n, min, max));
		});
	}

	void draw_phong(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		param_phong_->bind(proj,view);
		render_->draw(cgogn::rendering::TRIANGLES);
		param_phong_->release();
	}

	void draw_flat(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		param_flat_->bind(proj,view);
		render_->draw(cgogn::rendering::TRIANGLES);
		param_flat_->release();
	}

	void draw_vertices(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		param_point_sprite_->bind(proj,view);
		render_->draw(cgogn::rendering::POINTS);
		param_point_sprite_->release();
	}

	void draw_edges(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		param_edge_->bind(proj,view);
		//param_edge_->set_width(2.5f);
		render_->draw(cgogn::rendering::LINES);
		param_edge_->release();

	}

	void draw_normals(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		param_normal_->bind(proj,view);
		render_->draw(cgogn::rendering::POINTS);
		param_normal_->release();
	}

	cgogn::geometry::BoundingBox<Vec3> import(const std::string& filename)
	{
		cgogn::io::import_surface<Vec3>(map_, filename);

		vertex_position_ = map_.get_attribute<Vec3, Vertex::ORBIT>("position");
		vertex_normal_ = map_.add_attribute<Vec3, Vertex::ORBIT>("normal");
		scalar_field_ = map_.add_attribute<Scalar, Vertex::ORBIT>("scalar_field_");
		edge_metric_ = map_.add_attribute<Scalar, Edge::ORBIT>("edge_metric");

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

	Vertex find_source(Vertex v,
					   VertexAttribute<Scalar>& scalar_field,
					   VertexAttribute<Vertex>& path_to_source)
	{
		Vertex u = v;
		while (scalar_field[u] > Scalar(0)) {
			u = path_to_source[u];
		}
		return u;
	}

	Vertex farthest_extremity(const std::vector<Vertex> vertices,
							  VertexAttribute<Scalar>& scalar_field,
							  VertexAttribute<Vertex>& path_to_sources)
	{
		cgogn::dijkstra_compute_paths<Scalar>(map_, edge_metric_, vertices, scalar_field, path_to_sources);

		Scalar max_distance = Scalar(0);
		Vertex extremity = vertices[0];
		map_.foreach_cell([&](Vertex v)
		{
			Scalar distance = scalar_field[v];
			if(distance > max_distance) {
				max_distance = distance;
				extremity = v;
			}
		});

		return extremity;
	}

	// Search a couple of vertices that maximizes their geodesic distance
	// and compute their respective scalar field (geodesic distance to themselves)
	Scalar maximal_diameter(Vertex& v1, VertexAttribute<Scalar>& scalar_field1,
							Vertex& v2, VertexAttribute<Scalar>& scalar_field2,
							VertexAttribute<Vertex>& path_to_sources)
	{
		// Init v1 with a vertex near the center of the mesh
		v1 = central_vertex();

		// Init v2 with the farthest vertice from v1
		v2 = farthest_extremity({v1}, scalar_field1, path_to_sources);

		// Try to optimize these two vertices by maximizing their distance
		Scalar maximal = scalar_field1[v2];
		Scalar current = maximal;
		do {
			v1 = farthest_extremity({v2}, scalar_field2, path_to_sources);
			v2 = farthest_extremity({v1}, scalar_field1, path_to_sources);
			current = maximal;
			maximal = scalar_field1[v2];
		} while (current < maximal);

		return maximal;
	}

	// Remove the vertices whose distance to a source is below the threshold
	// and (if any) add the source of the closest to the filtered set
	void features_filter(std::vector<Vertex>& vertices,
						 std::vector<Vertex>& filtered,
						 Scalar threshold,
						 VertexAttribute<Scalar>& scalar_field,
						 VertexAttribute<Vertex>& path_to_sources)
	{
		if (vertices.empty()) return;
		std::vector<Vertex> vertices_to_keep;
		Vertex min_vertex = vertices.front();
		Scalar min_dist = scalar_field[min_vertex];

		for (Vertex v: vertices) {
			Scalar dist = scalar_field[v];
			// Search the closest vertex
			if (dist < min_dist) {
				min_vertex = v;
				min_dist = dist;
			}
			// Select the vertices whose distance is greater than the threshold
			if (dist > threshold) vertices_to_keep.push_back(v);
		}
		// A vertex that is near a source has been found
		if (min_dist <= threshold)
		{
			Vertex source = find_source(min_vertex, scalar_field, path_to_sources);
			filtered.push_back(source);
		}
		// Replace the initial vertices by the filtered ones
		vertices.clear();
		vertices.swap(vertices_to_keep);
	}

	// Remove from v1 and v2 their common vertices and put them in intersection
	void intersection(std::vector<Vertex>& v1,
					  std::vector<Vertex>& v2,
					  std::vector<Vertex>& intersection)
	{
		std::vector<Vertex> v1_not_in_v2;
		std::vector<Vertex> v2_not_in_v1;
		intersection.clear();

		for (Vertex v: v1)
		{
			bool found = false;
			for (Vertex u: v2)
			{
				if (u.dart == v.dart) {
					found = true;
					intersection.push_back(u);
				} else
				{
					v2_not_in_v1.push_back(u);
				}

			}
			v2.clear();
			v2.swap(v2_not_in_v1);
			if (!found) v1_not_in_v2.push_back(v);
		}

		v1.clear();
		v1.swap(v1_not_in_v2);
	}

	//	Find features in the scalar field
	void find_features(FeaturePoints<VEC3>& fp,
					   VertexAttribute<Scalar>& scalar_field)
	{
		VertexAttribute<Vertex> path_to_sources = map_.add_attribute<Vertex, Vertex::ORBIT>("path_to_filter");

		// Get the two farthest vertices v1 and v2 (making a maximal diameter)
		// and their scalar field f1 and f2
		Vertex v1;
		Vertex v2;
		VertexAttribute<Scalar> f1 = map_.add_attribute<Scalar, Vertex::ORBIT>("f1");
		VertexAttribute<Scalar> f2 = map_.add_attribute<Scalar, Vertex::ORBIT>("f2");
		Scalar max_distance = maximal_diameter(v1, f1, v2, f2, path_to_sources);
		Scalar filter_distance = max_distance / Scalar(4);

		// Get the maxima in the scalar field f1
		std::vector<Vertex> vertices_f1;
		cgogn::extract_maxima<Scalar>(map_, f1, vertices_f1);
		std::cout << "F1 size: " << vertices_f1.size() << std::endl;

		// Get the maxima in the scalar field f2
		std::vector<Vertex> vertices_f2;
		cgogn::extract_maxima<Scalar>(map_, f2, vertices_f2);
		std::cout << "F2 size: " << vertices_f2.size() << std::endl;

		// Build a scalar field from {v1, v2}
		fp.vertices_.push_back(v1);
		fp.vertices_.push_back(v2);
		cgogn::dijkstra_compute_paths<Scalar>(map_, edge_metric_, fp.vertices_, scalar_field, path_to_sources);

		// Initialize the sets of vertices filtered from f1 and f2 with v1 and v2
		std::vector<Vertex> vertices_f1_filtered;
		std::vector<Vertex> vertices_f2_filtered;
		vertices_f1_filtered.push_back(v1);
		vertices_f2_filtered.push_back(v2);

		// Filter the maxima of f1 and f2 against this first scalar field
		features_filter(vertices_f1, vertices_f1_filtered, filter_distance, scalar_field, path_to_sources);
		features_filter(vertices_f2, vertices_f2_filtered, filter_distance, scalar_field, path_to_sources);

		// Iteratively add a source in the scalar field and filter f1 and f2 with this new vertex
		// The added vertex if the farthest from all sources that are present in the scalar field
		// The loop ends when the distance to the last added source is smaller
		// than 1/4 of the computed maximal diameter
		Scalar target_distance = filter_distance;
		Scalar actual_distance = max_distance;
		std::cout << "Distances: (" << max_distance << ") 1.0 ";
		while (target_distance < actual_distance && !(vertices_f1.empty() && vertices_f2.empty())) {
			Vertex v = farthest_extremity(fp.vertices_, scalar_field, path_to_sources);
			fp.vertices_.push_back(v);
			actual_distance = scalar_field[v];
			filter_distance = std::min(filter_distance, target_distance / Scalar(3));
			std::cout << actual_distance/max_distance << " ";
			cgogn::dijkstra_compute_paths<Scalar>(map_, edge_metric_, fp.vertices_, scalar_field, path_to_sources);
			features_filter(vertices_f1, vertices_f1_filtered, filter_distance, scalar_field, path_to_sources);
			features_filter(vertices_f2, vertices_f2_filtered, filter_distance, scalar_field, path_to_sources);
		}
		std::cout << std::endl;

		intersection(vertices_f1_filtered, vertices_f2_filtered, fp.vertices_);
		std::cout << "Selected features: " << fp.vertices_.size() << std::endl;

		// Draw the unselected extrema in f1 and f2
		fp.draw(vertices_f1, vertex_position_, 0.3f, 0.3f, 1.0f, 0.6f, 1);
		std::cout << "F1 not filtered features: " << vertices_f1.size() << std::endl;
		fp.draw(vertices_f2, vertex_position_, 0.3f, 1.0f, 0.3f, 0.6f, 2);
		std::cout << "F2 not filtered features: " << vertices_f2.size() << std::endl;

		// Draw the selected features that are not in the intersection
		fp.draw(vertices_f1_filtered, vertex_position_, 1.0f, 0.2f, 1.0f, 0.8f, 1);
		std::cout << "F1 not in F2 features: " << vertices_f1_filtered.size() << std::endl;
		fp.draw(vertices_f2_filtered, vertex_position_, 1.0f, 0.4f, 0.4f, 0.8f, 2);
		std::cout << "F2 not in F1 features: " << vertices_f2_filtered.size() << std::endl;

		map_.remove_attribute(path_to_sources);
		map_.remove_attribute(f1);
		map_.remove_attribute(f2);
	}

	void height_function(FeaturePoints<Vec3>& fp)
	{
		cgogn::height_pl_function<Vec3>(map_, vertex_position_, scalar_field_);

		update_color(scalar_field_);
		fp.extract(map_, scalar_field_, vertex_position_);

		//		reeb_graph_->compute(scalar_field_);
	}

	void distance_to_center_function(FeaturePoints<Vec3>& fp)
	{
		compute_length(edge_metric_);

		Vertex v0 = central_vertex();
		cgogn::geodesic_distance_pl_function<Scalar>(map_, {v0}, edge_metric_, scalar_field_);

		update_color(scalar_field_);
		fp.extract(map_, scalar_field_, vertex_position_);

		//		reeb_graph_->compute(scalar_field_);
	}

	void edge_length_weighted_geodesic_distance_function(FeaturePoints<VEC3>& fp)
	{
		// Find features for the edge_metric
		compute_length(edge_metric_);
		find_features(fp, scalar_field_);

		// Build the scalar field from the selected features
		cgogn::geodesic_distance_pl_function<Scalar>(map_, fp.vertices_, edge_metric_, scalar_field_);

		// Draw the result
		update_color(scalar_field_);
		fp.extract(map_, scalar_field_, vertex_position_);

	}

	void curvature_weighted_geodesic_distance_function(FeaturePoints<VEC3>& fp)
	{
		// Find features for the edge_metric
		compute_curvature(edge_metric_);
		find_features(fp, scalar_field_);

		// Build the scalar field from the selected features
		cgogn::geodesic_distance_pl_function<Scalar>(map_, fp.vertices_, edge_metric_, scalar_field_);

		// Draw the result
		update_color(scalar_field_);
		fp.extract(map_, scalar_field_, vertex_position_);

	}

	void edge_length_weighted_morse_function(FeaturePoints<VEC3>& fp)
	{
		// Find features for the edge_metric
		compute_length(edge_metric_);

		// Compute a morse function with the metric
		morse_function(fp, scalar_field_, edge_metric_);

		// Draw the morse function and its critical points
		update_color(scalar_field_);

		std::vector<Vertex> maxima;
		std::vector<Vertex> minima;
		std::vector<Vertex> saddles;
		cgogn::extract_critical_points<Scalar>(map_, scalar_field_, maxima, minima, saddles);
		fp.draw(maxima, vertex_position_, 1.0f, 1.0f, 1.0f, 1.0f);
		fp.draw(minima, vertex_position_, 1.0f, 0.0f, 0.0f, 0.8f);
		fp.draw(saddles, vertex_position_, 1.0f, 1.0f, 0.0f, 0.8f);
	}

	void curvature_weighted_morse_function(FeaturePoints<VEC3>& fp)
	{
		// Find features for the edge_metric
		compute_curvature(edge_metric_);

		// Compute a morse function with the metric
		morse_function(fp, scalar_field_, edge_metric_);

		// Draw the morse function and its critical points
		update_color(scalar_field_);

		std::vector<Vertex> maxima;
		std::vector<Vertex> minima;
		std::vector<Vertex> saddles;
		cgogn::extract_critical_points<Scalar>(map_, scalar_field_, maxima, minima, saddles);
		fp.draw(maxima, vertex_position_, 1.0f, 1.0f, 1.0f, 1.0f);
		fp.draw(minima, vertex_position_, 1.0f, 0.0f, 0.0f, 0.8f);
		fp.draw(saddles, vertex_position_, 1.0f, 1.0f, 0.0f, 0.8f);
	}

	void show_level_sets(const VertexAttribute<Scalar>& scalar_field)
	{
		VertexAttribute<Scalar> level_sets = map_.add_attribute<Scalar, Vertex::ORBIT>("level_sets");
		cgogn::extract_level_sets<Scalar>(map_ , scalar_field, level_sets);

		// Draw the level sets
		update_color(level_sets);
		map_.remove_attribute(level_sets);
	}

	void morse_function(FeaturePoints<VEC3>& fp,
						VertexAttribute<Scalar>& scalar_field,
						const EdgeAttribute<Scalar>& edge_metric)
	{
		// Etape 1 get the two farthest vertices and their associated scalar field
		// (the scalar field contains the geodesic distance to the nearest feature)
		VertexAttribute<Scalar> dist_to_feature = map_.add_attribute<Scalar, Vertex::ORBIT>("dist_to_feature");
		find_features(fp, dist_to_feature);

		// Etape 2
		// 1 initial function with Feature vertices as seeds
		cgogn::normalized_geodesic_distance_pl_function<Scalar>(map_, fp.vertices_, edge_metric, dist_to_feature);

		// 2 inverse the normalized distance so that the maxima are on the features
		map_.foreach_cell([&] (Vertex v)
		{
			dist_to_feature[v] = 1 - dist_to_feature[v];
		});

		// 3 function perturbation (to remove extra minima and saddles)
		// Run dijkstra using dist_to_feature in place of estimated geodesic distances
		cgogn::dijkstra_to_morse_function<Scalar>(map_, dist_to_feature, scalar_field);

		map_.remove_attribute(dist_to_feature);
	}

	void compute_length(EdgeAttribute<Scalar>& length)
	{
		map_.foreach_cell([&](Edge e)
		{
			length[e] = cgogn::geometry::edge_length<Vec3>(map_, e, vertex_position_);
		});
	}

	void compute_curvature(EdgeAttribute<Scalar>& edge_metric)
	{
		EdgeAttribute<Scalar> length = map_.add_attribute<Scalar, Edge::ORBIT>("lenght");
		EdgeAttribute<Scalar> edgeangle = map_.add_attribute<Scalar, Edge::ORBIT>("edgeangle");
		EdgeAttribute<Scalar> edgeaera = map_.add_attribute<Scalar, Edge::ORBIT>("edgeaera");

		VertexAttribute<Scalar> kmax = map_.add_attribute<Scalar, Vertex::ORBIT>("kmax");
		VertexAttribute<Scalar> kmin = map_.add_attribute<Scalar, Vertex::ORBIT>("kmin");
		VertexAttribute<Vec3> Kmax = map_.add_attribute<Vec3, Vertex::ORBIT>("Kmax");
		VertexAttribute<Vec3> Kmin = map_.add_attribute<Vec3, Vertex::ORBIT>("Kmin");
		VertexAttribute<Vec3> knormal = map_.add_attribute<Vec3, Vertex::ORBIT>("knormal");

		compute_length(length);

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
			avg_e = length[e];
			avg_ki = kI[Vertex(e.dart)] + kI[Vertex(map_.phi1(e.dart))];
			++nbe;
		});
		avg_e /= nbe;
		avg_ki /= nbe;

		map_.foreach_cell([&](Edge e)
		{
			Scalar diffKI = kI[Vertex(e.dart)] - kI[Vertex(map_.phi1(e.dart))];

			Scalar w(0.0);
			if(kI[Vertex(e.dart)] < 0.0 && kI[Vertex(map_.phi1(e.dart))] < 0.0)
				w = 0.05;

			edge_metric[e] = (length[e] / avg_e) + (w * (diffKI / avg_ki));
		});

		map_.remove_attribute(length);
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
