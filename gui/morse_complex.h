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


#ifndef MORSE_COMPLEX_H
#define MORSE_COMPLEX_H

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/utils/definitions.h>
#include <cgogn/io/map_import.h>
#include <cgogn/io/map_export.h>

#include <cgogn/rendering/map_render.h>

#include <cgogn/rendering/drawer.h>
#include <cgogn/rendering/volume_drawer.h>

#include <cgogn/rendering/shaders/shader_simple_color.h>
#include <cgogn/rendering/shaders/shader_flat.h>
#include <cgogn/rendering/shaders/shader_phong.h>
#include <cgogn/rendering/shaders/shader_point_sprite.h>
#include <cgogn/rendering/shaders/shader_vector_per_vertex.h>
#include <cgogn/rendering/shaders/vbo.h>

#include <cgogn/geometry/algos/normal.h>

#include <cgogn/geometry/algos/angle.h>
#include <cgogn/geometry/algos/area.h>
#include <cgogn/geometry/algos/length.h>
#include <cgogn/geometry/algos/bounding_box.h>
#include <cgogn/geometry/algos/ear_triangulation.h>

#include <cgogn/core/utils/numerics.h>

#include <cgogn/geometry/algos/curvature.h>
#include <cgogn/helper_functions.h>

#include <gui/feature_points.h>

#include <cgogn/differential_topology/reeb_graph.h>

template <typename VEC3, typename MAP>
class MorseSmallComplex
{
public:
	using Vec3 = VEC3;
	using Scalar = typename Vec3::Scalar;

	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;
	using Volume = typename MAP::Volume;
	template<typename T>
	using VertexAttribute = typename MAP::template VertexAttribute<T>;
	template<typename T>
	using EdgeAttribute = typename MAP::template EdgeAttribute<T>;

public:
	MAP map_;
	static const unsigned int dimension_ = MAP::DIMENSION;

	VertexAttribute<Vec3> vertex_position_;
	VertexAttribute<Vec3> vertex_normal_;
	VertexAttribute<Vec3> vertex_color_;
	VertexAttribute<Scalar> scalar_field_;

	EdgeAttribute<Scalar> edge_metric_;

	cgogn::geometry::AABB<Vec3> bb_;

	QOpenGLFunctions_3_3_Core* ogl33_;

	std::unique_ptr<cgogn::rendering::VolumeDrawerColor> volume_drawer_;
	std::unique_ptr<cgogn::rendering::VolumeDrawerColor::Renderer> volume_renderer_;

	std::unique_ptr<cgogn::rendering::MapRender> surface_render_;

	std::unique_ptr<cgogn::rendering::VBO> vbo_pos_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_norm_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_color_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_sphere_sz_;

	std::unique_ptr<cgogn::rendering::ShaderBoldLine::Param> param_edge_;
	std::unique_ptr<cgogn::rendering::ShaderFlatColor::Param> param_flat_;
	std::unique_ptr<cgogn::rendering::ShaderVectorPerVertex::Param> param_normal_;
	std::unique_ptr<cgogn::rendering::ShaderPhongColor::Param> param_phong_;
	std::unique_ptr<cgogn::rendering::ShaderPointSpriteColorSize::Param> param_point_sprite_;

public:

	MorseSmallComplex(QOpenGLFunctions_3_3_Core* ogl33):
		map_(),
		vertex_position_(),
		vertex_normal_(),
		vertex_color_(),
		scalar_field_(),
		edge_metric_(),
		bb_(),
		ogl33_(ogl33),
		volume_drawer_(nullptr),
		volume_renderer_(nullptr),
		surface_render_(nullptr),
		vbo_pos_(nullptr),
		vbo_norm_(nullptr),
		vbo_color_(nullptr),
		vbo_sphere_sz_(nullptr)
	{
	}

	~MorseSmallComplex()
	{
		volume_drawer_.reset();
		surface_render_.reset();
		vbo_pos_.reset();
		vbo_norm_.reset();
		vbo_color_.reset();
		vbo_sphere_sz_.reset();
	}

	void init()
	{
		if (dimension_ == 2)
		{
			vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
			cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());

			vbo_norm_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
			cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_.get());

			// fill a color vbo with abs of normals
			vbo_color_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
			cgogn::rendering::update_vbo(vertex_normal_, vbo_color_.get(),[] (const Vec3& n) -> std::array<float,3>
			{
				return {float(std::abs(n[0])), float(std::abs(n[1])), float(std::abs(n[2]))};
			});

			// fill a sphere size vbo
			vbo_sphere_sz_ = cgogn::make_unique<cgogn::rendering::VBO>(1);
			cgogn::rendering::update_vbo(vertex_normal_, vbo_sphere_sz_.get(),[&] (const Vec3& n) -> float
			{
				return bb_.diag_size()/1000.0 * (1.0 + 2.0*std::abs(n[2]));
			});

			surface_render_ = cgogn::make_unique<cgogn::rendering::MapRender>();
			surface_render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS);
			surface_render_->init_primitives<Vec3>(map_, cgogn::rendering::LINES);
			surface_render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);

			param_point_sprite_ = cgogn::rendering::ShaderPointSpriteColorSize::generate_param();
			param_point_sprite_->set_all_vbos(vbo_pos_.get(),vbo_color_.get(),vbo_sphere_sz_.get());

			param_edge_ = cgogn::rendering::ShaderBoldLine::generate_param();
			param_edge_->set_position_vbo(vbo_pos_.get());
			param_edge_->color_ = QColor(255,255,0);
			param_edge_->width_= 2.5f;

			param_flat_ = cgogn::rendering::ShaderFlatColor::generate_param();
			param_flat_->set_position_vbo(vbo_pos_.get());
			param_flat_->ambiant_color_ = QColor(5,5,5);

			param_normal_ = cgogn::rendering::ShaderVectorPerVertex::generate_param();
			param_normal_->set_all_vbos(vbo_pos_.get(), vbo_norm_.get());
			param_normal_->color_ = QColor(200,0,200);
			param_normal_->length_ = bb_.diag_size()/50;

			param_phong_ = cgogn::rendering::ShaderPhongColor::generate_param();
			param_phong_->set_all_vbos(vbo_pos_.get(), vbo_norm_.get(), vbo_color_.get());
		}
		else if (dimension_ == 3u)
		{
			volume_drawer_ = cgogn::make_unique<cgogn::rendering::VolumeDrawerColor>();
			volume_drawer_->update_face<Vec3>(map_, vertex_position_, vertex_color_);
			volume_drawer_->update_edge<Vec3>(map_, vertex_position_);
			volume_renderer_ = volume_drawer_->generate_renderer();
		}
		else
			cgogn_log_warning("draw_flat") << "invalid dimension";
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 2>::type* = nullptr>
	void update_geometry_concrete()
	{
		cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());
		cgogn::geometry::compute_normal_vertices<Vec3, MAP>(map_, vertex_position_, vertex_normal_);
		cgogn::rendering::update_vbo(vertex_normal_, vbo_norm_.get());
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 3>::type* = nullptr>
	void update_geometry_concrete()
	{
		volume_drawer_->update_face<Vec3>(map_, vertex_position_, vertex_color_);
		volume_drawer_->update_edge<Vec3>(map_, vertex_position_);
	}

	void update_geometry()
	{
		update_geometry_concrete<MAP>();
	}

	void update_topology()
	{
		if (dimension_ == 2u)
		{
			surface_render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS);
			surface_render_->init_primitives<Vec3>(map_, cgogn::rendering::LINES);
			surface_render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES);
		}
		else if (dimension_ == 3u)
		{
			volume_drawer_->update_face<Vec3>(map_, vertex_position_, vertex_color_);
			volume_drawer_->update_edge<Vec3>(map_, vertex_position_);
		}
		else
			cgogn_log_warning("draw_flat") << "invalid dimension";
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

		if (dimension_ == 2u)
		{
			cgogn::rendering::update_vbo(scalar, vbo_color_.get(),
										 [min, max] (const Scalar& n) -> std::array<float,3>
			{
				return cgogn::color_map_blue_green_red(cgogn::numerics::scale_to_0_1(n, min, max));
				// return cgogn::color_map_hash(cgogn::numerics::scale_to_0_1(n, min, max));
			});
		}
		else if (dimension_ == 3u)
		{
			map_.foreach_cell([&](Vertex v)
			{
				std::array<float,3> color = cgogn::color_map_blue_green_red(
							cgogn::numerics::scale_to_0_1(scalar[v], min, max));
				vertex_color_[v] = Vec3(color[0], color[1], color[2]);
			});
			volume_drawer_->update_face<Vec3>(map_, vertex_position_, vertex_color_);
		}
		else
			cgogn_log_warning("draw_flat") << "invalid dimension";
	}

	void draw(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		if (dimension_ == 2u)
		{
			param_phong_->bind(proj,view);
			surface_render_->draw(cgogn::rendering::TRIANGLES);
			param_phong_->release();
		}
		else if (dimension_ == 3u)
		{
			volume_renderer_->set_explode_volume(0.8f);
			volume_renderer_->draw_faces(proj, view, ogl33_);
		}
		else
			cgogn_log_warning("draw_flat") << "invalid dimension";
	}

	void draw_flat(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		if (dimension_ == 2u)
		{
			param_flat_->bind(proj,view);
			surface_render_->draw(cgogn::rendering::TRIANGLES);
			param_flat_->release();
		}
		else if (dimension_ == 3u)
		{
			volume_renderer_->set_explode_volume(1.0f);
			volume_renderer_->draw_faces(proj, view, ogl33_);
		}
		else
			cgogn_log_warning("draw_flat") << "invalid dimension";
	}

	void draw_edges(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		if (dimension_ == 2u)
		{
			param_edge_->bind(proj,view);
			//param_edge_->set_width(2.5f);
			surface_render_->draw(cgogn::rendering::LINES);
			param_edge_->release();
		}
		else if (dimension_ == 3u)
		{
			volume_renderer_->draw_edges(proj, view, ogl33_);
		}
		else
			cgogn_log_warning("draw_flat") << "invalid dimension";
	}

	void draw_vertices(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		if (dimension_ == 2u)
		{
			param_point_sprite_->bind(proj,view);
			surface_render_->draw(cgogn::rendering::POINTS);
			param_point_sprite_->release();
			}
		else if (dimension_ == 3u)
		{
		}
		else
			cgogn_log_warning("draw_flat") << "invalid dimension";
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 2>::type* = nullptr>
	void import_concrete(const std::string& filename)
	{
		cgogn::io::import_surface<Vec3>(map_, filename);
		vertex_position_ = map_.template get_attribute<Vec3, Vertex::ORBIT>("position");
		if (!vertex_position_.is_valid())
		{
			cgogn_log_error("import") << "Missing attribute position. Aborting.";
			std::exit(EXIT_FAILURE);
		}
		vertex_normal_ = map_.template add_attribute<Vec3, Vertex::ORBIT>("normal");

		cgogn::geometry::compute_normal_vertices<Vec3>(map_, vertex_position_, vertex_normal_);
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 3>::type* = nullptr>
	void import_concrete(const std::string& filename)
	{
		cgogn::io::import_volume<Vec3>(map_, filename);
		vertex_position_ = map_.template get_attribute<Vec3, Vertex::ORBIT>("position");
		vertex_color_ = map_.template add_attribute<Vec3, Vertex::ORBIT>("color");

		map_.foreach_cell([&](Vertex v)
		{
			vertex_color_[v] = vertex_position_[v];
		});
	}

	void import(const std::string& filename)
	{
		import_concrete<MAP>(filename);

		scalar_field_ = map_.template add_attribute<Scalar, Vertex::ORBIT>("scalar_field_");
		edge_metric_ = map_.template add_attribute<Scalar, Edge::ORBIT>("edge_metric");
	}

	// Search a couple of vertices that maximizes their geodesic distance
	// and compute their respective scalar field (geodesic distance to themselves)
	Scalar maximal_diameter(Vertex& v1, VertexAttribute<Scalar>& scalar_field1,
							Vertex& v2, VertexAttribute<Scalar>& scalar_field2,
							VertexAttribute<Vertex>& path_to_sources)
	{
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, edge_metric_);
		Scalar max_distance;

		// Init v1 with a vertex near the center of the mesh
		v1 = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);

		// Init v2 with the farthest vertice from v1
		distance_field.dijkstra_compute_paths({v1}, scalar_field1, path_to_sources);
		v2 = distance_field.find_maximum(scalar_field1);
		max_distance = scalar_field1[v2];

		// Try to optimize these two vertices by maximizing their distance
		Scalar current_distance = max_distance;
		do {
			current_distance = max_distance;

			distance_field.dijkstra_compute_paths({v2}, scalar_field2, path_to_sources);
			v1 = distance_field.find_maximum(scalar_field2);

			distance_field.dijkstra_compute_paths({v1}, scalar_field1, path_to_sources);
			v2 = distance_field.find_maximum(scalar_field1);
			max_distance = scalar_field1[v2];
		} while (current_distance < max_distance);

		return max_distance;
	}

	// Remove the vertices whose distance to local minima (their sources)
	// of the given scalar field is below the threshold
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
			Vertex source = min_vertex;
			while (path_to_sources[source].dart != source.dart)
				source = path_to_sources[source];
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
					   VertexAttribute<Scalar>& scalar_field,
					   std::vector<Vertex>& features)
	{
		VertexAttribute<Vertex> path_to_sources = map_.template add_attribute<Vertex, Vertex::ORBIT>("path_to_filter");

		// Get the two farthest vertices v1 and v2 (making a maximal diameter)
		// and their scalar field f1 and f2
		Vertex v1;
		Vertex v2;
		VertexAttribute<Scalar> f1 = map_.template add_attribute<Scalar, Vertex::ORBIT>("f1");
		VertexAttribute<Scalar> f2 = map_.template add_attribute<Scalar, Vertex::ORBIT>("f2");
		Scalar max_distance = maximal_diameter(v1, f1, v2, f2, path_to_sources);
		Scalar filter_distance = max_distance / Scalar(4);

		// Get the maxima in the scalar field f1
		std::vector<Vertex> vertices_f1;
		cgogn::topology::extract_maxima<Scalar>(map_, f1, vertices_f1);
		// std::cout << "F1 size: " << vertices_f1.size() << std::endl;

		// Get the maxima in the scalar field f2
		std::vector<Vertex> vertices_f2;
		cgogn::topology::extract_maxima<Scalar>(map_, f2, vertices_f2);
		// std::cout << "F2 size: " << vertices_f2.size() << std::endl;

		// Build a scalar field from {v1, v2}
		features.push_back(v1);
		features.push_back(v2);
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, edge_metric_);
		distance_field.dijkstra_compute_paths(features, scalar_field, path_to_sources);

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
		// std::cout << "Distances: (" << max_distance << ") 1.0 ";
		while (target_distance < actual_distance && !(vertices_f1.empty() && vertices_f2.empty())) {
			distance_field.dijkstra_compute_paths(features, scalar_field, path_to_sources);
			Vertex v = distance_field.find_maximum(scalar_field);
			features.push_back(v);
			actual_distance = scalar_field[v];
			filter_distance = std::min(filter_distance, target_distance / Scalar(3));
			// std::cout << actual_distance/max_distance << " ";
			distance_field.dijkstra_compute_paths(features, scalar_field, path_to_sources);
			features_filter(vertices_f1, vertices_f1_filtered, filter_distance, scalar_field, path_to_sources);
			features_filter(vertices_f2, vertices_f2_filtered, filter_distance, scalar_field, path_to_sources);
		}
		// std::cout << std::endl;

		intersection(vertices_f1_filtered, vertices_f2_filtered, features);
		std::cout << "Selected features: " << features.size() << std::endl;

		// Draw the unselected extrema in f1 and f2
		// fp.draw_vertices(vertices_f1, vertex_position_, 0.3f, 0.3f, 1.0f, 0.6f, 1);
		// std::cout << "F1 not filtered features: " << vertices_f1.size() << std::endl;
		// fp.draw_vertices(vertices_f2, vertex_position_, 0.3f, 1.0f, 0.3f, 0.6f, 2);
		// std::cout << "F2 not filtered features: " << vertices_f2.size() << std::endl;

		// Draw the selected features that are not in the intersection
		// fp.draw_vertices(vertices_f1_filtered, vertex_position_, 1.0f, 0.2f, 1.0f, 0.8f, 1);
		// std::cout << "F1 not in F2 features: " << vertices_f1_filtered.size() << std::endl;
		// fp.draw_vertices(vertices_f2_filtered, vertex_position_, 1.0f, 0.4f, 0.4f, 0.8f, 2);
		// std::cout << "F2 not in F1 features: " << vertices_f2_filtered.size() << std::endl;

		map_.remove_attribute(path_to_sources);
		map_.remove_attribute(f1);
		map_.remove_attribute(f2);
	}

	void height_function(FeaturePoints<Vec3>& fp)
	{
		cgogn::topology::height_pl_function<Vec3>(map_, vertex_position_, scalar_field_);

		update_color(scalar_field_);
		fp.draw_critical_points(map_, scalar_field_, vertex_position_);
	}

	void distance_to_boundary_function(FeaturePoints<Vec3>& fp)
	{
		compute_length(edge_metric_);

		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_,edge_metric_);
		distance_field.distance_to_boundary(scalar_field_);
		update_color(scalar_field_);

		fp.draw_critical_points(map_, scalar_field_, vertex_position_);

		std::vector<Edge> ascending_1_manifold;
		cgogn::topology::extract_ascending_manifold<Scalar>(map_ , scalar_field_, ascending_1_manifold);
		fp.draw_edges(map_, ascending_1_manifold, vertex_position_, 1.0f, 0.5f, 0.0f);

		std::vector<Edge> descending_1_manifold;
		cgogn::topology::extract_descending_manifold<Scalar>(map_ , scalar_field_, descending_1_manifold);
		fp.draw_edges(map_, descending_1_manifold, vertex_position_, 0.5f, 0.5f, 1.0f);
	}

	void distance_to_center_function(FeaturePoints<Vec3>& fp)
	{
		compute_length(edge_metric_);

		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_,edge_metric_);
		distance_field.distance_to_center(vertex_position_, scalar_field_);

		update_color(scalar_field_);
		fp.draw_critical_points(map_, scalar_field_, vertex_position_);
	}

	void edge_length_weighted_geodesic_distance_function(FeaturePoints<VEC3>& fp)
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_length(edge_metric_);
		find_features(fp, scalar_field_, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_,edge_metric_);
		distance_field.dijkstra_compute_distances(features, scalar_field_);

		for (auto& s : scalar_field_) s = Scalar(1) - s;

		// Draw the result
		update_color(scalar_field_);
		fp.draw_critical_points(map_, scalar_field_, vertex_position_);
	}

	void curvature_weighted_geodesic_distance_function(FeaturePoints<VEC3>& fp)
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_length(edge_metric_);
		find_features(fp, scalar_field_, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_);
		distance_field.dijkstra_compute_distances(features, scalar_field_);

		for (auto& s : scalar_field_) s = Scalar(1) - s;

		// Draw the result
		update_color(scalar_field_);
		fp.draw_critical_points(map_, scalar_field_, vertex_position_);

	}

	void edge_length_weighted_morse_function(FeaturePoints<VEC3>& fp)
	{
		// Find features for the edge_metric
		compute_length(edge_metric_);

		// Compute a morse function with the metric
		morse_function(fp, scalar_field_, edge_metric_);

		// Draw the morse function and its critical points
		update_color(scalar_field_);

		fp.draw_critical_points(map_, scalar_field_, vertex_position_);

		std::vector<Edge> ascending_1_manifold;
		cgogn::topology::extract_ascending_manifold<Scalar>(map_ , scalar_field_, ascending_1_manifold);
		fp.draw_edges(map_, ascending_1_manifold, vertex_position_, 1.0f, 0.5f, 0.0f);

		std::vector<Edge> descending_1_manifold;
		cgogn::topology::extract_descending_manifold<Scalar>(map_ , scalar_field_, descending_1_manifold);
		fp.draw_edges(map_, descending_1_manifold, vertex_position_, 0.5f, 0.5f, 1.0f);
	}

	void curvature_weighted_morse_function(FeaturePoints<VEC3>& fp)
	{
		// Find features for the edge_metric
		compute_length(edge_metric_);

		// Compute a morse function with the metric
		morse_function(fp, scalar_field_, edge_metric_);

		// Draw the morse function and its critical points
		update_color(scalar_field_);

		fp.draw_critical_points(map_, scalar_field_, vertex_position_);
	}

	void show_level_sets(FeaturePoints<VEC3>& fp,
						 const VertexAttribute<Scalar>& scalar_field)
	{
		std::vector<Edge> level_lines;
		cgogn::topology::extract_level_sets<Scalar>(map_ , scalar_field, level_lines);
		update_color(scalar_field);
		fp.draw_edges(map_, level_lines, vertex_position_, 1.0f, 1.0f, 1.0f);

		fp.draw_critical_points(map_, scalar_field, vertex_position_);
	}

	void morse_function(FeaturePoints<VEC3>& fp,
						VertexAttribute<Scalar>& scalar_field,
						const EdgeAttribute<Scalar>& edge_metric)
	{
		// Etape 1 get the two farthest vertices and their associated scalar field
		// (the scalar field contains the geodesic distance to the nearest feature)
		VertexAttribute<Scalar> dist_to_feature = map_.template add_attribute<Scalar, Vertex::ORBIT>("dist_to_feature");
		std::vector<Vertex> features;
		find_features(fp, dist_to_feature, features);

		// Etape 2
		// Run dijkstra using dist_to_feature in place of estimated geodesic distances
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, edge_metric);
		distance_field.dijkstra_to_morse_function(features, scalar_field);

		map_.remove_attribute(dist_to_feature);
	}

	void compute_length(EdgeAttribute<Scalar>& length)
	{
		map_.foreach_cell([&](Edge e)
		{
			length[e] = cgogn::geometry::edge_length<Vec3>(map_, e, vertex_position_);
		});
	}

};

#endif // MORSE_COMPLEX_H
