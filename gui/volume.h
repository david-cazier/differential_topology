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


#ifndef VOLUME_H
#define VOLUME_H

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/utils/definitions.h>
#include <cgogn/io/map_import.h>
#include <cgogn/io/map_export.h>

#include <cgogn/rendering/map_render.h>
#include <cgogn/rendering/drawer.h>
#include <cgogn/rendering/volume_drawer.h>

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

template <typename VEC3>
class VolumeMesh
{
public:
	using Vec3 = VEC3;
	using Scalar = typename Vec3::Scalar;

	using CMap3 = cgogn::CMap3<cgogn::DefaultMapTraits>;
	using Vertex = CMap3::Vertex;
	using Edge = CMap3::Edge;
	using Face = CMap3::Face;
	using Volume = CMap3::Volume;
	template<typename T>
	using VertexAttribute = CMap3::VertexAttribute<T>;
	template<typename T>
	using EdgeAttribute = CMap3::EdgeAttribute<T>;

public:
	CMap3 map_;

	VertexAttribute<Vec3> vertex_position_;
	VertexAttribute<Vec3> vertex_color_;
	VertexAttribute<Scalar> scalar_field_;

	EdgeAttribute<Scalar> edge_metric_;

	cgogn::geometry::BoundingBox<Vec3> bb_;

	QOpenGLFunctions_3_3_Core* ogl33_;

    std::unique_ptr<cgogn::rendering::VolumeDrawerColor> volume_drawer_;
    std::unique_ptr<cgogn::rendering::VolumeDrawerColor::Renderer> volume_renderer_;

public:

	VolumeMesh(QOpenGLFunctions_3_3_Core* ogl33):
		map_(),
		vertex_position_(),
		vertex_color_(),
		scalar_field_(),
		edge_metric_(),
		bb_(),
		ogl33_(ogl33),
		volume_drawer_(nullptr),
		volume_renderer_(nullptr)
	{
	}

	~VolumeMesh()
	{
		volume_drawer_.reset();
	}

	template <typename T, typename MAP>
	inline T map_centroid(MAP& map, VertexAttribute<T>& attribute)
	{
		T result;
		result.setZero();
		unsigned int count = 0;

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
        volume_drawer_ = cgogn::make_unique<cgogn::rendering::VolumeDrawerColor>();
        volume_drawer_->update_face<Vec3>(map_, vertex_position_, vertex_color_);
        volume_drawer_->update_edge<Vec3>(map_, vertex_position_);

		volume_renderer_ = volume_drawer_->generate_renderer();
	}

	void update_geometry()
	{
        volume_drawer_->update_face<Vec3>(map_, vertex_position_, vertex_color_);
        volume_drawer_->update_edge<Vec3>(map_, vertex_position_);
	}

	void update_topology()
	{
        volume_drawer_->update_face<Vec3>(map_, vertex_position_, vertex_color_);
        volume_drawer_->update_edge<Vec3>(map_, vertex_position_);
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

        map_.foreach_cell([&](Vertex v)
        {
            std::array<Scalar,3> color = cgogn::color_map_blue_green_red(
                        cgogn::numerics::scale_to_0_1(scalar[v], min, max));
            vertex_color_[v] = Vec3(color[0], color[1], color[2]);
        });
        volume_drawer_->update_face<Vec3>(map_, vertex_position_, vertex_color_);
	}

	void draw_volume(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		volume_renderer_->set_explode_volume(0.8f);
		volume_renderer_->draw_faces(proj, view, ogl33_);
        volume_renderer_->draw_edges(proj, view, ogl33_);
    }

	void draw_flat(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		volume_renderer_->set_explode_volume(1.0f);
		volume_renderer_->draw_faces(proj, view, ogl33_);
	}

	void draw_edges(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		volume_renderer_->draw_edges(proj, view, ogl33_);
	}

	cgogn::geometry::BoundingBox<Vec3> import(const std::string& filename)
	{
		cgogn::io::import_volume<Vec3>(map_, filename);

		vertex_position_ = map_.get_attribute<Vec3, Vertex::ORBIT>("position");
		vertex_color_ = map_.add_attribute<Vec3, Vertex::ORBIT>("color");
		scalar_field_ = map_.add_attribute<Scalar, Vertex::ORBIT>("scalar_field_");
		edge_metric_ = map_.add_attribute<Scalar, Edge::ORBIT>("edge_metric");

		cgogn::geometry::compute_bounding_box(vertex_position_, bb_);

		return bb_;
	}

	Vertex central_vertex()
	{
		Vec3 barycenter = map_centroid<Vec3>(map_, vertex_position_);

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
					   VertexAttribute<Scalar>& scalar_field,
					   std::vector<Vertex>& features)
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
		// std::cout << "F1 size: " << vertices_f1.size() << std::endl;

		// Get the maxima in the scalar field f2
		std::vector<Vertex> vertices_f2;
		cgogn::extract_maxima<Scalar>(map_, f2, vertices_f2);
		// std::cout << "F2 size: " << vertices_f2.size() << std::endl;

		// Build a scalar field from {v1, v2}
		features.push_back(v1);
		features.push_back(v2);
		cgogn::dijkstra_compute_paths<Scalar>(map_, edge_metric_, features, scalar_field, path_to_sources);

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
			Vertex v = farthest_extremity(features, scalar_field, path_to_sources);
			features.push_back(v);
			actual_distance = scalar_field[v];
			filter_distance = std::min(filter_distance, target_distance / Scalar(3));
			// std::cout << actual_distance/max_distance << " ";
			cgogn::dijkstra_compute_paths<Scalar>(map_, edge_metric_, features, scalar_field, path_to_sources);
			features_filter(vertices_f1, vertices_f1_filtered, filter_distance, scalar_field, path_to_sources);
			features_filter(vertices_f2, vertices_f2_filtered, filter_distance, scalar_field, path_to_sources);
		}
		// std::cout << std::endl;

		intersection(vertices_f1_filtered, vertices_f2_filtered, features);
		std::cout << "Selected features: " << features.size() << std::endl;

		// Draw the unselected extrema in f1 and f2
		fp.draw_vertices(vertices_f1, vertex_position_, 0.3f, 0.3f, 1.0f, 0.6f, 1);
		// std::cout << "F1 not filtered features: " << vertices_f1.size() << std::endl;
		fp.draw_vertices(vertices_f2, vertex_position_, 0.3f, 1.0f, 0.3f, 0.6f, 2);
		// std::cout << "F2 not filtered features: " << vertices_f2.size() << std::endl;

		// Draw the selected features that are not in the intersection
		fp.draw_vertices(vertices_f1_filtered, vertex_position_, 1.0f, 0.2f, 1.0f, 0.8f, 1);
		// std::cout << "F1 not in F2 features: " << vertices_f1_filtered.size() << std::endl;
		fp.draw_vertices(vertices_f2_filtered, vertex_position_, 1.0f, 0.4f, 0.4f, 0.8f, 2);
		// std::cout << "F2 not in F1 features: " << vertices_f2_filtered.size() << std::endl;

		map_.remove_attribute(path_to_sources);
		map_.remove_attribute(f1);
		map_.remove_attribute(f2);
	}

	void height_function(FeaturePoints<Vec3>& fp)
	{
		cgogn::height_pl_function<Vec3>(map_, vertex_position_, scalar_field_);

		update_color(scalar_field_);
		fp.draw_critical_points(map_, scalar_field_, vertex_position_);
	}

	void distance_to_center_function(FeaturePoints<Vec3>& fp)
	{
		compute_length(edge_metric_);

		Vertex v0 = central_vertex();
		cgogn::geodesic_distance_pl_function<Scalar>(map_, {v0}, edge_metric_, scalar_field_);

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
		cgogn::geodesic_distance_pl_function<Scalar>(map_, features, edge_metric_, scalar_field_);

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
		cgogn::geodesic_distance_pl_function<Scalar>(map_, features, edge_metric_, scalar_field_);

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
		cgogn::extract_level_sets<Scalar>(map_ , scalar_field, level_lines);
		update_color(scalar_field);
		fp.draw_edges(map_, level_lines, vertex_position_, 1.0f, 1.0f, 1.0f);

		std::vector<Vertex> maxima;
		std::vector<Vertex> minima;
		std::vector<Vertex> saddles;
		cgogn::extract_critical_points<Scalar>(map_, scalar_field, maxima, minima, saddles);
		fp.draw_vertices(maxima, vertex_position_, 1.0f, 1.0f, 1.0f, 1.0f);
		fp.draw_vertices(minima, vertex_position_, 1.0f, 0.0f, 0.0f, 0.8f);
		fp.draw_vertices(saddles, vertex_position_, 1.0f, 1.0f, 0.0f, 0.8f);
	}

	void morse_function(FeaturePoints<VEC3>& fp,
						VertexAttribute<Scalar>& scalar_field,
						const EdgeAttribute<Scalar>& edge_metric)
	{
		// Etape 1 get the two farthest vertices and their associated scalar field
		// (the scalar field contains the geodesic distance to the nearest feature)
		VertexAttribute<Scalar> dist_to_feature = map_.add_attribute<Scalar, Vertex::ORBIT>("dist_to_feature");
		std::vector<Vertex> features;
		find_features(fp, dist_to_feature, features);

		// Etape 2
		// 1 initial function with Feature vertices as seeds
		cgogn::normalized_geodesic_distance_pl_function<Scalar>(map_, features, edge_metric, dist_to_feature);

		// 2 inverse the normalized distance so that the maxima are on the features
		for (auto& s : dist_to_feature) s = Scalar(1) - s;

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

};

#endif // VOLUME_H
