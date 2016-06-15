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

#include <cgogn/rendering/shaders/shader_scalar_per_vertex.h>
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

#include <cgogn/topology/types/adjacency_cache.h>
#include <cgogn/topology/algos/features.h>

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

	cgogn::topology::AdjacencyCache<MAP> adjacency_cache_;

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

	std::unique_ptr<cgogn::rendering::DisplayListDrawer> features_drawer_;
	std::unique_ptr<cgogn::rendering::DisplayListDrawer::Renderer> features_renderer_;

	std::unique_ptr<cgogn::rendering::VBO> vbo_pos_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_color_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_scalar_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_sphere_sz_;

	std::unique_ptr<cgogn::rendering::ShaderBoldLine::Param> param_edge_;
	std::unique_ptr<cgogn::rendering::ShaderScalarPerVertex::Param> param_scalar_;
	std::unique_ptr<cgogn::rendering::ShaderPointSpriteColorSize::Param> param_point_sprite_;

public:

	MorseSmallComplex(QOpenGLFunctions_3_3_Core* ogl33):
		map_(),
		adjacency_cache_(map_),
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
		vbo_color_(nullptr),
		vbo_scalar_(nullptr),
		vbo_sphere_sz_(nullptr)
	{
	}

	~MorseSmallComplex()
	{
		volume_drawer_.reset();
		surface_render_.reset();
		features_drawer_.reset();
		vbo_pos_.reset();
		vbo_color_.reset();
		vbo_scalar_.reset();
		vbo_sphere_sz_.reset();
	}

	void init()
	{
		if (dimension_ == 2)
		{
			vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
			cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());

			vbo_color_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
			cgogn::rendering::update_vbo(scalar_field_, vbo_color_.get(), [] (double s) -> std::array<float,3>
			{
				return {float(s), float(s), float(s)};
			});

			vbo_scalar_ = cgogn::make_unique<cgogn::rendering::VBO>(1);
			cgogn::rendering::update_vbo(scalar_field_, vbo_scalar_.get());

			// fill a sphere size vbo
			vbo_sphere_sz_ = cgogn::make_unique<cgogn::rendering::VBO>(1);
			cgogn::rendering::update_vbo(scalar_field_, vbo_sphere_sz_.get(),[&] (double s) -> float
			{
				return bb_.max_size()/1000.0f * (2.0 + s);
			});

			surface_render_ = cgogn::make_unique<cgogn::rendering::MapRender>();
			surface_render_->init_primitives<Vec3>(map_, cgogn::rendering::POINTS);
			surface_render_->init_primitives<Vec3>(map_, cgogn::rendering::LINES);
			surface_render_->init_primitives<Vec3>(map_, cgogn::rendering::TRIANGLES, &vertex_position_);

			param_point_sprite_ = cgogn::rendering::ShaderPointSpriteColorSize::generate_param();
			param_point_sprite_->set_all_vbos(vbo_pos_.get(),vbo_color_.get(),vbo_sphere_sz_.get());

			param_edge_ = cgogn::rendering::ShaderBoldLine::generate_param();
			param_edge_->color_ = QColor(10,10,40);
			param_edge_->width_= 1.5f;
			param_edge_->set_position_vbo(vbo_pos_.get());

			param_scalar_ = cgogn::rendering::ShaderScalarPerVertex::generate_param();
			param_scalar_->color_map_ = cgogn::rendering::ShaderScalarPerVertex::ColorMap::BGR;
			param_scalar_->show_iso_lines_ = true;
			param_scalar_->min_value_ = 0.0f;
			param_scalar_->max_value_ = 0.0f;
			param_scalar_->set_all_vbos(vbo_pos_.get(), vbo_scalar_.get());
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

		features_drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
		features_renderer_ = features_drawer_->generate_renderer();
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 2>::type* = nullptr>
	void update_geometry_concrete()
	{
		cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());
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
			cgogn_log_warning("update_topology") << "invalid dimension";
	}

	void update_scalar_field(bool level_sets = false, bool morse_complex = false)
	{
		// Search the maximal and minimal value of the scalar field
		Scalar min = std::numeric_limits<Scalar>::max();
		Scalar max = std::numeric_limits<Scalar>::min();
		for(auto& v : scalar_field_)
		{
			min = std::min(min, v);
			max = std::max(max, v);
		}

		// Update the surface or volume rendering
		if (dimension_ == 2u)
		{
			param_scalar_->min_value_ = min;
			param_scalar_->max_value_ = max;
			cgogn::rendering::update_vbo(scalar_field_, vbo_scalar_.get());
			cgogn::rendering::update_vbo(scalar_field_, vbo_color_.get(), [] (double s) -> std::array<float,3>
			{
				return {float(s), float(s), float(s)};
			});
			cgogn::rendering::update_vbo(scalar_field_, vbo_sphere_sz_.get(),[&] (double s) -> float
			{
				return bb_.max_size()/1000.0f * (2.0 + s);
			});
		}
		else if (dimension_ == 3u)
		{
			map_.foreach_cell([&](Vertex v)
			{
				std::array<float,3> color = cgogn::color_map_blue_green_red(
							cgogn::numerics::scale_to_0_1(scalar_field_[v], min, max));
				vertex_color_[v] = Vec3(color[0], color[1], color[2]);
			});
			volume_drawer_->update_face<Vec3>(map_, vertex_position_, vertex_color_);
		}
		else
			cgogn_log_warning("draw_flat") << "invalid dimension";

		features_drawer_->new_list();

		// Draw the critical points
		cgogn::topology::ScalarField<Scalar, MAP> scalar_field(map_, adjacency_cache_, scalar_field_);
		scalar_field.critical_vertex_analysis();
		draw_vertices(scalar_field.get_maxima(), 1.0f, 1.0f, 1.0f, 1.0f);
		if (scalar_field.get_minima().size() < 100u)
			draw_vertices(scalar_field.get_minima(), 1.0f, 0.0f, 0.0f, 1.0f);
		else
			draw_vertices(scalar_field.get_minima(), 1.0f, 0.0f, 0.0f, 0.1f);
		draw_vertices(scalar_field.get_saddles(), 1.0f, 1.0f, 0.0f, 0.4f);

		// Draw the level sets
		if (level_sets)
		{
			std::vector<Edge> level_lines;
			scalar_field.extract_level_sets(level_lines);
			draw_edges(level_lines, 1.0f, 1.0f, 1.0f);
		}

		// Draw the ascending and descending manyfold of the morse complex
		if (morse_complex)
		{
			std::vector<Edge> morse_lines;
			scalar_field.extract_descending_manifold(morse_lines);
			draw_edges(morse_lines, 0.5f, 0.5f, 1.0f);
			morse_lines.clear();
			scalar_field.extract_ascending_manifold(morse_lines);
			draw_edges(morse_lines, 1.0f, 0.5f, 0.0f);
		}

		features_drawer_->end_list();
	}

	void draw_edges(const std::vector<Edge>& edges,
					float r, float g, float b)
	{
		Scalar width = 10.0f * bb_.max_size()/50.0f;
		if (!edges.empty()) {
			features_drawer_->line_width(width);
			features_drawer_->begin(GL_LINES);
			features_drawer_->color3f(r, g, b);

			for (auto& e: edges) {
				features_drawer_->vertex3fv(vertex_position_[Vertex(e.dart)]);
				features_drawer_->vertex3fv(vertex_position_[Vertex(map_.phi1(e.dart))]);
			}
			features_drawer_->end();
		}
	}

	void draw_vertices(const std::vector<Vertex>& vertices,
					   float r, float g, float b, float ratio, int shift=0)
	{
		Scalar radius = ratio*bb_.max_size()/50.0f;
		if (!vertices.empty()) {
			features_drawer_->ball_size(radius);
			features_drawer_->begin(GL_POINTS);
			features_drawer_->color3f(r, g, b);

			for (auto& v: vertices) {
				switch(shift) {
					case  1 :
						features_drawer_->vertex3fv(vertex_position_[v]+Vec3(radius/Scalar(2),-radius/Scalar(2),Scalar(0)));
						break;
					case  2 :
						features_drawer_->vertex3fv(vertex_position_[v]+Vec3(-radius/Scalar(2),radius/Scalar(2),Scalar(0)));
						break;
					case  3 :
						features_drawer_->vertex3fv(vertex_position_[v]+Vec3(radius/Scalar(2),radius/Scalar(2),Scalar(0)));
						break;
					default :
						features_drawer_->vertex3fv(vertex_position_[v]);
				}
			}
			features_drawer_->end();
		}
	}

	void draw_features(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		features_renderer_->draw(proj, view, ogl33_);
	}

	void draw(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		if (dimension_ == 2u)
		{
			param_scalar_->bind(proj,view);
			surface_render_->draw(cgogn::rendering::TRIANGLES);
			param_scalar_->release();
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
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			surface_render_->draw(cgogn::rendering::LINES);
			glDisable(GL_BLEND);
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

		cgogn::geometry::compute_normal<Vec3>(map_, vertex_position_, vertex_normal_);
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
		adjacency_cache_.init();
		scalar_field_ = map_.template add_attribute<Scalar, Vertex::ORBIT>("scalar_field_");
		edge_metric_ = map_.template add_attribute<Scalar, Edge::ORBIT>("edge_metric");
	}

	void height_function()
	{
		map_.foreach_cell([&] (Vertex v)
		{
			scalar_field_[v] = vertex_position_[v][0];
		});

		update_scalar_field();
	}

	void distance_to_boundary_function()
	{
		VertexAttribute<Scalar> features_field;
		features_field = map_.template add_attribute<Scalar, Vertex::ORBIT>("features_field");

		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_length(edge_metric_);

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.get_filtered_features(center, features_field, features);

		VertexAttribute<Scalar> boundary_field;
		boundary_field = map_.template add_attribute<Scalar, Vertex::ORBIT>("boundary_field");
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.distance_to_boundary(boundary_field);

		cgogn::topology::ScalarField<Scalar, MAP> scalar_field(map_, adjacency_cache_, boundary_field);
		scalar_field.critical_vertex_analysis();

		for (Vertex v : scalar_field.get_maxima())
			features.push_back(v);

		distance_field.distance_to_features(features, scalar_field_);

		for (auto& s : scalar_field_) s = Scalar(1) - s;

		update_scalar_field(false, true);

		map_.remove_attribute(features_field);
		map_.remove_attribute(boundary_field);
	}

	void distance_to_center_function()
	{
		compute_length(edge_metric_);

		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.distance_to_center(vertex_position_, scalar_field_);

		update_scalar_field();
	}

	void edge_length_weighted_geodesic_distance_function()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_length(edge_metric_);

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.get_filtered_features(center, scalar_field_, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.distance_to_features(features, scalar_field_);

		for (auto& s : scalar_field_) s = Scalar(1) - s;

		update_scalar_field();
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 2>::type* = nullptr>
	void curvature_weighted_geodesic_distance_function()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_curvature<MAP>(edge_metric_);

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.get_filtered_features(center, scalar_field_, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.distance_to_features(features, scalar_field_);

		for (auto& s : scalar_field_) s = Scalar(1) - s;

		update_scalar_field();
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 3>::type* = nullptr>
	void curvature_weighted_geodesic_distance_function()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_);
		features_finder.get_filtered_features(center, scalar_field_, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_);
		distance_field.distance_to_features(features, scalar_field_);

		for (auto& s : scalar_field_) s = Scalar(1) - s;

		update_scalar_field();
	}

	void curvature_weighted_geodesic_distance_function()
	{
		curvature_weighted_geodesic_distance_function<MAP>();
	}

	void edge_length_weighted_morse_function()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_length(edge_metric_);

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.get_filtered_features(center, scalar_field_, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.morse_distance_to_features(features, scalar_field_);

		update_scalar_field(false, true);
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 2>::type* = nullptr>
	void curvature_weighted_morse_function()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;
		compute_curvature<MAP>(edge_metric_);

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.get_filtered_features(center, scalar_field_, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_, edge_metric_);
		distance_field.morse_distance_to_features(features, scalar_field_);

		update_scalar_field();
	}

	template <typename T, typename std::enable_if<T::DIMENSION == 3>::type* = nullptr>
	void curvature_weighted_morse_function()
	{
		// Find features for the edge_metric
		std::vector<Vertex> features;

		Vertex center = cgogn::geometry::central_vertex<Vec3, MAP>(map_, vertex_position_);
		cgogn::topology::FeaturesFinder<Scalar, MAP> features_finder(map_, adjacency_cache_, edge_metric_);
		features_finder.get_filtered_features(center, scalar_field_, features);

		// Build the scalar field from the selected features
		cgogn::topology::DistanceField<Scalar, MAP> distance_field(map_, adjacency_cache_);
		distance_field.morse_distance_to_features(features, scalar_field_);

		update_scalar_field();
	}

	void curvature_weighted_morse_function()
	{
		curvature_weighted_morse_function<MAP>();
	}

	void show_level_sets()
	{
		update_scalar_field(true);
	}

	void compute_length(EdgeAttribute<Scalar>& length)
	{
		map_.foreach_cell([&](Edge e)
		{
			length[e] = cgogn::geometry::length<Vec3>(map_, e, vertex_position_);
		});
	}

	template <typename CONCRETE_MAP, typename std::enable_if<CONCRETE_MAP::DIMENSION == 2>::type* = nullptr>
	void compute_curvature(EdgeAttribute<Scalar>& edge_metric)
	{
		EdgeAttribute<Scalar> length = map_.template add_attribute<Scalar, Edge::ORBIT>("lenght");
		EdgeAttribute<Scalar> edgeangle = map_.template add_attribute<Scalar, Edge::ORBIT>("edgeangle");
		EdgeAttribute<Scalar> edgeaera = map_.template add_attribute<Scalar, Edge::ORBIT>("edgeaera");

		VertexAttribute<Scalar> kmax = map_.template add_attribute<Scalar, Vertex::ORBIT>("kmax");
		VertexAttribute<Scalar> kmin = map_.template add_attribute<Scalar, Vertex::ORBIT>("kmin");
		VertexAttribute<Vec3> Kmax = map_.template add_attribute<Vec3, Vertex::ORBIT>("Kmax");
		VertexAttribute<Vec3> Kmin = map_.template add_attribute<Vec3, Vertex::ORBIT>("Kmin");
		VertexAttribute<Vec3> knormal = map_.template add_attribute<Vec3, Vertex::ORBIT>("knormal");

		compute_length(length);

		cgogn::geometry::compute_angle_between_face_normals<Vec3, MAP>(map_, vertex_position_, edgeangle);
		cgogn::geometry::compute_incident_faces_area<Vec3, Edge, MAP>(map_, vertex_position_, edgeaera);

		Scalar meanEdgeLength = cgogn::geometry::mean_edge_length<Vec3>(map_, vertex_position_);

		Scalar radius = Scalar(2.0) * meanEdgeLength;

		cgogn::geometry::compute_curvature<Vec3>(map_,radius, vertex_position_, vertex_normal_,edgeangle,edgeaera,kmax,kmin,Kmax,Kmin,knormal);

		//compute kmean
		VertexAttribute<Scalar> kmean = map_.template add_attribute<Scalar, Vertex::ORBIT>("kmean");

		double min = std::numeric_limits<double>::max();
		double max = std::numeric_limits<double>::min();

		map_.foreach_cell([&](Vertex v)
		{
			kmean[v] = (kmin[v] + kmax[v]) / Scalar(2.0);
			min = std::min(min, kmean[v]);
			max = std::max(max, kmean[v]);
		});

		//compute kgaussian
		VertexAttribute<Scalar> kgaussian = map_.template add_attribute<Scalar, Vertex::ORBIT>("kgaussian");

		min = std::numeric_limits<double>::max();
		max = std::numeric_limits<double>::min();

		map_.foreach_cell([&](Vertex v)
		{
			kgaussian[v] = (kmin[v] * kmax[v]);
			min = std::min(min, kgaussian[v]);
			max = std::max(max, kgaussian[v]);
		});

		//compute kindex
		VertexAttribute<Scalar> k1 = map_.template add_attribute<Scalar, Vertex::ORBIT>("k1");
		VertexAttribute<Scalar> k2 = map_.template add_attribute<Scalar, Vertex::ORBIT>("k2");
		VertexAttribute<Scalar> kI = map_.template add_attribute<Scalar, Vertex::ORBIT>("kI");

		map_.foreach_cell([&](Vertex v)
		{
			k1[v] = kmean[v] + std::sqrt(kmean[v] * kmean[v] - kgaussian[v]);
			k2[v] = kmean[v] - std::sqrt(kmean[v] * kmean[v] - kgaussian[v]);

			if(k1[v] == k2[v])
				kI[v] = 0.0;
			else
				kI[v] = (2 / M_PI) * std::atan((k1[v] + k2[v]) / (k1[v] - k2[v]));
		});

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

#endif // MORSE_COMPLEX_H
