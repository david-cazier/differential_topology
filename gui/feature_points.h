#ifndef FEATURE_POINTS_H
#define FEATURE_POINTS_H

#include <vector>

#include <cgogn/differential_topology/pl_functions.h>
#include <cgogn/rendering/drawer.h>

#include <cgogn/core/cmap/cmap2.h>


class FeaturePoints
{

public:
	using Dart = cgogn::Dart;
	using CMap2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
	using Vertex = CMap2::Vertex;

	template <typename T>
	using VertexAttribute = CMap2::VertexAttribute<T>;

	std::vector<Vertex> vertices_;
	cgogn::rendering::Drawer* drawer_;
	QOpenGLFunctions_3_3_Core* ogl33_;

	FeaturePoints(QOpenGLFunctions_3_3_Core* ogl33):
		drawer_(nullptr),
		ogl33_(ogl33)
	{}

	~FeaturePoints()
	{
		delete drawer_;
	}

	void init()
	{
		drawer_ = new cgogn::rendering::Drawer(ogl33_);
	}

	void clear()
	{
		vertices_.clear();
	}

	template <typename VEC3>
	void extract(CMap2& map,
				 CMap2::VertexAttribute<typename VEC3::Scalar> scalar,
				 CMap2::VertexAttribute<VEC3> position)
	{
		cgogn::extract_feature_points<typename VEC3::Scalar>(map, scalar, vertices_);

		drawer_->new_list();
		this->draw(vertices_, position, 1.0f, 1.0f, 1.0f, 1.0f);
		drawer_->end_list();
	}

	template <typename VEC3>
	void extract_intersection(CMap2& map,
							  CMap2::VertexAttribute<typename VEC3::Scalar> scalar1,
							  CMap2::VertexAttribute<typename VEC3::Scalar> scalar2,
							  CMap2::VertexAttribute<VEC3> position,
							  CMap2::EdgeAttribute<typename VEC3::Scalar> weight)
	{
		using Scalar = typename VEC3::Scalar;

		drawer_->new_list();

		std::vector<Vertex> vertices_f1;
		cgogn::extract_feature_points<Scalar>(map, scalar1, vertices_f1);
		this->draw(vertices_f1, position, 0.3f, 0.3f, 1.0f, 0.6f);

		std::vector<Vertex> vertices_f2;
		cgogn::extract_feature_points<Scalar>(map, scalar2, vertices_f2);
		this->draw(vertices_f2, position, 0.3f, 1.0f, 0.3f, 0.6f);

		//6. Intersection F1, F2
		VertexAttribute<Scalar> dist = map.add_attribute<Scalar, Vertex::ORBIT>("dist");
		VertexAttribute<Vertex> prev = map.add_attribute<Vertex, Vertex::ORBIT>("prev");

		// Remove from vertices_f2 the vertices that are too close to each other
		std::vector<Vertex> vertices_f2_not_close;
		while(!vertices_f2.empty())
		{
			Vertex v = vertices_f2.back();
			vertices_f2.pop_back();
			if (!vertices_f2.empty()) {
				cgogn::dijkstra_compute_normalized_paths<Scalar>(map, weight, vertices_f2, dist, prev);
				std::cout << "1>" << dist[v];
				if(dist[v] > Scalar(0.01)) {
					vertices_f2_not_close.push_back(v);
					std::cout << " +";
				}
				std::cout << std::endl;
			}
		}

		// Remove from vertices_f2 the vertices that are too close to vertices of vertices_f1
		cgogn::dijkstra_compute_normalized_paths<Scalar>(map, weight, vertices_f1, dist, prev);

		// search if dist < threshold
		for(auto& v: vertices_f2_not_close)
		{
			std::cout << "2>" << dist[v];
			if(dist[v] < Scalar(0.2)) {
				vertices_.push_back(v);
				std::cout << " +";
			}
			std::cout << std::endl;
		}

		map.remove_attribute(dist);
		map.remove_attribute(prev);

		this->draw(vertices_, position, 1.0f, 1.0f, 1.0f, 1.0f);
		drawer_->end_list();
	}

	template <typename VEC3>
	void minimize_distance(CMap2::VertexAttribute<typename VEC3::Scalar> distances)
	{
		using Scalar = typename VEC3::Scalar;

		Scalar dist = std::numeric_limits<Scalar>::max();
		Scalar distmin;
		Vertex vmin;

		for(std::vector<Vertex>::iterator it = vertices_.begin(); it != vertices_.end(); ++it)
		{
			if(distances[*it] < dist)
			{
				distmin = distances[*it];
				vmin = *it;
			}
		}
	}

	template <typename VEC3>
	void draw(std::vector<Vertex> vertices,
			  CMap2::VertexAttribute<VEC3> position,
			  float r, float g, float b, float ratio)
	{
		if (!vertices.empty()) {
			cgogn::geometry::BoundingBox<VEC3> bbox;
			cgogn::geometry::compute_bounding_box(position, bbox);

			drawer_->ball_size(ratio*bbox.max_size()/50.0f);
			drawer_->begin(GL_POINTS);
			drawer_->color3f(r, g, b);

			for (auto& v: vertices)
					drawer_->vertex3fv(position[v]);

			drawer_->end();
		}
	}

	void draw(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		drawer_->call_list(proj,view);
	}

};

#endif // FEATURE_POINTS_H
