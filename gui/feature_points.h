#ifndef FEATURE_POINTS_H
#define FEATURE_POINTS_H

#include <vector>

#include <cgogn/differential_topology/pl_functions.h>
#include <cgogn/rendering/drawer.h>

#include <cgogn/core/cmap/cmap2.h>

template <typename VEC3>
class FeaturePoints
{

public:
	using Vec3 = VEC3;
	using Scalar = typename Vec3::Scalar;

	using Dart = cgogn::Dart;
	using CMap2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
	using Vertex = CMap2::Vertex;

	template <typename T>
	using VertexAttribute = CMap2::VertexAttribute<T>;

	std::vector<Vertex> vertices_;

	cgogn::rendering::Drawer* drawer_;
	cgogn::geometry::BoundingBox<Vec3> bb_;

	QOpenGLFunctions_3_3_Core* ogl33_;

	FeaturePoints(QOpenGLFunctions_3_3_Core* ogl33):
		drawer_(nullptr),
		bb_(),
		ogl33_(ogl33)
	{}

	~FeaturePoints()
	{
		delete drawer_;
	}

	void init(cgogn::geometry::BoundingBox<Vec3> bb)
	{
		drawer_ = new cgogn::rendering::Drawer();
		bb_ = bb;
	}

	void clear()
	{
		vertices_.clear();
	}

	void begin_draw()
	{
		drawer_->new_list();
	}

	void end_draw()
	{
		drawer_->end_list();
	}

	void extract(CMap2& map,
				 CMap2::VertexAttribute<Scalar> scalar,
				 CMap2::VertexAttribute<Vec3> position)
	{
		std::vector<Vertex> minima;
		std::vector<Vertex> saddles;
		cgogn::extract_critical_points<Scalar>(map, scalar, vertices_, minima, saddles);
		this->draw(minima, position, 1.0f, 1.0f, 1.0f, 1.0f);
		this->draw(vertices_, position, 1.0f, 0.0f, 0.0f, 0.8f);
		this->draw(saddles, position, 1.0f, 1.0f, 0.0f, 0.6f);
	}

	void extract_intersection(CMap2& map,
							  CMap2::VertexAttribute<Scalar> scalar1,
							  CMap2::VertexAttribute<Scalar> scalar2,
							  CMap2::VertexAttribute<Vec3> position,
							  CMap2::EdgeAttribute<Scalar> weight)
	{
		std::vector<Vertex> vertices_f1;
		cgogn::extract_extrema<Scalar>(map, scalar1, vertices_f1);
		this->draw(vertices_f1, position, 0.3f, 0.3f, 1.0f, 0.8f);
		std::cout << "F1 size: " << vertices_f1.size() << std::endl;

		std::vector<Vertex> vertices_f2;
		cgogn::extract_extrema<Scalar>(map, scalar2, vertices_f2);
		this->draw(vertices_f2, position, 0.3f, 1.0f, 0.3f, 0.8f);
		std::cout << "F2 size: " << vertices_f2.size() << std::endl;

		//6. Intersection F1, F2
		VertexAttribute<Scalar> dist_to_vertex = map.add_attribute<Scalar, Vertex::ORBIT>("dist_to_vertex");
		VertexAttribute<Vertex> path_to_vertex = map.add_attribute<Vertex, Vertex::ORBIT>("path_to_vertex");

		// Remove from vertices_f2 the vertices that are too close to each other
//		std::vector<Vertex> vertices_f2_not_close;
//		while(!vertices_f2.empty())
//		{
//			Vertex v = vertices_f2.back();
//			vertices_f2.pop_back();
//			if (!vertices_f2.empty()) {
//				cgogn::dijkstra_compute_normalized_paths<Scalar>(map, weight, vertices_f2, dist_to_vertex, path_to_vertex);
//				if(dist_to_vertex[v] > Scalar(0.05))
//					vertices_f2_not_close.push_back(v);
//			}
//		}
//		std::cout << "F2b size: " << vertices_f2_not_close.size() << std::endl;

		// Remove from vertices_f2 the vertices that are too far to vertices of vertices_f1
		cgogn::dijkstra_compute_normalized_paths<Scalar>(map, weight, vertices_f1, dist_to_vertex, path_to_vertex);

		// search if dist < threshold
		for(auto& v: vertices_f2)
		{
			if(dist_to_vertex[v] < Scalar(0.2))
				vertices_.push_back(v);
		}
		std::cout << "F12 size: " << vertices_.size() << std::endl;

		map.remove_attribute(dist_to_vertex);
		map.remove_attribute(path_to_vertex);

		this->draw(vertices_, position, 1.0f, 1.0f, 1.0f, 1.0f);
	}

	void minimize_distance(CMap2::VertexAttribute<Scalar> distances)
	{
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

	void draw(std::vector<Vertex> vertices,
			  CMap2::VertexAttribute<Vec3> position,
			  float r, float g, float b, float ratio, int shift=0)
	{
		Scalar radius = ratio*bb_.max_size()/50.0f;
		if (!vertices.empty()) {
			drawer_->ball_size(radius);
			drawer_->begin(GL_POINTS);
			drawer_->color3f(r, g, b);

			for (auto& v: vertices) {
				switch(shift) {
					case  1 :
						drawer_->vertex3fv(position[v]+Vec3(radius/Scalar(2),-radius/Scalar(2),Scalar(0)));
						break;
					case  2 :
						drawer_->vertex3fv(position[v]+Vec3(-radius/Scalar(2),radius/Scalar(2),Scalar(0)));
						break;
					case  3 :
						drawer_->vertex3fv(position[v]+Vec3(radius/Scalar(2),radius/Scalar(2),Scalar(0)));
						break;
					default :
						drawer_->vertex3fv(position[v]);
				}
			}

			drawer_->end();
		}
	}

	void draw(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		drawer_->call_list(proj, view, ogl33_);
	}

};

#endif // FEATURE_POINTS_H
