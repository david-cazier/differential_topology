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
		drawer_->ball_size(0.015f);
		drawer_->begin(GL_POINTS);
		drawer_->color3f(1.0f, 1.0f, 1.0f);
		for (std::vector<Vertex>::iterator it = vertices_.begin(); it != vertices_.end(); ++it)
		{
				drawer_->vertex3fv(position[*it]);
		}
		drawer_->end();
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
////		std::vector<cgogn::Dart> vertices_f1;
//		cgogn::extract_feature_points<typename VEC3::Scalar>(map, scalar1, vertices_);
//		drawer_->new_list();
//		drawer_->ball_size(0.015f);
//		drawer_->begin(GL_POINTS);
//		drawer_->color3f(1.0f, 1.0f, 0.25f);
//		for (std::vector<Vertex>::iterator it = vertices_.begin(); it != vertices_.end(); ++it)
//		{
//				drawer_->vertex3fv(position[*it]);
//		}
//		drawer_->end();


//		vertices_.clear();

////		std::vector<cgogn::Dart> vertices_f2;
//		cgogn::extract_feature_points<typename VEC3::Scalar>(map, scalar2, vertices_);
//		drawer_->ball_size(0.016f);
//		drawer_->begin(GL_POINTS);
//		drawer_->color3f(1.0f, 1.0f, 0.75f);
//		for (std::vector<Vertex>::iterator it = vertices_.begin(); it != vertices_.end(); ++it)
//		{
//				drawer_->vertex3fv(position[*it]);
//		}
//		drawer_->end();

//		drawer_->end_list();

		std::vector<Vertex> vertices_f1;
		cgogn::extract_feature_points<Scalar>(map, scalar1, vertices_f1);
		std::vector<Vertex> vertices_f2;
		cgogn::extract_feature_points<Scalar>(map, scalar2, vertices_f2);

		//6. Intersection F1, F2
		VertexAttribute<Scalar> dist = map.add_attribute<Scalar, Vertex::ORBIT>("dist");
		VertexAttribute<cgogn::Dart> prev = map.add_attribute<cgogn::Dart, Vertex::ORBIT>("prev");

		for (std::vector<Vertex>::iterator it = vertices_f1.begin(); it !=vertices_f1.end(); ++it)
		{
			cgogn::dijkstra_compute_normalized_paths<Scalar>(map, weight, *it, dist, prev);

			// search if dist < threshold
			for(unsigned int i = 0 ; i < vertices_f2.size() ; ++i)
			{
				if(dist[vertices_f2[i]] < 0.05)
				{
					//std::cout << "yes! " << std::endl;
					vertices_.push_back(vertices_f2[i]);
				}
			}
		}
		map.remove_attribute(dist);
		map.remove_attribute(prev);

		drawer_->new_list();
		drawer_->ball_size(0.015f);
		drawer_->begin(GL_POINTS);
		drawer_->color3f(1.0f, 1.0f, 0.25f);
		for (std::vector<Vertex>::iterator it = vertices_.begin(); it != vertices_.end(); ++it)
		{
			drawer_->vertex3fv(position[*it]);
		}
		drawer_->end();
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

	void draw(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		drawer_->call_list(proj,view);
	}

};

#endif // FEATURE_POINTS_H
