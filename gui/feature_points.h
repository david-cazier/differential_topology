#ifndef FEATURE_POINTS_H
#define FEATURE_POINTS_H

#include <vector>

#include <cgogn/differential_topology/pl_functions.h>
#include <rendering/drawer.h>

#include <core/cmap/cmap2.h>


class FeaturePoints
{

public:
	using Dart = cgogn::Dart;
	using CMap2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
	using Vertex = CMap2::Vertex;

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
				 CMap2::VertexAttributeHandler<typename VEC3::Scalar> scalar,
				 CMap2::VertexAttributeHandler<VEC3> position)
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
							  CMap2::VertexAttributeHandler<typename VEC3::Scalar> scalar1,
							  CMap2::VertexAttributeHandler<typename VEC3::Scalar> scalar2,
							  CMap2::VertexAttributeHandler<VEC3> position)
	{
//		std::vector<cgogn::Dart> vertices_f1;
		cgogn::extract_feature_points<typename VEC3::Scalar>(map, scalar1, vertices_);
		drawer_->new_list();
		drawer_->ball_size(0.015f);
		drawer_->begin(GL_POINTS);
		drawer_->color3f(1.0f, 1.0f, 0.25f);
		for (std::vector<Vertex>::iterator it = vertices_.begin(); it != vertices_.end(); ++it)
		{
				drawer_->vertex3fv(position[*it]);
		}
		drawer_->end();


		vertices_.clear();

//		std::vector<cgogn::Dart> vertices_f2;
		cgogn::extract_feature_points<typename VEC3::Scalar>(map, scalar2, vertices_);
		drawer_->ball_size(0.016f);
		drawer_->begin(GL_POINTS);
		drawer_->color3f(1.0f, 1.0f, 0.75f);
		for (std::vector<Vertex>::iterator it = vertices_.begin(); it != vertices_.end(); ++it)
		{
				drawer_->vertex3fv(position[*it]);
		}
		drawer_->end();

		drawer_->end_list();
	}

	void draw(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		drawer_->call_list(proj,view);
	}

};

#endif // FEATURE_POINTS_H
