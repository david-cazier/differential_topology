#ifndef GRAPH_H
#define GRAPH_H

#include <cgogn/rendering/drawer.h>

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/differential_topology/reeb_graph.h>
#include <cgogn/core/cmap/cmap2.h>


class Graph
{
public:
	using Vec3 = Eigen::Vector3d;
	using Scalar = Eigen::Vector3d::Scalar;

	using CMap2 = cgogn::CMap2<cgogn::DefaultMapTraits>;

	cgogn::rendering::Drawer* drawer_;
	QOpenGLFunctions_3_3_Core* ogl33_;

	Graph(QOpenGLFunctions_3_3_Core* ogl33):
		drawer_(nullptr),
		ogl33_(ogl33)
	{}

	~Graph()
	{
		delete drawer_;
	}

	void init()
	{
		drawer_ = new cgogn::rendering::Drawer(ogl33_);
	}

	void set(cgogn::ReebGraph<Scalar, CMap2>* rg)
	{
		//init le dessin
	}

	void draw(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
		drawer_->call_list(proj,view);
	}
};

#endif // GRAPH_H
