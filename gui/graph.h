#ifndef GRAPH_H
#define GRAPH_H

#include <cgogn/differential_topology/reeb_graph.h>
#include <rendering/drawer.h>

#include <geometry/types/eigen.h>
#include <core/cmap/cmap2.h>


class Graph
{
public:
	using Vec3 = Eigen::Vector3d;
	using Scalar = Eigen::Vector3d::Scalar;

	using Dart = cgogn::Dart;
	using CMap2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
	using Vertex = CMap2::Vertex;

	cgogn::rendering::Drawer* drawer_;
	QOpenGLFunctions_3_3_Core* ogl33_;

	cgogn::ReebGraph<Vec3, CMap2> reeb_graph;

	Graph(QOpenGLFunctions_3_3_Core* ogl33):
			drawer_(nullptr),
			ogl33_(ogl33),
			reeb_graph()
	{}

	~Graph()
	{
			delete drawer_;
	}

	void init()
	{
			drawer_ = new cgogn::rendering::Drawer(ogl33_);
	}

	void draw(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
			drawer_->call_list(proj,view);
	}
};

#endif // GRAPH_H
