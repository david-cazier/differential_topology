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
		drawer_ = new cgogn::rendering::Drawer();
	}

	void set(cgogn::ReebGraph<Vec3, CMap2>* rg, cgogn::CMap2<cgogn::DefaultMapTraits>::VertexAttribute<Vec3> position)
	{
		using DGraph = cgogn::ReebGraph<Vec3, CMap2>::DGraph;
		using Node = cgogn::ReebGraph<Vec3, CMap2>::Node;
		using Arc = cgogn::ReebGraph<Vec3, CMap2>::Arc;

		using Vertex = typename CMap2::Vertex;

		//init le dessin
		drawer_->new_list();
		drawer_->ball_size(0.015f);
		drawer_->begin(GL_POINTS);
		rg->graph_.foreach_cell([&] (Node n)
		{

//			Vertex v = rg->linked_vertex(n);
//			drawer_->vertex3fv(position[v]);
		});
		drawer_->end();
		drawer_->end_list();

	}

	void draw(const QMatrix4x4& proj, const QMatrix4x4& view)
	{
//		drawer_->call_list(proj, view, ogl33_);
	}
};

#endif // GRAPH_H
