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
	using CMap3 = cgogn::CMap3<cgogn::DefaultMapTraits>;
	using Vertex = CMap3::Vertex;
	using Edge = CMap3::Edge;

	template <typename T>
	using VertexAttribute = CMap3::VertexAttribute<T>;

	std::unique_ptr<cgogn::rendering::DisplayListDrawer> drawer_;
	std::unique_ptr<cgogn::rendering::DisplayListDrawer::Renderer> renderer_;
	cgogn::geometry::BoundingBox<Vec3> bb_;

	QOpenGLFunctions_3_3_Core* ogl33_;

	FeaturePoints(QOpenGLFunctions_3_3_Core* ogl33):
		drawer_(nullptr),
		bb_(),
		ogl33_(ogl33)
	{}

	~FeaturePoints()
	{
		drawer_.reset();
	}

	void init(cgogn::geometry::BoundingBox<Vec3> bb)
	{
		drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
		renderer_ = drawer_->generate_renderer();
		bb_ = bb;
	}

	void begin_draw()
	{
		drawer_->new_list();
	}

	void end_draw()
	{
		drawer_->end_list();
	}

	void draw_edges(CMap3& map,
					std::vector<Edge> edges,
					CMap3::VertexAttribute<Vec3> position,
					float r, float g, float b)
	{
		Scalar width = bb_.max_size()/50.0f;
		if (!edges.empty()) {
			drawer_->line_width(width);
			drawer_->begin(GL_LINES);
			drawer_->color3f(r, g, b);

			for (auto& e: edges) {
				drawer_->vertex3fv(position[Vertex(e.dart)]);
				drawer_->vertex3fv(position[Vertex(map.phi1(e.dart))]);
			}
			drawer_->end();
		}
	}

	void draw_critical_points(CMap3& map,
				 CMap3::VertexAttribute<Scalar> scalar,
				 CMap3::VertexAttribute<Vec3> position)
	{
		std::vector<Vertex> maxima;
		std::vector<Vertex> minima;
		std::vector<Vertex> saddles;
		cgogn::extract_critical_points<Scalar>(map, scalar, maxima, minima, saddles);
		std::cout << "maxima : " << maxima.size() << std::endl;
		std::cout << "minima : " << minima.size() << std::endl;
		std::cout << "saddles : " << saddles.size() << std::endl;
		this->draw_vertices(minima, position, 1.0f, 1.0f, 1.0f, 1.0f);
		this->draw_vertices(maxima, position, 1.0f, 0.0f, 0.0f, 0.8f);
		this->draw_vertices(saddles, position, 1.0f, 1.0f, 0.0f, 0.6f);
	}

	void draw_vertices(std::vector<Vertex> vertices,
			  CMap3::VertexAttribute<Vec3> position,
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
		renderer_->draw(proj, view, ogl33_);
	}
};

#endif // FEATURE_POINTS_H
