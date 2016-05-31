#ifndef FEATURE_POINTS_H
#define FEATURE_POINTS_H

#include <vector>

#include <cgogn/topology/algos/scalar_field.h>
#include <cgogn/rendering/drawer.h>

#include <cgogn/core/cmap/cmap2.h>

template <typename VEC3>
class FeaturePoints
{

public:
	using Vec3 = VEC3;
	using Scalar = typename Vec3::Scalar;

	std::unique_ptr<cgogn::rendering::DisplayListDrawer> drawer_;
	std::unique_ptr<cgogn::rendering::DisplayListDrawer::Renderer> renderer_;

	cgogn::geometry::AABB<Vec3> bb_;

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

	void init(cgogn::geometry::AABB<Vec3> bb)
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

	template <typename MAP>
	void draw_edges(MAP& map,
					const std::vector<typename MAP::Edge>& edges,
					const typename MAP::template VertexAttribute<Vec3>& position,
					float r, float g, float b)
	{
		using Vertex = typename MAP::Vertex;

		Scalar width = 10.0f * bb_.max_size()/50.0f;
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

	template <typename MAP>
	void draw_critical_points(MAP& map,
							  const typename MAP::template VertexAttribute<Scalar>& scalar,
							  const typename MAP::template VertexAttribute<Vec3>& position)
	{
		using Vertex = typename MAP::Vertex;

		cgogn::topology::ScalarField<Scalar, MAP> scalar_field(map, scalar);
		scalar_field.differential_analysis();
		std::cout << "maxima : " << scalar_field.get_maxima().size() << std::endl;
		std::cout << "minima : " << scalar_field.get_minima().size() << std::endl;
		std::cout << "saddles : " << scalar_field.get_saddles().size() << std::endl;
		this->draw_vertices<MAP>(scalar_field.get_maxima(), position, 1.0f, 1.0f, 1.0f, 1.0f);
		if (scalar_field.get_minima().size() < 100u)
			this->draw_vertices<MAP>(scalar_field.get_minima(), position, 1.0f, 0.0f, 0.0f, 1.0f);
		else
			this->draw_vertices<MAP>(scalar_field.get_minima(), position, 1.0f, 0.0f, 0.0f, 0.1f);
		this->draw_vertices<MAP>(scalar_field.get_saddles(), position, 1.0f, 1.0f, 0.0f, 0.4f);
	}

	template <typename MAP>
	void draw_vertices(const std::vector<typename MAP::Vertex>& vertices,
					   const typename MAP::template VertexAttribute<Vec3>& position,
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
