
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

#ifndef RENDERING_GRAPH_RENDER_H_
#define RENDERING_GRAPH_RENDER_H_

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>

#include <core/cmap/map_base.h> // impossible to include directly attribute_handler.h !
#include <geometry/algos/ear_triangulation.h>
#include <rendering/drawer.h>

#include <rendering/shaders/vbo.h>

namespace cgogn
{

namespace rendering
{

//enum DrawingType
//{
//		POINTS = 0,
//		LINES,
//		SIZE_BUFFER
//};

class GraphRender
{
protected:

		QOpenGLBuffer* indices_buffers_[SIZE_BUFFER];
		bool indices_buffers_uptodate_[SIZE_BUFFER];
		unsigned int nb_indices_[SIZE_BUFFER];

public:

		inline GraphRender()
		{
				for (unsigned int i = 0u; i < SIZE_BUFFER; ++i)
				{
						indices_buffers_[i] = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
						indices_buffers_[i]->setUsagePattern(QOpenGLBuffer::StaticDraw);
				}
		}

		inline ~GraphRender()
		{
				for (unsigned int i = 0u; i < SIZE_BUFFER; ++i)
						delete indices_buffers_[i];
		}

		inline bool is_primitive_uptodate(DrawingType prim)  { return indices_buffers_uptodate_[prim]; }

		template <typename MAP>
		void init_points(MAP& m, std::vector<unsigned int>& table_indices)
		{
				//		table_indices.reserve(m.get_nb_darts()/6);
				m.foreach_cell([&] (typename MAP::Vertex v)
				{
						table_indices.push_back(m.get_embedding(v));
				});
		}

		template <typename MAP>
		void init_lines(MAP& m, std::vector<unsigned int>& table_indices)
		{
				using Vertex = typename MAP::Vertex;
				using Edge = typename MAP::Edge;
				//		table_indices.reserve(m.get_nb_darts()/2);
				m.foreach_cell([&] (Edge e)
				{
						table_indices.push_back(m.get_embedding(Vertex(e.dart)));
						table_indices.push_back(m.get_embedding(Vertex(m.phi1(e.dart))));
				});
		}

		template <typename VEC3, typename MAP>
		void init_primitives(MAP& m, DrawingType prim, const typename MAP::template VertexAttributeHandler<VEC3>& position)
		{
				std::vector<unsigned int> table_indices;

				switch (prim)
				{
						case POINTS:
								init_points(m, table_indices);
								break;
						case LINES:
								init_lines(m, table_indices);
								break;
						default:
								break;
				}

				if (!indices_buffers_[prim]->isCreated())
						indices_buffers_[prim]->create();

				indices_buffers_uptodate_[prim] = true;
				nb_indices_[prim] = static_cast<unsigned int>(table_indices.size());
				indices_buffers_[prim]->bind();
				indices_buffers_[prim]->allocate(&(table_indices[0]), nb_indices_[prim] * sizeof(unsigned int));
				indices_buffers_[prim]->release();
		}

		inline void draw(DrawingType prim)
		{
				QOpenGLFunctions *ogl = QOpenGLContext::currentContext()->functions();

				indices_buffers_[prim]->bind();
				switch (prim)
				{
						case POINTS:
								ogl->glDrawElements(GL_POINTS, nb_indices_[POINTS], GL_UNSIGNED_INT, 0);
								break;
						case LINES:
								ogl->glDrawElements(GL_LINES, nb_indices_[LINES], GL_UNSIGNED_INT, 0);
								break;
						default:
								break;
				}

				indices_buffers_[prim]->release();
		}
};



} // namespace rendering

} // namespace cgogn

#endif // RENDERING_GRAPH_RENDER_H_
