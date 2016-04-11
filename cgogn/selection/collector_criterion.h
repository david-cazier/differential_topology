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

#ifndef SELECTION_COLLECTOR_CRITERION_H_
#define SELECTION_COLLECTOR_CRITERION_H_

#include <cgogn/core/basic/dart.h>

namespace cgogn
{

namespace selection
{

class CollectorCriterion
{
public:
	CollectorCriterion() {}
	virtual ~CollectorCriterion() {}
	virtual void init(Dart center) = 0;
	virtual bool is_inside(Dart d) = 0;
};

// tests if the distance between vertices is below some threshold
template <typename VEC3, typename MAP>
class CollectorCriterion_VertexWithinSphere : public CollectorCriterion
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	template <typename T>
	using VertexAttribute = typename MAP::template VertexAttribute<T>;
	template <typename T>
	using EdgeAttribute = typename MAP::template EdgeAttribute<T>;

	using Scalar = typename VEC3::Scalar;

private:
	const VertexAttribute<VEC3>& vertexPositions;
	Scalar threshold;

public:
	VEC3 centerPosition;

public:
	CollectorCriterion_VertexWithinSphere(const VertexAttribute<VEC3>& p, Scalar th) :
		vertexPositions(p), threshold(th)//, centerPosition(0)
	{}

	void init(Dart center)
	{
		centerPosition = vertexPositions[Vertex(center)];
	}

	bool is_inside(Dart d)
	{
		return (vertexPositions[Vertex(d)] - centerPosition).norm() < threshold ;
	}
};


} //namespace selection

} // namespace cgogn

#endif // SELECTION_COLLECTOR_CRITERION_H_
