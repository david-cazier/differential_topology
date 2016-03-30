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

#ifndef GEOMETRY_FUNCTIONS_INTERSECTION2_H_
#define GEOMETRY_FUNCTIONS_INTERSECTION2_H_

namespace cgogn
{

namespace geometry
{


template <typename VEC3_T>
bool is_point_in_sphere(const VEC3_T& point, const VEC3_T& center, const typename VEC3_T::Scalar& radius)
{
	return (point - center).norm() < radius;
}

template <typename VEC3, typename MAP>
bool intersection_sphere_edge(
		const MAP& map,
		const VEC3& center,
		typename VEC3::Scalar radius,
		typename MAP::Edge e,
		const typename MAP::template VertexAttributeHandler<VEC3>& position,
		typename VEC3::Scalar& alpha)
{
	using Vertex = typename MAP::Vertex;
	using Scalar = typename VEC3::Scalar;

	const VEC3& p1 = position[Vertex(e.dart)];
	const VEC3& p2 = position[Vertex(map.phi1(e))];

	if(cgogn::geometry::is_point_in_sphere(p1, center, radius) && !cgogn::geometry::is_point_in_sphere(p2, center, radius))
	{
		VEC3 p = p1 - center;
		VEC3 qminusp = p2 - center - p;
		Scalar s = p.dot(qminusp);
		Scalar n2 = qminusp.squaredNorm();
		alpha = (- s + sqrt(s*s + n2 * (radius*radius - p.squaredNorm()))) / n2;
		return true ;
	}
	return false ;
}

} // namespace geometry

} // namespace cgogn

#endif // GEOMETRY_FUNCTIONS_INTERSECTION2_H_
