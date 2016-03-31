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

#ifndef GEOMETRY_ALGOS_CURVATOR_H_
#define GEOMETRY_ALGOS_CURVATOR_H_

#include <core/basic/cell.h>

#include <geometry/types/geometry_traits.h>
#include <geometry/types/eigen.h>


#include <cgogn/geometry/algos/length2.h>
#include <cgogn/selection/collector.h>
#include <cgogn/geometry/functions/intersection2.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3>
void normal_cycles_sort_and_set_eigen_components(
		const VEC3& e_val,
		const Eigen::Matrix<typename VEC3::Scalar, 3, 3>& e_vec,
		typename VEC3::Scalar& kmax,
		typename VEC3::Scalar& kmin,
		VEC3& Kmax,
		VEC3& Kmin,
		VEC3& Knormal,
		const VEC3& normal)
{
	// sort eigen components : ev[inormal] has minimal absolute value ; kmin = ev[imin] <= ev[imax] = kmax
	int inormal = 0, imin, imax;
	if (fabs(e_val[1]) < fabs(e_val[inormal]))
		inormal = 1;

	if (fabs(e_val[2]) < fabs(e_val[inormal]))
		inormal = 2;

	imin = (inormal + 1) % 3;
	imax = (inormal + 2) % 3;

	if (e_val[imax] < e_val[imin])
	{
		int tmp = imin;
		imin = imax;
		imax = tmp;
	}

	// set curvatures from sorted eigen components
	// warning : Kmin and Kmax are switched w.r.t. kmin and kmax
	// normal direction : minimal absolute eigen value
	Knormal[0] = e_vec(0,inormal);
	Knormal[1] = e_vec(1,inormal);
	Knormal[2] = e_vec(2,inormal);

	if (Knormal.dot(normal) < 0)
		Knormal *= -1; // change orientation

	// min curvature
	kmin = e_val[imin] ;
	Kmin[0] = e_vec(0,imax);
	Kmin[1] = e_vec(1,imax);
	Kmin[2] = e_vec(2,imax);

	// max curvature
	kmax = e_val[imax] ;
	Kmax[0] = e_vec(0,imin);
	Kmax[1] = e_vec(1,imin);
	Kmax[2] = e_vec(2,imin);
}

template <typename VEC3, typename MAP>
void curvature_normal_cycle_projected(
		const MAP& map,
		const typename MAP::Vertex v,
		const typename VEC3::Scalar radius,
		const typename MAP::template VertexAttributeHandler<VEC3>& position,
		const typename MAP::template VertexAttributeHandler<VEC3>& normal,
		const typename MAP::template EdgeAttributeHandler<typename VEC3::Scalar>& edgeangle,
		const typename MAP::template EdgeAttributeHandler<typename VEC3::Scalar>& edgearea,
		typename MAP::template VertexAttributeHandler<typename VEC3::Scalar>& kmax,
		typename MAP::template VertexAttributeHandler<typename VEC3::Scalar>& kmin,
		typename MAP::template VertexAttributeHandler<VEC3>& Kmax,
		typename MAP::template VertexAttributeHandler<VEC3>& Kmin,
		typename MAP::template VertexAttributeHandler<VEC3>& Knormal)
{
	using Scalar = typename VEC3::Scalar;
	using Matrix3s = Eigen::Matrix<Scalar, 3, 3>;

	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;

	// collect the normal cycle tensor
	selection::CollectorCriterion_VertexWithinSphere<VEC3, MAP> crit(position, radius);
	selection::Collector_Vertices<VEC3, MAP> neigh(map, crit);

	neigh.collectAll(v.dart) ;

	Matrix3s tensor = Matrix3s::Zero();
	//compute normal cycles tensor
	// collect edges inside the neighborhood
	for (Edge e : neigh.getInsideEdges())
	{
		VEC3 ev = cgogn::geometry::vector_from<VEC3, MAP>(neigh.getMap(), e.dart, position);
		tensor += ev * ev.transpose() * edgeangle[e] * (1.0f / ev.norm());
	}

	// collect edges on the border
	for (Dart d : neigh.getBorder())
	{
		Scalar alpha;
		cgogn::geometry::intersection_sphere_edge<VEC3, MAP>(neigh.getMap(), crit.centerPosition, radius, Edge(d), position, alpha);
		VEC3 ev = cgogn::geometry::vector_from<VEC3,MAP>(neigh.getMap(), d, position);
		tensor += ev * ev.transpose() * edgeangle[Edge(d)] * (1.0f / ev.norm()) * alpha;
	}

	tensor /= neigh.computeArea(position, edgearea, radius);

	// project the tensor
	Matrix3s proj;
	proj.setIdentity();
	proj -= normal[v] * normal[v].transpose();
	tensor = proj * tensor * proj;

	// solve eigen problem
	Eigen::SelfAdjointEigenSolver<Matrix3s> solver(tensor);
	const VEC3& ev = solver.eigenvalues();
	const Matrix3s& evec = solver.eigenvectors();

	normal_cycles_sort_and_set_eigen_components<VEC3>(ev,evec,kmax[v],kmin[v],Kmax[v],Kmin[v],Knormal[v],normal[v]);
}

template <typename VEC3, typename MAP>
void curvature_normal_cycles_projected(
		const MAP& map,
		const typename VEC3::Scalar radius,
		const typename MAP::template VertexAttributeHandler<VEC3>& position,
		const typename MAP::template VertexAttributeHandler<VEC3>& normal,
		const typename MAP::template EdgeAttributeHandler<typename VEC3::Scalar>& edgeangle,
		const typename MAP::template EdgeAttributeHandler<typename VEC3::Scalar>& edgearea,
		typename MAP::template VertexAttributeHandler<typename VEC3::Scalar>& kmax,
		typename MAP::template VertexAttributeHandler<typename VEC3::Scalar>& kmin,
		typename MAP::template VertexAttributeHandler<VEC3>& Kmax,
		typename MAP::template VertexAttributeHandler<VEC3>& Kmin,
		typename MAP::template VertexAttributeHandler<VEC3>& Knormal)
{
	map.foreach_cell([&] (typename MAP::Vertex v)
	{
		curvature_normal_cycle_projected<VEC3, MAP>(map, v, radius, position,
																   normal, edgeangle, edgearea,
																   kmax, kmin, Kmax, Kmin, Knormal);
	});
}

} // namespace geometry

} // namespace cgogn

#endif // GEOMETRY_ALGOS_CURVATOR_H_
