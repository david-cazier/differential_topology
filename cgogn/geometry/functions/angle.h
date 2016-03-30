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

#ifndef GEOMETRY_FUNCTIONS_ANGLE_H_
#define GEOMETRY_FUNCTIONS_ANGLE_H_

#include <cmath>

namespace cgogn
{

namespace geometry
{

/**
 * cosinus of the angle formed by 2 vectors
 */
template <typename VEC3_T>
inline typename VEC3_T::Scalar cos_angle(const VEC3_T& p1, const VEC3_T& p2)
{
    typename VEC3_T::Scalar np1 = p1.norm2();
    typename VEC3_T::Scalar np2 = p2.norm2();

    typename VEC3_T::Scalar res = p1.dot(p2) / std::sqrt(np1 * np2);
    return res > VEC3_T::Scalar(1.0) ? VEC3_T::Scalar(1.0) : (res < VEC3_T::Scalar(-1.0) ? VEC3_T::Scalar(-1.0) : res);
}

/**
 * angle formed by 2 vectors
 */
template <typename VEC3_T>
inline typename VEC3_T::Scalar angle(const VEC3_T& p1, const VEC3_T& p2)
{
		return std::acos(cos_angle(p1,p2)) ;
}


} // namespace geometry

} // namespace cgogn

#endif // GEOMETRY_FUNCTIONS_ANGLE_H_
