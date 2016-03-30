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

#ifndef CORE_UTILS_NUMERIC_H_
#define CORE_UTILS_NUMERIC_H_


#include <cstdint>
#include <cmath>
#include <algorithm>

namespace cgogn
{

//\todo move the entire numerics namespace to numeric.h
//\todo move functions in precision.h to numeric.h
namespace numeric
{

template <typename F, typename I>
inline F scale_expand_within_0_1(const F x, const I n)
{
		for (I i = 1; i <= n; i++)
				x = F((1.0 - cos(M_PI * x)) / 2.0);
		for (I i = -1; i >= n; i--)
				x = F(acos(1.0 - 2.0 * x) / M_PI);
		return x;
}

template <typename F, typename I>
inline F scale_expand_towards_1(const F x, const I n)
{
		for (I i = 1; i <= n; i++)
				x = F(sin(x * M_PI / 2.0));
		for (I i = -1; i >= n; i--)
				x = F(asin(x) * 2.0 / M_PI);
		return x;
}

template <typename F>
inline F scale_to_0_1(const F x, const F min, const F max)
{
  return (x - min) / (max - min);
}

template <typename F>
inline F scale_and_clamp_to_0_1(const F x, const F min, const F max)
{
		F v = (x - min) / (max - min);
		return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

template <typename F>
inline void scale_centering_around_0(F& min, F& max)
{
		F new_max = std::max(max, -min);
		min = std::min(min, -max);
		max = new_max;
}

template <typename F>
inline F scale_to_0_1_around_one_half(const F x, const F min, const F max)
{
		F ma = std::max(max, -min);
		F mi = std::min(min, -max);
		return (x - mi) / (ma - mi);
}

} // namespace numerics

} // namespace cgogn

#endif // CORE_UTILS_NUMERIC_H_

