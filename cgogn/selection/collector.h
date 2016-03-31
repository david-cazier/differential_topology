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

#ifndef SELECTION_COLLECTOR_H_
#define SELECTION_COLLECTOR_H_

#include <core/basic/dart.h>
#include <core/basic/cell_marker.h>

#include <cgogn/selection/collector_criterion.h>

#include <cgogn/geometry/functions/intersection2.h>

namespace cgogn
{

namespace selection
{

template <typename VEC3, typename MAP>
class Collector
{
protected:
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	template <typename T>
	using VertexAttributeHandler = typename MAP::template VertexAttributeHandler<T>;
	template <typename T>
	using EdgeAttributeHandler = typename MAP::template EdgeAttributeHandler<T>;

	using Scalar = typename VEC3::Scalar;

protected:
	const MAP& map;

	Dart centerDart;

	bool isInsideCollected;

	std::vector<Vertex> insideVertices;
	std::vector<Edge> insideEdges;
	std::vector<Face> insideFaces;
	std::vector<Dart> border;

public:
	Collector(const MAP& m) : map(m), isInsideCollected(false)
	{}

	inline void init(Dart d)
	{
		centerDart = d;
		isInsideCollected = false;
		insideVertices.clear();
		insideEdges.clear();
		insideFaces.clear();
		border.clear();
	}

	virtual void collectAll(Dart d) = 0;
	virtual void collectBorder(Dart d) = 0;

	template <typename FUNC>
	void applyOnInsideVertices(FUNC& func)
	{
		assert(isInsideCollected || !"applyOnInsideVertices: inside cells have not been collected.") ;
		for (Vertex v : insideVertices)
			func(v.dart);
	}

	template <typename FUNC>
	void applyOnInsideEdges(FUNC& func)
	{
		assert(isInsideCollected || !"applyOnInsideEdges: inside cells have not been collected.") ;
		for (Edge e : insideEdges)
			func(e.dart);
	}


	template <typename FUNC>
	void applyOnInsideFaces(FUNC& func)
	{
		assert(isInsideCollected || !"applyOnInsideFaces: inside cells have not been collected.") ;
		for (Face f : insideFaces)
			func(f.dart);
	}

	template <typename FUNC>
	void applyOnBorder(FUNC& func)
	{
		for (Dart d : border)
			func(d);
	}

	inline void sort()
	{
		std::sort(insideVertices.begin(), insideVertices.end());
		std::sort(insideEdges.begin(), insideEdges.end());
		std::sort(insideFaces.begin(), insideFaces.end());
		std::sort(border.begin(), border.end());
	}

	inline const MAP& getMap() { return map; }

	inline Dart getCenterDart() const { return centerDart; }

	inline const std::vector<Vertex>& getInsideVertices() const { assert(isInsideCollected || !"getInsideVertices: inside cells have not been collected.") ; return insideVertices; }
	inline const std::vector<Edge>& getInsideEdges() const { assert(isInsideCollected || !"getInsideEdges: inside cells have not been collected.") ; return insideEdges; }
	inline const std::vector<Face>& getInsideFaces() const { assert(isInsideCollected || !"getInsideFaces: inside cells have not been collected.") ; return insideFaces; }
	inline const std::vector<Dart>& getBorder() const { return border; }

	inline unsigned int getNbInsideVertices() const { assert(isInsideCollected || !"getNbInsideVertices: inside cells have not been collected.") ; return insideVertices.size(); }
	inline unsigned int getNbInsideEdges() const { assert(isInsideCollected || !"getNbInsideEdges: inside cells have not been collected.") ; return insideEdges.size(); }
	inline unsigned int getNbInsideFaces() const { assert(isInsideCollected || !"getNbInsideFaces: inside cells have not been collected.") ; return insideFaces.size(); }
	inline unsigned int getNbBorder() const { return border.size(); }

	virtual Scalar computeArea(const VertexAttributeHandler<VEC3>& /*pos*/)
	{
		assert(!"Warning: Collector<PFP>::computeArea() should be overloaded in non-virtual derived classes");
		return 0.0;
	}

	virtual Scalar computeArea(const VertexAttributeHandler<VEC3>& /*pos*/, const EdgeAttributeHandler<Scalar>& /*edgearea*/)
	{
		assert(!"Warning: Collector<PFP>::computeArea() should be overloaded in non-virtual derived classes");
		return 0.0;
	}

	virtual Scalar borderEdgeRatio(Dart /*d*/, const VertexAttributeHandler<VEC3>& /*pos*/)
	{
		assert(!"Warning: Collector<PFP>::borderEdgeRatio() should be overloaded in non-virtual derived classes");
		return 1.0;
	}
};


/*********************************************************
 * Collector Vertices
 *********************************************************/

/*
 * collect all vertices of the connected component containing "centerDart"
 * that satisfy the CollectorCriterion
 * (hopefully) it defines a 2-manifold (if inserting border-vertices along the border-edges)
 */
template <typename VEC3, typename MAP>
class Collector_Vertices : public Collector<VEC3, MAP>
{
	using Inherit = Collector<VEC3, MAP>;

	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	template <typename T>
	using VertexAttributeHandler = typename Inherit::template VertexAttributeHandler<T>;
	template <typename T>
	using EdgeAttributeHandler = typename Inherit::template EdgeAttributeHandler<T>;

	using Scalar = typename Inherit::Scalar;

protected:
	CollectorCriterion & crit;

public:
	Collector_Vertices(const MAP& m, CollectorCriterion& c) :
		Collector<VEC3, MAP>(m),
		crit(c)
	{}
	void collectAll(Dart d)
	{
		crit.init(d);
		this->init(d);
		this->isInsideCollected = true;
		this->insideEdges.reserve(32);
		this->insideFaces.reserve(32);
		this->border.reserve(32);

		CellMarkerStore<MAP, Vertex::ORBIT> vm(this->map);	// mark the collected inside-vertices
		CellMarkerStore<MAP, Edge::ORBIT> em(this->map);	// mark the collected inside-edges + border-edges
		CellMarkerStore<MAP, Face::ORBIT> fm(this->map);	// mark the collected inside-faces + border-faces

		Vertex vc(this->centerDart);
		this->insideVertices.push_back(vc);
		vm.mark(vc);

		unsigned int i = 0;
		while (i < this->insideVertices.size())
		{
			Dart end = this->insideVertices[i].dart;
			Dart e = end;
			do
			{
				if (! em.is_marked(Edge(e)) || ! fm.is_marked(Face(e))) // are both tests useful ?
				{
					const Dart f = this->map.phi1(e);
					const Dart g = this->map.phi1(f);

					if (! crit.is_inside(f))
					{
						this->border.push_back(e); // add to border
						em.mark(Edge(e));
						fm.mark(Face(e)); // is it useful ?
					}
					else
					{
						if (! vm.is_marked(Vertex(f)))
						{
							this->insideVertices.push_back(Vertex(f));
							vm.mark(Vertex(f));
						}
						if (! em.is_marked(Edge(e)))
						{
							this->insideEdges.push_back(Edge(e));
							em.mark(Edge(e));
						}
						if (! fm.is_marked(Face(e)) && crit.is_inside(g))
						{
							this->insideFaces.push_back(Face(e));
							fm.mark(Face(e));
						}
					}
				}
				e = this->map.phi2(this->map.phi_1(e));
			} while (e != end);
			++i;
		}
	}


	void collectBorder(Dart d)
	{
		crit.init(d);
		this->init(d);
		this->border.reserve(128);
		this->insideVertices.reserve(128);

		CellMarkerStore<MAP, Vertex::ORBIT> vm(this->map);	// mark the collected inside-vertices
		CellMarkerStore<MAP, Edge::ORBIT> em(this->map);	// mark the collected inside-edges + border-edges

		Vertex vc(this->centerDart);
		this->insideVertices.push_back(vc);
		vm.mark(vc);

		unsigned int i = 0;
		while (i < this->insideVertices.size())
		{
			Dart end = this->insideVertices[i].dart;
			Dart e = end;
			do
			{
				if ( ! em.is_marked(Edge(e)) )
				{
					const Dart f = this->map.phi1(e);

					if (! crit.is_inside(f))
					{
						this->border.push_back(e); // add to border
					}
					else
					{
						if (! vm.is_marked(Vertex(f)))
						{
							this->insideVertices.push_back(Vertex(f));
							vm.mark(Vertex(f));
						}
					}
					em.mark(Edge(e));
				}
				e = this->map.phi2(this->map.phi_1(e));
			} while (e != end);
			++i;
		}
		this->insideVertices.clear();
	}

	Scalar computeArea(const VertexAttributeHandler<VEC3>& pos, const EdgeAttributeHandler<Scalar>& edgearea, Scalar radius)
	{
		assert(this->isInsideCollected || !"computeArea: inside cells have not been collected.") ;

		const VEC3& centerPosition = pos[Vertex(this->centerDart)];

		Scalar area(0);

		for (Edge e : this->insideEdges)
			area += edgearea[e];

		for (Dart d : this->border)
		{
			Scalar alpha;
			cgogn::geometry::intersection_sphere_edge<VEC3, MAP>(this->map, centerPosition, radius, Edge(d), pos, alpha);
			area += alpha * edgearea[Edge(d)];
		}

		return area;
	}
};


} //namespace selection

} // namespace cgogn

#endif // SELECTION_COLLECTOR_H_
