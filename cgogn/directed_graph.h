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

#ifndef DIRECTED_GRAPH_H
#define DIRECTED_GRAPH_H

#include <cgogn/core/cmap/map_base.h>
#include <cgogn/core/utils/masks.h>

namespace cgogn
{

template <typename MAP_TRAITS, typename MAP_TYPE>
class DirectedGraph_T;

template <typename MAP_TRAITS, typename MAP_TYPE>
class IsolatedFilter : public CellFilters
{
private:
	const DirectedGraph_T<MAP_TRAITS, MAP_TYPE>* g_;

public:
	IsolatedFilter(const DirectedGraph_T<MAP_TRAITS, MAP_TYPE>* g) : g_(g)
	{}

	virtual ~IsolatedFilter() {}

protected:

//	bool filter_DART(Cell<Orbit::DART> c) const override
//	{
//		return true;// !g_->is_isolated(c.dart);
//	}


};


template <typename MAP_TRAITS, typename MAP_TYPE>
class DirectedGraph_T : public MapBase<MAP_TRAITS, MAP_TYPE>
{
public:

    static const int PRIM_SIZE = 1;

    using MapTraits = MAP_TRAITS;
    using MapType = MAP_TYPE;
    using Inherit = MapBase<MAP_TRAITS, MAP_TYPE>;
	using Self = DirectedGraph_T<MAP_TRAITS, MAP_TYPE>;

    friend class MapBase<MAP_TRAITS, MAP_TYPE>;
    friend class DartMarker_T<Self>;
    friend class cgogn::DartMarkerStore<Self>;

    using Vertex = Cell<Orbit::PHI21>;
	using Edge = Cell<Orbit::DART>;
	using ConnectedComponent = Cell<Orbit::PHI1>;

    using IsolatedVertex = Vertex;

    template <typename T>
    using ChunkArray = typename Inherit::template ChunkArray<T>;
    template <typename T>
    using ChunkArrayContainer = typename Inherit::template ChunkArrayContainer<T>;

    template <typename T, Orbit ORBIT>
	using Attribute = typename Inherit::template Attribute<T, ORBIT>;
    template <typename T>
	using VertexAttribute = Attribute<T, Vertex::ORBIT>;
    template <typename T>
	using EdgeAttribute = Attribute<T, Edge::ORBIT>;

    using DartMarker = typename cgogn::DartMarker<Self>;
    using DartMarkerStore = typename cgogn::DartMarkerStore<Self>;

    template <Orbit ORBIT>
    using CellMarker = typename cgogn::CellMarker<Self, ORBIT>;

protected:

    ChunkArray<Dart>* phi1_;
    ChunkArray<Dart>* phi_1_;
    ChunkArray<Dart>* phi2_;

    /// boundary marker shortcut
    ChunkArray<bool>* isolated_marker_;

	IsolatedFilter<MAP_TRAITS, MAP_TYPE>* filter_;


    void init()
    {
        phi1_ = this->topology_.template add_attribute<Dart>("phi1");
        phi_1_ = this->topology_.template add_attribute<Dart>("phi_1");
        phi2_ = this->topology_.template add_attribute<Dart>("phi2");

        isolated_marker_ = this->topology_.template add_marker_attribute();

		filter_ = new IsolatedFilter<MAP_TRAITS, MAP_TYPE>(this);
    }

public:
	DirectedGraph_T() : Inherit()
    {
        init();
    }

	~DirectedGraph_T() override
	{
		delete filter_;
	}

	DirectedGraph_T(Self const&) = delete;
	DirectedGraph_T(Self &&) = delete;
    Self& operator=(Self const&) = delete;
    Self& operator=(Self &&) = delete;

    /*!
     * \brief Check the integrity of embedding information
     * \retval true if the embedding is correct, false otherwise
     */
    inline bool check_embedding_integrity()
    {
        return true;
        //        bool results = true;

        //        if(this->template is_embedded<Vertex>())
        //            result = result && this->template is_well_embedded<Vertex>();

        //        return results;
    }

protected:

    /*!
    * \brief Init an newly added dart.
    * The dart is defined as a fixed point for PHI1 and PHI2.
    */
    inline void init_dart(Dart d)
    {
        (*phi1_)[d.index] = d;
        (*phi_1_)[d.index] = d;
        (*phi2_)[d.index] = d;
    }

	inline bool check_integrity(Dart) const
    {
		return true;/*(phi1(phi_1(d)) == d && phi_1(phi1(d)) == d) &&
				(phi2(phi2(d)) == d && phi2(d) != d && ;*/
    }

    /*!
     * \brief Link two darts with the phi1 permutation what either merge or split their orbit(s).
     * @param d: the first dart
     * @param e: the second dart
     * - Before: d->f and e->g
     * - After:  d->g and e->f
     * Join the orbits of dart d and e if they are distinct
     * - Starting from two cycles : d->f->...->d and e->g->...->e
     * - It makes one cycle d->g->...->e->f->...->d
     * If e = g then insert e in the cycle of d : d->e->f->...->d
     * If d and e are in the same orbit of phi1, this orbit is split in two cycles.
     * - Starting with d->g->...e->f->...->d
     * - It makes two cycles : d->f->...->d and e->g->...->e
     */
    void phi1_sew(Dart d, Dart e)
    {
        Dart f = phi1(d);
        Dart g = phi1(e);
        (*phi1_)[d.index] = g;
        (*phi1_)[e.index] = f;
        (*phi_1_)[g.index] = d;
        (*phi_1_)[f.index] = e;
    }

    /*!
     * \brief Remove the successor of a given dart from its permutation
     * @param d a dart
     * - Before: d->e->f
     * - After:  d->f and e->e
     */
    void phi1_unsew(Dart d)
    {
        Dart e = phi1(d);
        Dart f = phi1(e);
        (*phi1_)[d.index] = f;
        (*phi1_)[e.index] = e;
        (*phi_1_)[f.index] = d;
        (*phi_1_)[e.index] = e;
    }

    /**
     * \brief Link dart d with dart e by the phi2 involution
     * @param d,e the darts to link
     *	- Before: d->d and e->e
     *	- After:  d->e and e->d
     */
    inline void phi2_sew(Dart d, Dart e)
    {
        cgogn_assert(phi2(d) == d);
        cgogn_assert(phi2(e) == e);
        (*phi2_)[d.index] = e;
        (*phi2_)[e.index] = d;
    }

    /**
     * \brief Remove the phi2 link between the current dart and its linked dart
     * @param d the dart to unlink
     * - Before: d->e and e->d
     * - After:  d->d and e->e
     */
    inline void phi2_unsew(Dart d)
    {
        Dart e = phi2(d);
        (*phi2_)[d.index] = d;
        (*phi2_)[e.index] = e;
    }

public:
    /*!
     * \brief phi1
     * @param d
     * @return phi1(d)
     */
    inline Dart phi1(Dart d) const
    {
        return (*phi1_)[d.index];
    }

    /*!
     * \brief phi_1
     * @param d
     * @return phi_1(d)
     */
	inline Dart phi_1(Dart d) const
    {
        return (*phi_1_)[d.index];
    }

	/*!
	 * \brief phi2
	 * @param d
	 * @return phi2(d)
	 */
    inline Dart phi2(Dart d) const
    {
        return (*phi2_)[d.index];
    }

protected:
    inline Dart add_vertex_topo()
    {
        Dart d = this->add_dart();
        set_isolated(d,true);

        return d;
    }

public:
    /*!
     * \brief Add an embedded vertex (or dart) in the map.
     * \return The added dart. If the map has DART attributes,
     * the inserted darts are automatically embedded on new attribute elements.
     */
    inline Vertex add_vertex()
    {
        CGOGN_CHECK_CONCRETE_TYPE;

        const Vertex v(add_vertex_topo());

        if (this->template is_embedded<Vertex>())
            this->new_orbit_embedding(v);

        return v;
    }


    /*!
     * \brief Remove a vertex (or dart) from the map.
     */
    inline void remove_vertex(Vertex v)
    {
        CGOGN_CHECK_CONCRETE_TYPE;

        this->remove_dart(v.dart);
    }

protected:
	inline Dart connect_vertices_topo(Dart v1, Dart v2)
    {
		set_isolated(v1, false);
		set_isolated(v2, false);

        phi1_sew(v1, v2);
        phi2_unsew(v1);
        phi2_unsew(v2);
        phi2_sew(v1, v2);

        return v1;
    }

public:
    inline Edge connect_vertices(Vertex v1, Vertex v2)
    {
        CGOGN_CHECK_CONCRETE_TYPE;

		Edge e(connect_vertices_topo(v1.dart, v2.dart));

        if (this->template is_embedded<Edge>())
            this->new_orbit_embedding(e);

        return e;
    }

	inline Vertex merge_vertices(Vertex v1, Vertex v2)
	{
		CGOGN_CHECK_CONCRETE_TYPE;
	}

    inline void erase_edge(Edge e)
    {
        CGOGN_CHECK_CONCRETE_TYPE;
    }

    inline Vertex split_edge(Edge e)
    {
        CGOGN_CHECK_CONCRETE_TYPE;
    }

    inline Edge make_loop(unsigned int n)
    {
        CGOGN_CHECK_CONCRETE_TYPE;

        Vertex last;
        Vertex first;
        Edge result;

        for(unsigned int i = 0; i < n; i++)
        {
            Vertex cur = add_vertex();
            if(last.is_valid())
            {
                result = connect_vertices(last, cur);
            }
            else
            {
                first = cur;
            }
            last = cur;
        }
        connect_vertices(last, first) ;

        return result;
    }

    inline Edge make_polyline(unsigned int n)
    {
        CGOGN_CHECK_CONCRETE_TYPE;

        Vertex last;
        Edge result;

        for(unsigned int i = 0; i < n; i++)
        {
            Vertex cur = add_vertex();
            if(last.is_valid())
            {
                result = connect_vertices(last, cur);
            }
            last = cur ;
        }

        return result ;
    }

    inline bool is_isolated(Dart d) const
    {
        return (*isolated_marker_)[d.index];
    }

    inline void set_isolated(Dart d, bool b)
    {
        isolated_marker_->set_value(d.index, b);
    }

	template <typename FUNC>
    inline void foreach_cell(const FUNC& f) const
	{
		Inherit::foreach_cell(f, *filter_);
    }

protected:

    template <typename FUNC>
	inline void foreach_dart_of_DART(Dart d, const FUNC& f) const
    {
		f(d);
    }

    template <typename FUNC>
    inline void foreach_dart_of_PHI21(Dart d, const FUNC& f) const
    {
        Dart it = d;
        do
        {
            f(it);
            it = phi2(this->phi_1(it));
        } while (it != d);
    }

    template <Orbit ORBIT, typename FUNC>
    inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
    {
        static_assert(check_func_parameter_type(FUNC, Dart), "Wrong function parameter type");
        static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 || ORBIT == Orbit::PHI2 ||
                      ORBIT == Orbit::PHI1_PHI2 || ORBIT == Orbit::PHI21,
                      "Orbit not supported in a Graph");

        switch (ORBIT)
        {
			case Orbit::DART: foreach_dart_of_DART(c.dart, f); break;
			case Orbit::PHI21: foreach_dart_of_PHI21(c.dart, f); break;
			case Orbit::PHI2:
            case Orbit::PHI1:
            case Orbit::PHI1_PHI2:
            case Orbit::PHI2_PHI3:
            case Orbit::PHI1_PHI3:
            case Orbit::PHI21_PHI31:
            default: cgogn_assert_not_reached("Orbit not supported in a Graph"); break;
        }
    }

};

template <typename MAP_TRAITS>
struct DirectedGraphType
{
	using TYPE = DirectedGraph_T<MAP_TRAITS, DirectedGraphType<MAP_TRAITS>>;
};

template <typename MAP_TRAITS>
using DirectedGraph = DirectedGraph_T<MAP_TRAITS, DirectedGraphType<MAP_TRAITS>>;

}

#endif // DIRECTED_GRAPH_H
