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

#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <map>
#include <set>
#include <vector>
#include <algorithm>


namespace cgogn
{


/*
* adapted from http://rosettacode.org/wiki/Dijkstra's_algorithm#C.2B.2B
* \todo replace Dart by Vertex in previous attribute
*/
template <typename T, typename MAP>
void dijkstra_compute_paths(
        MAP& map,
        const typename MAP::template EdgeAttributeHandler<T>& weight,
        const typename MAP::Vertex source,
        typename MAP::template VertexAttributeHandler<T>& min_distance,
        typename MAP::template VertexAttributeHandler<Dart>& previous)
{
   using Vertex = typename MAP::Vertex;
   using Edge = typename MAP::Edge;

   T max_weight = std::numeric_limits<T>::infinity();

   for(auto& d : min_distance)
       d = max_weight;

   for(auto& p : previous)
       p = Dart();

   min_distance[source] = T(0.0);

   std::set<std::pair<T, unsigned int> > vertex_queue;
   vertex_queue.insert(std::make_pair(min_distance[source], source.dart.index));

   while(!vertex_queue.empty())
   {
       double dist = vertex_queue.begin()->first;
       Dart u = Dart(vertex_queue.begin()->second);

       vertex_queue.erase(vertex_queue.begin());

      map.foreach_adjacent_vertex_through_edge(Vertex(u), [&](Vertex v)
      {
           double distance_through_u = dist + weight[Edge(v)];
           if(distance_through_u < min_distance[v])
           {
               vertex_queue.erase(std::make_pair(min_distance[v], v.dart.index));

               min_distance[v] = distance_through_u;
               previous[v] = u;

               vertex_queue.insert(std::make_pair(min_distance[v], v.dart.index));
           }
      });
   }
}

template <typename T, typename MAP>
void dijkstra_compute_normalized_paths(
        MAP& map,
        const typename MAP::template EdgeAttributeHandler<T>& weight,
        const typename MAP::Vertex source,
        typename MAP::template VertexAttributeHandler<T>& min_distance,
        typename MAP::template VertexAttributeHandler<Dart>& previous)
{
    dijkstra_compute_paths<T>(map, weight, source, min_distance, previous);

    //find max of min_distance
    double max_d = std::numeric_limits<double>::min();
    double min_d = std::numeric_limits<double>::max();

    for(auto& d : min_distance)
    {
        max_d = std::max(max_d, d);
        min_d = std::min(min_d, d);
    }

    //normalize
    for(auto& d : min_distance)
        d = (d - min_d) / (max_d - min_d);
}

template <typename T, typename MAP>
Dart argmin(MAP& map,
            const typename MAP::template VertexAttributeHandler<T>& attribut)
{
    using Vertex = typename MAP::Vertex;

    double min = std::numeric_limits<double>::infinity();
    Dart d_min;
    map.foreach_cell([&] (Vertex v)
    {
        double cur = attribut[v];
        if(cur < min)
            d_min = v;
    });

    return d_min;
}

template <typename T, typename MAP>
Dart argmin(std::map<unsigned int, Dart> v,
            const typename MAP::template VertexAttributeHandler<T>& attribut)
{
    using Vertex = typename MAP::Vertex;

    double min = std::numeric_limits<double>::infinity();
    Dart d_min;
    for(std::map<unsigned int, Dart>::iterator it = v.begin() ; it != v.end() ; ++it)
    {
        Dart v = it->second  ;
        double cur = attribut[Vertex(v)];
        if(cur < min)
            d_min = v;
    }

    return d_min;
}

//Function value driven dijkstra based function
template <typename T, typename MAP>
void dijkstra_compute_perturbated_function(
        MAP& map,
        const typename MAP::template VertexAttributeHandler<T>& f,
        typename MAP::template VertexAttributeHandler<T>& f_pertubated)
{
    using Vertex = typename MAP::Vertex ;

    unsigned int i = 0;
    std::map<unsigned int, Dart> visited;
    std::map<unsigned int, Dart> candidates;

    Dart dmin = argmin<T>(map, f);
    candidates.insert(std::pair<unsigned int, Dart>(map.template get_embedding(Vertex(dmin)), dmin));

    unsigned int Nv = map.template nb_cells<Vertex::ORBIT>();

    do
    {
        Dart vt = argmin<T,MAP>(candidates, f);

		f_pertubated[Vertex(vt)] = static_cast<T>(i) / static_cast<T>(Nv);

        candidates.erase(map.template get_embedding(Vertex(vt)));

        visited.insert(std::pair<unsigned int, Dart>(map.template get_embedding(Vertex(vt)), vt));

        map.foreach_adjacent_vertex_through_edge(Vertex(vt), [&](Vertex v){
            if(visited.find(map.template get_embedding(Vertex(v))) == visited.end())
                candidates.insert(std::pair<unsigned int, Dart>(map.template get_embedding(Vertex(v)), v));
        });

        ++i;
    }
    while(!candidates.empty());
}

template <typename MAP>
void dijkstra_compute_shortest_path_to(
        const typename MAP::Vertex v,
        const typename MAP::template VertexAttributeHandler< typename MAP::Vertex>& previous,
        std::vector<Dart>& path)
{
    using Vertex = typename MAP::Vertex;

   path.clear();
   Dart source = v ;
   Dart end = Dart();

   for( ; source != end; source = previous[Vertex(source)])
   {
       path.push_back(source);
   }

   path.push_back(v);
}

}
#endif // DIJKSTRA_H
