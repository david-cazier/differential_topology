#ifndef FEATURE_POINTS_EXTRACTION_H
#define FEATURE_POINTS_EXTRACTION_H

//#include "Algo/Geometry/centroid.h"
#include <limits>
//#include "Algo/Geometry/basic.h"
#include "dijkstra.h"
#include "pl_functions.h"


namespace CGoGN
{



template <typename MAP, typename V_ATT>
typename V_ATT::DATA_TYPE surfaceCentroid(MAP& map, const V_ATT& attributs)
{

    typename V_ATT::DATA_TYPE center(0.0);
    unsigned int count = 0 ;

    foreach_cell<VERTEX>(map, [&] (Vertex v)
    {
        center += attributs[v];
        ++count;
    });

    center /= float(count) ;
    return center ;
}



//void compute_feature_vertices( )

template <typename PFP>
void extract_feature_points(typename PFP::MAP& map, VertexAttribute<typename PFP::VEC3, typename PFP::MAP>& position, std::vector<Dart>& vertices)
{
    typedef typename PFP::MAP MAP;
    typedef typename PFP::VEC3 VEC3;
    typedef typename PFP::REAL REAL;


    VEC3 barycenter = surfaceCentroid(map, position);

    //1. compute v0: the vertex whose distance to the barycenter of map is minimal
    double dist_v0 = std::numeric_limits<double>::max();
    Vertex v0 = NIL;

    foreach_cell<VERTEX>(map, [&](Vertex v)
    {
        VEC3 origin = position[v];
        origin -= barycenter;
        double dist = origin.norm();

        if(dist < dist_v0)
        {
            dist_v0 = dist;
            v0 = v;
        }
    });

    //2. map the vertices to their geodesic distance to v0
    // find the vertex v1 that maximizes f0
//    double dist_v1 = 0.0;
//    Vertex v1 = NIL;
//    VertexAttribute<double, MAP> f0 = map.template addAttribute<double, VERTEX, MAP>("f0");

//    foreach_cell<VERTEX>(map, [&](Vertex v)
//    {
//        //double dist_v = geodesic_distance<PFP>(map, position, v0, v);
//        f0[v] = dist_v;

//        if(dist_v > dist_v1)
//        {
//            dist_v1 = dist_v;
//            v1 = v;
//        }
//    });

//    foreach_cell<VERTEX>(map, [&](Vertex v)
//    {
//        int i = is_critical_vertex<PFP>(map,v,f0);
//        if(i == 1)
//            vertices.push_back(v);
//    });

    //    vertices.push_back(v1);

    //    //3. map the vertices to their geodesic distance to v1
    //    // find the vertex v2  that maximized f1
    //    double dist_v2 = 0.0;
    //    Vertex v2 = NIL;
    //    VertexAttribute<double, MAP> f1 = map.template addAttribute<double, VERTEX, MAP>("f1");

    //    foreach_cell<VERTEX>(map, [&](Vertex v)
    //    {
    //        double dist_v = geodesic_distance<PFP>(map, position, v1, v);
    //        f1[v] = dist_v;

    //        if(dist_v > dist_v2)
    //        {
    //            dist_v2 = dist_v;
    //            v2 = v;
    //        }
    //    });

    //vertices.push_back(v2);
}


}

#endif // FEATURE_POINTS_EXTRACTION_H
