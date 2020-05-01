#pragma once
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <iostream>  // for std::cout
#include <utility>   // for std::pair
#include <algorithm> // for std::for_each

using namespace boost;
typedef adjacency_list<listS, vecS, undirectedS,
                       no_property, property<edge_weight_t, double>>
    Graph;

typedef std::pair<int, int> Edge;

namespace Lobo
{
class LoboTetMesh;

class LoboTetMeshGraph
{
public:
    LoboTetMeshGraph(LoboTetMesh *tetmesh);
    ~LoboTetMeshGraph();

    virtual void init();

    virtual void compute_dijkstra_shortest_paths(int startnode,std::vector<double> &distance);


protected:

    virtual void initEdgeList();

    LoboTetMesh* tetmesh;
    Graph* boost_graph;
    std::vector<Edge> edge_list;
    std::vector<double> edge_weights;
};

} // namespace Lobo