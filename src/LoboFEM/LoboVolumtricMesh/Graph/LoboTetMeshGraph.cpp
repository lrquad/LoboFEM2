#include "LoboTetMeshGraph.h"
#include "LoboVolumtricMesh/LoboTetMesh.h"

typedef graph_traits<Graph>::vertex_descriptor Vertex;


Lobo::LoboTetMeshGraph::LoboTetMeshGraph(LoboTetMesh *tetmesh_)
{
    this->tetmesh = tetmesh_;
}

Lobo::LoboTetMeshGraph::~LoboTetMeshGraph()
{
    delete boost_graph;
}

void Lobo::LoboTetMeshGraph::init()
{
    int numVertex = tetmesh->getNumVertex();
    initEdgeList();
    boost_graph = new Graph(edge_list.begin(), edge_list.end(), edge_weights.begin(), numVertex);
}

void Lobo::LoboTetMeshGraph::compute_dijkstra_shortest_paths(int startnode, std::vector<double> &distance)
{
    int numVertex = tetmesh->getNumVertex();
	distance.resize(numVertex);
	Vertex s = *(vertices(*boost_graph).first + startnode);
	dijkstra_shortest_paths(*boost_graph, s, distance_map(&distance[0]));
}

void Lobo::LoboTetMeshGraph::initEdgeList()
{
    edge_list.clear();
    int numVertex = tetmesh->getNumVertex();
    for(int i=0;i<numVertex;i++)
    {
        int nodeid = i;
        NodeData* node = tetmesh->getTetNode(nodeid);
        int neighborsize = node->neighbor.size();
        for(int j=0;j<neighborsize;j++)
        {
            int neighborid = node->neighbor[j];
            //avoid insert same edge
			if (neighborid > nodeid)
			{
				edge_list.push_back(Edge(nodeid, neighborid));
				double distance = (tetmesh->getNodeRestPosition(nodeid) - tetmesh->getNodeRestPosition(neighborid)).norm();
				edge_weights.push_back(distance);
			}
        }
    }

}