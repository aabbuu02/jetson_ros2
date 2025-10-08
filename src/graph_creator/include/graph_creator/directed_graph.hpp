#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>

#include <ros/ros.h>

#include "graph_data_types.hpp"

namespace Graph
{
    class DiGraph
    {
        public:

        DiGraph();
        ~DiGraph();

        //Graph
        bool isEmpty();
        void clear();

        //Vertex
        void addVertex(const std::string &name, const Vertex &v=Vertex());
        bool deleteVertex(const std::string &name);
        bool getVertexToEdgeDependency(const std::string &vertex_name, EdgeDepSet &eds); //Returns all edges connected to a vertex
        bool isVertex(const std::string &vertex_name);
        VertexPtr getVertexData(const std::string &vertex_name);
        VerticesMapPtr getVertices();
        
        //Edge
        bool addEdge(const std::string &edge_name, const std::string &src_vertex_name, const std::string &dst_vertex_name, const Edge &e=Edge());
        bool deleteEdge(const std::string &edge_name);
        bool getEdgeToVertexDependency(const std::string &edge_name, VertexDepPair &vdp); //Returns all vertices connected to an edge
        bool isEdge(const std::string &edge_name);
        EdgePtr getEdgeData(const std::string &edge_name); 
        EdgesMapPtr getEdges();
        //TODO: Delete edges given two vertices only (Take the symmetric set difference between two sets and delete the common edge)

        //Miscellaneous graph elements dependency
        void addMiscDependency(const std::string &key_element, const std::string &value_element, bool mutual_dep=false);
        bool deleteMiscDependencyKey(const std::string &key_element);
        bool deleteMiscDependencyValue(const std::string &key_element, const std::string &value_element);
        bool getMiscDependency(const std::string &key_element, MiscDepSet &mds);
        bool isMiscellaneous(const std::string &key_element);
        //TODO: Deletes elements based on whether existing vertices and edges requires them (For deleting all graph element relations)  

        void printGraph(bool verbose=false) const;
        void printVertices() const;
        void printEdges() const;
        void printVertexToEdgesDeps() const;
        void printEdgeToVerticesDeps() const;
        void printMiscellaneousDeps() const;
        void printAdditionalInfo() const;

        private:

        Graph::AdjacencyListMap m_graph;

        //Vertices
        Graph::VerticesMap m_vertices;
        
        //Edges
        Graph::EdgesMap m_edges;

        //Vertex and edge dependencies
        Graph::VertexToEdgesDepMap m_v2e_dep;
        Graph::EdgeToVerticesDepMap m_e2v_dep;

        //Miscellaneous dependencies
        Graph::MiscDepMap m_misc_dep;
    };
}
