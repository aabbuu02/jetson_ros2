#include <graph_creator/directed_graph.hpp>

namespace Graph
{
    DiGraph::DiGraph()
    {

    }


    DiGraph::~DiGraph()
    {

    }

    bool DiGraph::isEmpty()
    {
        return m_graph.empty();
    }

    void DiGraph::clear()
    {
        m_graph.clear();
        m_vertices.clear();
        m_edges.clear();
        m_v2e_dep.clear();
        m_e2v_dep.clear();
        m_misc_dep.clear();
    }

    void DiGraph::addVertex(const std::string &vertex_name, const Vertex &v)
    {
        if(m_graph.find(vertex_name) == m_graph.end())
        {
            //Adding to vertex tracking map
            m_vertices.emplace(vertex_name, std::make_shared<Vertex>(v));
            
            //Adding to vertex to edges dependency map
            EdgeDepSet es_dep;
            m_v2e_dep.emplace(vertex_name, es_dep);

            //Adding to graph
            AdjacencyVertexSet vs;
            m_graph.emplace(vertex_name, vs);
            ROS_INFO_STREAM("Added vertex "<<v.vertex_name);
            return;
        }

        ROS_WARN_STREAM("Vertex "<<vertex_name<<" already present");
    }

    bool DiGraph::deleteVertex(const std::string &vertex_name)
    {
        if(m_graph.find(vertex_name) != m_graph.end())
        {
            //Deleting from vertex tracking map
            m_vertices.erase(vertex_name);

            //TODO: Can be optimized and simplified
            //Deleting from vertex to edge dependency map 
            EdgeDepSet es_dep = m_v2e_dep.find(vertex_name)->second; //For deleting edges from the deleted vertex
            for(EdgeDepSet::iterator es_dep_it = es_dep.begin(); es_dep_it != es_dep.end(); ++es_dep_it)
            {
                VertexDepPair v_dep = m_e2v_dep.at(*es_dep_it);

                //For deleting edge dependency to connecting vertices from the deleted vertex
                if(v_dep.first != vertex_name) m_v2e_dep.find(v_dep.first)->second.erase(*es_dep_it);
                if(v_dep.second != vertex_name) m_v2e_dep.find(v_dep.second)->second.erase(*es_dep_it);

                m_edges.erase(*es_dep_it); //Deleting from the edge tracking map
                m_e2v_dep.erase(*es_dep_it); //Deleting the edge from the edge to vertices dependency map
                m_misc_dep.erase(*es_dep_it); //Deleting the edge from the miscellaneous dependency map
            }

            m_v2e_dep.erase(vertex_name); //Deleting the vertex from the vertex to edge dependency map
            m_misc_dep.erase(vertex_name); //Deleting the vertex from the miscellaneous dependency map

            //Deleting from graph
            m_graph.erase(vertex_name);

            for(AdjacencyListMap::iterator it = m_graph.begin(); it != m_graph.end(); ++it)
            {
                if(it->second.find(vertex_name) != it->second.end()) 
                    it->second.erase(vertex_name);
            }

            ROS_INFO_STREAM("Vertex "<<vertex_name<<" deleted");
            return true;
        }

        ROS_WARN_STREAM("Vertex "<<vertex_name<<" not present");
        return false;
    }

    bool DiGraph::getVertexToEdgeDependency(const std::string &vertex_name, EdgeDepSet &eds)
    {
        if(m_v2e_dep.find(vertex_name) != m_v2e_dep.end())
        {
            eds = m_v2e_dep.at(vertex_name);
            return true;
        }

        ROS_WARN_STREAM("Vertex "<<vertex_name<<" to edge dependencies not found");
        return false;
    }

    bool DiGraph::isVertex(const std::string &vertex_name)
    {
        return (m_vertices.find(vertex_name) != m_vertices.end());
    }

    VertexPtr DiGraph::getVertexData(const std::string &vertex_name)
    {
        if(m_vertices.find(vertex_name) != m_vertices.end())
        {
            return m_vertices.at(vertex_name);
        }

        return VertexPtr(nullptr);
    }

    VerticesMapPtr DiGraph::getVertices()
    {
        return std::make_shared<VerticesMap>(m_vertices);
    }

    bool DiGraph::addEdge(const std::string &edge_name, const std::string &src_vertex_name, const std::string &dst_vertex_name, const Edge &e)
    {   
        if(m_graph.find(src_vertex_name) == m_graph.end() && m_graph.find(dst_vertex_name) == m_graph.end())
        {
            ROS_WARN_STREAM("Source vertex "<<src_vertex_name<<" and destination vertex "<<dst_vertex_name<<" not found");
            return false;
        }
        else if(m_graph.find(src_vertex_name) == m_graph.end())
        {
            ROS_WARN_STREAM("Source vertex "<<src_vertex_name<<" not found");
            return false;
        }
        else if(m_graph.find(dst_vertex_name) == m_graph.end())
        {
            ROS_WARN_STREAM("Destination vertex "<<dst_vertex_name<<" not found");
            return false;
        }
        else
        {
            //Adding to edge tracking map
            m_edges.emplace(edge_name, std::make_shared<Edge>(e));

            //Update vertex to edge dependency
            m_v2e_dep.find(src_vertex_name)->second.insert(edge_name); //Adding the edge to the source vertex dependency set
            m_v2e_dep.find(dst_vertex_name)->second.insert(edge_name); //Adding the edge to the destination vertex dependency set

            //Adding to edge to vertices dependency map
            m_e2v_dep.emplace(edge_name, std::make_pair(src_vertex_name, dst_vertex_name));

            //Adding edge to graph
            if(m_graph.at(src_vertex_name).find(dst_vertex_name) == m_graph.at(src_vertex_name).end())
            {
                m_graph.at(src_vertex_name).insert(dst_vertex_name);
                ROS_INFO_STREAM("Edge between vertex "<<src_vertex_name<<" and "<<dst_vertex_name<<" added");
            }
            else
            {
                ROS_WARN_STREAM("Edge between vertex "<<src_vertex_name<<" and "<<dst_vertex_name<<" already exists");
            }

            return true;
        }
    }

    bool DiGraph::deleteEdge(const std::string &edge_name)
    {
        if(m_e2v_dep.find(edge_name) == m_e2v_dep.end()) //Checking to see if the edge actually exists
        {
            ROS_ERROR_STREAM("Edge "<<edge_name<<" could not be found. Initial edge creation probably failed!");
            return false;
        }

        std::string src_vertex_name, dst_vertex_name;
        src_vertex_name = m_e2v_dep.at(edge_name).first;
        dst_vertex_name = m_e2v_dep.at(edge_name).second;

        if(m_graph.find(src_vertex_name) == m_graph.end() && m_graph.find(dst_vertex_name) == m_graph.end())
        {
            ROS_WARN_STREAM("Source vertex "<<src_vertex_name<<" and destination vertex "<<dst_vertex_name<<" not found");
            return false;
        }
        else if(m_graph.find(src_vertex_name) == m_graph.end())
        {
            ROS_WARN_STREAM("Source vertex "<<src_vertex_name<<" not found");
            return false;
        }
        else if(m_graph.find(dst_vertex_name) == m_graph.end())
        {
            ROS_WARN_STREAM("Destination vertex "<<dst_vertex_name<<" not found");
            return false;
        }
        else
        {
            //Deleting the edge from the tracking map
            m_edges.erase(edge_name);

            //Deleting the edge in the source and destination vertexs' vertex to edge dependency map
            m_v2e_dep.at(src_vertex_name).erase(edge_name);
            m_v2e_dep.at(dst_vertex_name).erase(edge_name);

            m_e2v_dep.erase(edge_name); //Deleting from the edge to vertices dependency list
            m_misc_dep.erase(edge_name); //Deleting the edge from the miscellaneous dependency map

            //Deleting from graph
            if(m_graph.at(src_vertex_name).find(dst_vertex_name) != m_graph.at(src_vertex_name).end())
            {
                m_graph.at(src_vertex_name).erase(dst_vertex_name);
                ROS_INFO_STREAM("Edge between vertex "<<src_vertex_name<<" and "<<dst_vertex_name<<" deleted");
            }
            else
            {
                ROS_WARN_STREAM("Edge between vertex "<<src_vertex_name<<" and "<<dst_vertex_name<<" does not exist");
            }

            return true;
        }
    }

    bool DiGraph::getEdgeToVertexDependency(const std::string &edge_name, VertexDepPair &vdp)
    {
        if(m_e2v_dep.find(edge_name) != m_e2v_dep.end())
        {
            vdp = m_e2v_dep.at(edge_name);
            return true;
        }

        ROS_WARN_STREAM("Edge "<<edge_name<<" to vertex dependencies not found");
        return false;
    }

    bool DiGraph::isEdge(const std::string &edge_name)
    {
        return (m_edges.find(edge_name) != m_edges.end());
    }

    EdgePtr DiGraph::getEdgeData(const std::string &edge_name)
    {
        if(m_edges.find(edge_name) != m_edges.end())
        {
            return m_edges.at(edge_name);
        }

        return EdgePtr(nullptr);
    }

    EdgesMapPtr DiGraph::getEdges()
    {
        return std::make_shared<EdgesMap>(m_edges);
    }

    void DiGraph::addMiscDependency(const std::string &key_element, const std::string &value_element, bool mutual_dep)
    {
        if(m_misc_dep.find(key_element) != m_misc_dep.end())
        {
            m_misc_dep.at(key_element).insert(value_element);  
            ROS_INFO_STREAM("Updated existing graph element "<<key_element<<" with new "<<value_element<<" dependency"); 
        }
        else
        {
            MiscDepSet ms({value_element});
            m_misc_dep.emplace(key_element, ms);
            ROS_INFO_STREAM("Added new graph element dependency between "<<key_element<<" and "<<value_element);
        }

        if(!mutual_dep) return;
        
        ROS_INFO_STREAM("Adding mutual dependency");
        if(m_misc_dep.find(value_element) != m_misc_dep.end())
        {
            m_misc_dep.at(value_element).insert(key_element);
            ROS_INFO_STREAM("Updated existing graph element "<<value_element<<" with new "<<key_element<<" dependency"); 
        }
        else
        {
            MiscDepSet ms({key_element});
            m_misc_dep.emplace(value_element, ms);
            ROS_INFO_STREAM("Added new graph element dependency between "<<value_element<<" and "<<key_element);
        }
    }

    bool DiGraph::deleteMiscDependencyKey(const std::string &key_element)
    {
        if(isVertex(key_element))
        {
            ROS_WARN_STREAM("Vertex "<<key_element<<" cannot be deleted for miscellaneous dependency until deleted from graph");
            return false;
        }
        else if(isEdge(key_element))
        {
            ROS_WARN_STREAM("Edge "<<key_element<<" cannot be deleted for miscellaneous dependency until deleted from graph");
            return false;
        }

        if(m_misc_dep.find(key_element) != m_misc_dep.end())
        {
            m_misc_dep.erase(key_element);
            return true;
        }

        ROS_WARN_STREAM("Miscellaneous graph element key "<<key_element<<" not found");
        return false;
    }
    
    bool DiGraph::deleteMiscDependencyValue(const std::string &key_element, const std::string &value_element)
    {
        if(m_misc_dep.find(key_element) != m_misc_dep.end())
        {
            if(m_misc_dep.at(key_element).find(value_element) != m_misc_dep.at(key_element).end())
            {
                m_misc_dep.at(key_element).erase(value_element);
                if(m_misc_dep.at(key_element).size() == 0) 
                {
                    m_misc_dep.erase(key_element);
                    ROS_INFO_STREAM("Removing key "<<key_element<<" as no it has no dependency");
                }
                ROS_INFO_STREAM("Graph element dependency between "<<key_element<<" and "<<value_element<<" deleted");
                return true;
            }
            else
            {
                ROS_WARN_STREAM("Graph element dependency between "<<key_element<<" and "<<value_element<<" does not exist");
                return false;
            }
        }
        
        ROS_WARN_STREAM("Miscellaneous graph element key "<<key_element<<" not found");
        return false;
    }

    bool DiGraph::getMiscDependency(const std::string &key_element, MiscDepSet &mds)
    {
        if(m_misc_dep.find(key_element) != m_misc_dep.end())
        {
            mds = m_misc_dep.at(key_element);
            return true;
        }

        ROS_WARN_STREAM("Miscellaneous "<<key_element<<" graph element dependencies not found");
        return false;
    }

    bool DiGraph::isMiscellaneous(const std::string &key_element)
    {
        return (m_misc_dep.find(key_element) != m_misc_dep.end());
    }

    void DiGraph::printGraph(bool verbose) const
    {
        if(m_graph.empty())
        {
            ROS_WARN_STREAM("The graph is empty");
            if(!verbose) return;
        }

        std::stringstream ss_gph;

        if(!m_graph.empty())
        {
            for(AdjacencyListMap::const_iterator it = m_graph.begin(); it != m_graph.end(); ++it)
            {
                ss_gph<<"[ "<<it->first<<" ] --> [ ";
                for(auto vertex_name: it->second)
                {
                    ss_gph<<vertex_name<<" ";
                }
                ss_gph<<" ]\n";
            }

            ss_gph<<"\n";

            ROS_DEBUG_STREAM_NAMED("print_graph", "Graph\n"<<ss_gph.str());
        }

        if(verbose) printAdditionalInfo();
    }

    void DiGraph::printVertices() const
    {
        if(m_vertices.empty())
        {
            ROS_WARN_STREAM("The vertices list is empty");
            return;
        }

        std::stringstream ss_vl;

        for(VerticesMap::const_iterator it = m_vertices.begin(); it != m_vertices.end(); ++it)
        {
            ss_vl<<it->first<<"\n";
        }

        ss_vl<<"\n";

        ROS_DEBUG_STREAM_NAMED("vertex_list","Vertex list\n"<<ss_vl.str());
    }

    void DiGraph::printEdges() const
    {
        if(m_edges.empty())
        {
            ROS_WARN_STREAM("The edges list is empty");
            return;
        }

        std::stringstream ss_el;

        for(EdgesMap::const_iterator it  = m_edges.begin(); it != m_edges.end(); ++it)
        {
            ss_el<<it->first<<"\n";
        }

        ss_el<<"\n";

        ROS_DEBUG_STREAM_NAMED("edge_list", "Edge list\n"<<ss_el.str());
    }

    void DiGraph::printVertexToEdgesDeps() const
    {
        if(m_v2e_dep.empty())
        {
            ROS_WARN_STREAM("Vertex to edges dependency list is empty");
            return;
        }

        std::stringstream ss_v2e_dep;
        
        for(VertexToEdgesDepMap::const_iterator it_v = m_v2e_dep.begin(); it_v != m_v2e_dep.end(); ++it_v)
        {
            ss_v2e_dep<<"Vertex: "<<it_v->first<<" - Edge(s): { ";
            for(EdgeDepSet::const_iterator it_e = it_v->second.begin(); it_e != it_v->second.end(); ++it_e)
            {
                ss_v2e_dep<<*it_e<<" ";
            }
            ss_v2e_dep<<" }\n";
        }

        ss_v2e_dep<<"\n";

        ROS_DEBUG_STREAM_NAMED("vertex_to_edges_dependency", "Vertex to edges dependency list\n"<<ss_v2e_dep.str());
    }

    void DiGraph::printEdgeToVerticesDeps() const
    {
        if(m_e2v_dep.empty())
        {
            ROS_WARN_STREAM("Edge to vertices dependency list is empty");
            return;
        }

        std::stringstream ss_e2v_dep;
        
        for(EdgeToVerticesDepMap::const_iterator it_e = m_e2v_dep.begin(); it_e != m_e2v_dep.end(); ++it_e)
        {
            ss_e2v_dep<<"Edge: "<<it_e->first<<" ( Source vertex: "<<it_e->second.first<<", Destination vertex: "<<it_e->second.second<<" )\n";
        }

        ss_e2v_dep<<"\n";

        ROS_DEBUG_STREAM_NAMED("edge_to_vertices_dependency", "Edge to vertices dependency list\n"<<ss_e2v_dep.str());
    }

    void DiGraph::printMiscellaneousDeps() const
    {
        if(m_misc_dep.empty())
        {
            ROS_WARN_STREAM("Miscellaneous graph element dependency list is empty");
            return;
        }

        std::stringstream ss_md;

        for(MiscDepMap::const_iterator it_1 = m_misc_dep.begin(); it_1 != m_misc_dep.end(); ++it_1)
        {
            ss_md<<"Key element: "<<it_1->first<<" - Value Element(s): { ";
            for(MiscDepSet::const_iterator it_2 = it_1->second.begin(); it_2 != it_1->second.end(); ++it_2)
            {
                ss_md<<*it_2<<" ";
            }
            ss_md<<" }\n";
        }

        ss_md<<"\n";

        ROS_DEBUG_STREAM_NAMED("miscellaneous_dependency_list", "Miscellaneous graph element dependency list\n"<<ss_md.str());
    }

    void DiGraph::printAdditionalInfo() const
    {
        printVertices();
        printEdges();
        printVertexToEdgesDeps();
        printEdgeToVerticesDeps();
        printMiscellaneousDeps();
    }
}
