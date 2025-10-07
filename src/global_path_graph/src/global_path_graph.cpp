#include <global_path_graph/global_path_graph.hpp>

GlobalPathGraph::GlobalPathGraph()
{
    
}

bool GlobalPathGraph::addPoint(const uint32_t &point_id, const graph_msgs::Vertex &point)
{
    if(m_adjacency_sm.find(point_id) == m_adjacency_sm.end())
    {
        //Adding point to tracking map
        m_points.emplace(point_id, std::make_shared<graph_msgs::Vertex>(point));

        //Adding point to the graph
        AdjacencyPointSet ps;
        m_adjacency_sm.emplace(point_id, ps);

        ROS_INFO_STREAM("Added point with "<<point_id);
        return true;
    }
    
    ROS_WARN_STREAM("Point with "<<point_id<<" already present");
    return false;
}

bool GlobalPathGraph::deletePoint(const uint32_t &point_id)
{
    if(m_adjacency_sm.find(point_id) != m_adjacency_sm.end())
    {
        //Deleting from point tracking map
        m_points.erase(point_id);

        //Deleting point from the graph
        m_adjacency_sm.erase(point_id);

        for(AdjacencySetMap::iterator it = m_adjacency_sm.begin(); it != m_adjacency_sm.end(); ++it)
        {
            if(it->second.find(point_id) != it->second.end());
                it->second.erase(point_id); //Erasing vertex copies in the graph
        }

        ROS_INFO_STREAM("Point with "<<point_id<<" deleted");
        return true;
    }
    
    ROS_WARN_STREAM("Point with "<<point_id<<" not present");
    return false;
}

GlobalPathGraph::PointPtr GlobalPathGraph::getPointData(const uint32_t &point_id)
{
    if(m_points.find(point_id) != m_points.end())
    {
        return m_points.at(point_id);
    }

    return PointPtr(nullptr);
}

bool GlobalPathGraph::addPath(const std::pair<uint32_t, uint32_t> &point_ids, const graph_msgs::Edge &path)
{
    if(m_adjacency_sm.find(point_ids.first) == m_adjacency_sm.end() && m_adjacency_sm.find(point_ids.second) == m_adjacency_sm.end())
    {
        ROS_WARN_STREAM("Source point "<<point_ids.first<<" and target point "<<point_ids.second<<" not found");
        return false;
    }
    else if(m_adjacency_sm.find(point_ids.first) == m_adjacency_sm.end())
    {
        ROS_WARN_STREAM("Source point "<<point_ids.first<<" not found");
        return false;
    }
    else if(m_adjacency_sm.find(point_ids.second) == m_adjacency_sm.end())
    {
        ROS_WARN_STREAM("Target point "<<point_ids.second<<" not found");
        return false;
    }
    else
    {
        //Adding path to tracking map
        m_paths.emplace(point_ids, std::make_shared<graph_msgs::Edge>(path));

        //Adding path to the graph
        if(m_adjacency_sm.at(point_ids.first).find(point_ids.second) == m_adjacency_sm.at(point_ids.first).end())
        {
            m_adjacency_sm.at(point_ids.first).insert(point_ids.second);
            ROS_INFO_STREAM("Added path between point "<<point_ids.first<<" and "<<point_ids.second);
        }
        else
        {
            ROS_WARN_STREAM("Path between point "<<point_ids.first<<" and "<<point_ids.second<<" already exists");
        }
    }

    return true;
}

bool GlobalPathGraph::deletePath(const std::pair<uint32_t, uint32_t> &point_ids)
{
    if(m_adjacency_sm.find(point_ids.first) == m_adjacency_sm.end() && m_adjacency_sm.find(point_ids.second) == m_adjacency_sm.end())
    {
        ROS_WARN_STREAM("Source point "<<point_ids.first<<" and target point "<<point_ids.second<<" not found");
        return false;
    }
    else if(m_adjacency_sm.find(point_ids.first) == m_adjacency_sm.end())
    {
        ROS_WARN_STREAM("Source point "<<point_ids.first<<" not found");
        return false;
    }
    else if(m_adjacency_sm.find(point_ids.second) == m_adjacency_sm.end())
    {
        ROS_WARN_STREAM("Target point "<<point_ids.second<<" not found");
        return false;
    }
    else
    {
        //Deleting path from the tracking map
        m_paths.erase(point_ids);

        //Deleting from the graph
        if(m_adjacency_sm.at(point_ids.first).find(point_ids.second) != m_adjacency_sm.at(point_ids.first).end())
        {
            m_adjacency_sm.at(point_ids.first).erase(point_ids.second);
            ROS_INFO_STREAM("Path between point "<<point_ids.first<<" and "<<point_ids.second<<" deleted");
        }
        else
        {
            ROS_WARN_STREAM("Path between point "<<point_ids.first<<" and "<<point_ids.second<<" does not exist");
        }
    }

    return true;
}

GlobalPathGraph::PathPtr GlobalPathGraph::getPathData(const std::pair<uint32_t, uint32_t> &point_ids)
{
    if(m_paths.find(point_ids) != m_paths.end())
    {
        return m_paths.at(point_ids);
    }

    return PathPtr(nullptr);
}

GlobalPathGraph::AdjacencySetMapPtr GlobalPathGraph::getGraphFromMsg(const graph_msgs::Graph &graph)
{
    //Clear the entire graph just in case along with point and paths
    clear();

    for(const graph_msgs::Vertex &v_msg: graph.vertices)
    {
        if(!addPoint(v_msg.id, v_msg)) 
            ROS_WARN_STREAM("Point with ID "<<v_msg.id<<" was not added");
    }

    for(const graph_msgs::Edge &e_msg: graph.edges)
    {
        if(!addPath({e_msg.source_vertex_id, e_msg.target_vertex_id}, e_msg)) 
            ROS_WARN_STREAM("Path between point "<<e_msg.source_vertex_id<<" and "<<e_msg.target_vertex_id<<" was not added");
    }

    return std::make_shared<AdjacencySetMap>(m_adjacency_sm);
}

GlobalPathGraph::AdjacencySetMapPtr GlobalPathGraph::getGraph()
{
    return std::make_shared<AdjacencySetMap>(m_adjacency_sm);
}

void GlobalPathGraph::clear()
{
    m_points.clear();
    m_paths.clear();

    m_adjacency_sm.clear();
}

bool GlobalPathGraph::isEmpty()
{
    return m_adjacency_sm.empty();
}

void GlobalPathGraph::printPathGraph(bool verbose)
{
    if(m_adjacency_sm.empty())
    {
        ROS_WARN_STREAM("Path graph is empty");
        if(!verbose) return;
    }

    if(!m_adjacency_sm.empty())
    {
        ROS_INFO_STREAM("Path graph");

        std::cout<<"\n";
        for(AdjacencySetMap::const_iterator it = m_adjacency_sm.begin(); it != m_adjacency_sm.end(); ++it)
        {
            std::cout<<"[ "<<it->first<<" ] --> [ ";
            for(const uint32_t point_id: it->second)
            {
                std::cout<<point_id<<" ";
            }
            std::cout<<" ]\n";
        }
        std::cout<<"\n"; 
    }

    if(verbose)
    {
        printAdditionalInfo();
    }
}

void GlobalPathGraph::printAdditionalInfo()
{
    printPoints();
    printPaths();
}

void GlobalPathGraph::printPoints()
{
    if(m_points.empty())
    {
        ROS_WARN_STREAM("The points list is empty");
        return;
    }

    ROS_INFO_STREAM("Points");
    for(PointsMap::const_iterator it = m_points.begin(); it != m_points.end(); ++it)
    {
        std::cout<<it->first<<"\n";
    }

    std::cout<<std::endl;
}

void GlobalPathGraph::printPaths()
{
    if(m_paths.empty())
    {
        ROS_WARN_STREAM("The paths list is empty");
    }

    ROS_INFO_STREAM("Paths");
    for(PathsMap::const_iterator it = m_paths.begin(); it != m_paths.end(); ++it)
    {
        std::cout<<it->first.first<<" --> "<<it->first.second<<"\n";
    }

    std::cout<<std::endl;
}

