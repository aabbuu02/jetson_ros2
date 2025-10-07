#include <graph_server/graph_server.hpp>

GraphServer::GraphServer()
{
    //Publishers
    m_graph_pub = m_nh.advertise<graph_msgs::Graph>("graph", 1, true);

    //Service servers
    m_load_graph_srv = m_nh.advertiseService("load_graph", &GraphServer::loadGraphServiceCallback, this);
    m_get_graph_srv = m_nh.advertiseService("get_graph", &GraphServer::getGraphServiceCallback, this);
}

bool GraphServer::loadGraphServiceCallback(graph_msgs::LoadGraph::Request &req, graph_msgs::LoadGraph::Response &res)
{
    ROS_INFO_STREAM("Load graph service called");

    if(req.graph_url.size() == 0)
    {
        ROS_ERROR_STREAM("The graph file URL is empty");
        res.result = graph_msgs::LoadGraph::Response::RESULT_INVALID_GRAPH_FILE_PATH;
        return true;
    }

    std::string final_file_path = "";

    if(!Graph::File::getGraphFilePath(req.graph_url, final_file_path))
    {
        ROS_ERROR_STREAM("The graph file path is not valid");
        res.result = graph_msgs::LoadGraph::Response::RESULT_INVALID_GRAPH_FILE_PATH;
        return true;
    }

    ROS_INFO_STREAM("Graph file path: "<<final_file_path);

    if(!loadGraphFromFileAndPublish(final_file_path))
    {
        ROS_ERROR_STREAM("Graph could not be found with in the specified path");
        res.result = graph_msgs::LoadGraph::Response::RESULT_GRAPH_DOES_NOT_EXIST;
        return true;
    }

    res.graph = m_graph_msg;
    res.result = graph_msgs::LoadGraph::Response::RESULT_SUCCESS;
    return true;
}

bool GraphServer::getGraphServiceCallback(graph_msgs::GetGraph::Request &req, graph_msgs::GetGraph::Response &res)
{
    ROS_INFO_STREAM("Get graph service called");

    res.graph = m_graph_msg;
    if(m_is_initialized) res.result = graph_msgs::GetGraph::Response::RESULT_GRAPH_VALID;
    else res.result = graph_msgs::GetGraph::Response::RESULT_GRAPH_INVALID;

    return true;
}

bool GraphServer::loadGraphFromFileAndPublish(const std::string &filename)
{
    GraphLoader gl;

    Graph::VertexList vl;
    Graph::EdgeList el;

    if(!gl.loadGraph(filename))
    {
        ROS_ERROR_STREAM("Graph file "<<filename<<" could not be loaded");
        return false;
    }

    gl.getVerticesAndEdges(vl, el);

    m_graph_msg.header.frame_id = "map";
    m_graph_msg.header.stamp = ros::Time::now();
    m_graph_msg.header.seq++;

    m_graph_msg.meta_data.header.frame_id = "";
    m_graph_msg.meta_data.header.stamp = ros::Time::now();
    m_graph_msg.meta_data.header.seq++;

    gl.getGraphName(m_graph_msg.meta_data.graph_name);
    
    std::string graph_type = "";
    gl.getGraphType(graph_type);
    if(graph_type == "directed") m_graph_msg.meta_data.graph_type = graph_msgs::GraphMetaData::DIRECTED;
    else m_graph_msg.meta_data.graph_type = graph_msgs::GraphMetaData::UNDIRECTED;

    m_graph_msg.meta_data.number_of_vertices = gl.getVerticesCount();
    m_graph_msg.meta_data.number_of_edges = gl.getEdgesCount();

    ROS_INFO_STREAM("Loaded graph file information:");
    ROS_INFO_STREAM("Graph name: "<<m_graph_msg.meta_data.graph_name);
    ROS_INFO_STREAM("Graph type: "<<graph_type);
    ROS_INFO_STREAM("Number of vertices: "<<m_graph_msg.meta_data.number_of_vertices);
    ROS_INFO_STREAM("Number of edges: "<<m_graph_msg.meta_data.number_of_edges);

    publishGraph(vl, el);

    m_is_initialized = true;

    return true;
}

void GraphServer::publishGraph(const Graph::VertexList &vl, const Graph::EdgeList &el)
{
    Graph::Utilities::convertFromEdgeAndVertexListToMsg(vl, el, m_graph_msg);
    m_graph_pub.publish(m_graph_msg);

    ROS_INFO_STREAM("Final published graph information");
    ROS_INFO_STREAM("Number of vertices: "<<m_graph_msg.vertices.size());
    ROS_INFO_STREAM("Number of edges: "<<m_graph_msg.edges.size());
}

inline void displayUsage()
{
    std::stringstream ss;
    ss<<"Graph server usage: \n";
    ss<<"Usage: graph_server graph_file_name.graphml\n";
    ss<<"-h: To display this help\n";
    ROS_INFO_STREAM(ss.str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "graph_server");

    GraphServer gs;

    if(argc == 1) ROS_WARN_STREAM("No graphs are given at startup. Please use service to load the graph file");
    else if(argc == 2)
    {
        if(strcmp(argv[1], "-h") == 0) displayUsage();
        else if(strlen(argv[1]) > 0)
        {
            std::string final_file_path = "";
            ROS_INFO_STREAM("Graph file given at startup: "<<argv[1]);

            if(Graph::File::getGraphFilePath(argv[1], final_file_path))
            {
                ROS_INFO_STREAM("Final path of graph file given at startup: "<<final_file_path);

                if(!gs.loadGraphFromFileAndPublish(final_file_path)) 
                    ROS_ERROR_STREAM("Could not load graph given at startup. Please use service to load the graph file");
            }
            else ROS_WARN_STREAM("The graph file path given at startup is invalid. Please use service to load the graph file");
        }
        else ROS_WARN_STREAM("The graph file name is empty. Please use service to load the graph file");
    }
    else if(argc > 2)
    {
        ROS_ERROR_STREAM("Too many arguments passed to graph server");
        displayUsage();
    }

    ros::spin();

    return EXIT_SUCCESS;
}
