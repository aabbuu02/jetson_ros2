
#include <global_path_graph_planner/path_graph_planner_node.hpp>

PathGraphPlanner::PathGraphPlanner()
{
    //Publishers
    m_graph_plan_pub = m_nh.advertise<nav_msgs::Path>("graph_plan", 1);

    //Subscribers
    m_graph_sub = m_nh.subscribe("graph", 1, &PathGraphPlanner::graphCallback, this);

    //Services servers
    m_get_graph_plan_srv = m_nh.advertiseService("make_graph_plan", &PathGraphPlanner::getGraphPlanServiceCallback, this);
}

void PathGraphPlanner::graphCallback(const graph_msgs::Graph &graph_msg)
{
    m_graph = m_gpg.getGraphFromMsg(graph_msg);
    
    if(!m_graph)
    {
        ROS_WARN_STREAM("Path graph planner received an invalid graph");
        m_is_graph_initialized = false;
        return;
    }

    if(m_graph->empty()) 
    {
        ROS_WARN_STREAM("Path graph planner received an empty graph");
        m_is_graph_initialized = false;
        return;
    }

    ROS_INFO_STREAM("Path graph planner received a valid graph");

    m_gpg.printPathGraph(true);
    
    if(!m_planner.setGraph(m_gpg)) //Set the graph to the planner
    {
        ROS_WARN_STREAM("Setting graph in planner failed");
        m_is_graph_initialized = false;
	return;
    }

    m_is_graph_initialized = true;
}

bool PathGraphPlanner::getGraphPlanServiceCallback(graph_msgs::GetGraphPlan::Request &req, 
                                                   graph_msgs::GetGraphPlan::Response &res)
{
    ROS_INFO_STREAM("Get plan from graph service called");
    ROS_INFO_STREAM("Plan requested from source vertex "<<req.source_id<<" to target vertex "<<req.target_id);

    if(!m_is_graph_initialized)
    {
        ROS_ERROR_STREAM("Planning on graph cannot be done as graph is not initialized");
        res.success = false;
        res.message = "Graph not initialized";
        return true;
    }

    std::vector<geometry_msgs::PoseStamped> retreived_graph_plan;
    std::vector<uint32_t> retrived_vertices_in_plan;
    
    bool plan_success = m_planner.getPlanFromGraph(req.source_id, req.target_id, retreived_graph_plan, 
                                                   req.ignore_goal_orientation, req.ignore_path_orientation);

    if(!plan_success)
    {
        res.success = false;
        res.message = "Retreiving plan from graph failed";
        return true;
    }

    m_planner.getPointsInPlan(retrived_vertices_in_plan);
    
    ROS_INFO_STREAM("Retreived point plan size: "<<retrived_vertices_in_plan.size());

    for(const uint32_t &v_id: retrived_vertices_in_plan)
    {
        graph_msgs::Vertex v = *m_gpg.getPointData(v_id);
        res.vertices_in_plan.push_back(v);
    }

    res.plan.header.frame_id = "map";
    res.plan.header.seq++;
    res.plan.poses = retreived_graph_plan;
    res.plan_distance = m_planner.getPlanDistance();

    res.success = true;
    res.message = "Retreived plan from graph successfully";

    //Publish the retreived plan
    m_graph_plan_pub.publish(res.plan);

    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_path_graph_planner");

    PathGraphPlanner pgp;
    ros::spin();

    return EXIT_SUCCESS;
}
