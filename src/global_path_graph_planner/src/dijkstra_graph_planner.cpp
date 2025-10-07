#include <global_path_graph_planner/dijkstra_graph_planner.hpp>

DijkstraGraphPlanner::DijkstraGraphPlanner()
{

}


bool DijkstraGraphPlanner::setGraph(GlobalPathGraph &gpg)
{
    if(!gpg.getGraph())
    {
        ROS_ERROR_STREAM("Invalid graph was passed to graph planner");
        return false;
    }

    if(gpg.isEmpty())
    {
        ROS_ERROR_STREAM("An empty graph was passed to graph planner");
        return false;
    }

    m_gpg = gpg;
    m_graph = m_gpg.getGraph();

    return true;
}

bool DijkstraGraphPlanner::getPointPlanFromGraph(const uint32_t &src_point_id, const uint32_t &dst_point_id, 
                                                 std::vector<uint32_t> &points_in_plan)
{
    //Clear the point plan
    points_in_plan.clear();

    //Clear the plan in terms of point vector and set;
    m_points_in_plan_v.clear();

    if(!dijkstraPlanner(src_point_id, dst_point_id, m_points_in_plan_v))
        return false;

    printPointsInPlan();

    points_in_plan = m_points_in_plan_v;
    return true;
}

bool DijkstraGraphPlanner::getPlanFromGraph(const uint32_t &src_point_id, const uint32_t &dst_point_id, 
                                            std::vector<geometry_msgs::PoseStamped> &plan, bool ignore_goal_orientation,
                                            bool ignore_path_orientation)
{
    //Clear the plan
    plan.clear();
    
    //Clear plan in terms of point vector and set
    m_points_in_plan_v.clear();

    if(!dijkstraPlanner(src_point_id, dst_point_id, m_points_in_plan_v))
        return false;

    if(!getPlanFromPoints(m_points_in_plan_v, plan, ignore_goal_orientation, ignore_path_orientation))
        return false;

    printPointsInPlan();

    return true;
}

bool DijkstraGraphPlanner::getPlanFromPoints(const std::vector<uint32_t> &point_plan, std::vector<geometry_msgs::PoseStamped> &plan,
                                             bool ignore_goal_orientation, bool ignore_path_orientation)
{
    GlobalPathGraph::PointPtr pnt_1_ptr, pnt_2_ptr;
    geometry_msgs::PoseStamped pose_stamped_msg;
    
    m_plan_distance = 0.0;

    if(point_plan.size() == 1)
    {
        pnt_2_ptr = m_gpg.getPointData(point_plan.back());

        if(!pnt_2_ptr)
        {
            ROS_ERROR_STREAM("Point "<<point_plan.back()<<" was not found with in the graph for single pose plan");
            return false;
        }

        ROS_WARN_COND(ignore_goal_orientation, "Plan retreived was for a single point, therefore, the point orientation will be applied. Cannot ignore goal orientation");
    }

    for(size_t i = 0; i < point_plan.size()-1; ++i)
    {
        //The path and point data should not be null here as they were accessed by the planner before
        //TODO: Check whether the above holds true for cases when a new graph is received via a callback and set here
        //Get the path information
        GlobalPathGraph::PathPtr pth_ptr = m_gpg.getPathData({point_plan.at(i), point_plan.at(i+1)});

        if(!pth_ptr)
        {
            ROS_ERROR_STREAM("No path exists from "<<point_plan.at(i)<<" to "<<point_plan.at(i+1));
            return false;
        }

        //Get path point information for edge calculation
        pnt_1_ptr = m_gpg.getPointData(pth_ptr->source_vertex_id);
        pnt_2_ptr = m_gpg.getPointData(pth_ptr->target_vertex_id);
        
        if(!pnt_1_ptr)
        {
            ROS_ERROR_STREAM("Source point "<<pth_ptr->source_vertex_id<<" was not found with in the path graph");
            return false;
        }

        if(!pnt_2_ptr)
        {
            ROS_ERROR_STREAM("Destination point "<<pth_ptr->target_vertex_id<<" was not found with in the path graph");
            return false;
        }

        double length = 0.0;
        std::vector<geometry_msgs::Pose> eqd_poses;

        switch (pth_ptr->type)
        {
            case graph_msgs::Edge::STRAIGHT_LINE:
            {
                Graph::Curves::calculateEquiDistantPosesLine({pnt_1_ptr->pose, pnt_2_ptr->pose}, eqd_poses, length, m_pose_spacing);
                m_plan_distance += length;

                break;
            }

            case graph_msgs::Edge::CUBIC_BEZIER:
            {
                if(pth_ptr->control_points.size() != 2) //Check for the correct size of control points
                {
                    ROS_ERROR_STREAM("Cannot construct Bezier curve without two control points");
                    return false;
                }

                Graph::Curves::calculateEquiDistantPosesBezier({pnt_1_ptr->pose, pth_ptr->control_points.at(0),
                                                                pth_ptr->control_points.at(1), pnt_2_ptr->pose}, 
                                                                eqd_poses, length, m_pose_spacing, m_resolution);
                m_plan_distance += length;

                break;
            }
            
            default:
            {
                ROS_ERROR_STREAM("Unknown graph type was provided");
                return false;
            }
        }

        pose_stamped_msg.header.frame_id = "map";

        for(size_t i = 0; i < eqd_poses.size(); ++i)
        {
            pose_stamped_msg.header.seq++;
            pose_stamped_msg.header.stamp = ros::Time::now();

            pose_stamped_msg.pose = eqd_poses.at(i);

            //Use independent orientation that was set previously when creating the path or edge
            if(pth_ptr->independent_orientation && !ignore_path_orientation)
                pose_stamped_msg.pose.orientation = pth_ptr->control_orientation.orientation;

            plan.push_back(pose_stamped_msg);
        }
    }

    if((!pnt_2_ptr->ignore_orientation && !ignore_goal_orientation) || point_plan.size() == 1)
    {
        pose_stamped_msg.header.seq++;
        pose_stamped_msg.header.stamp = ros::Time::now();

        pose_stamped_msg.pose = pnt_2_ptr->pose; //The destination pose
        plan.push_back(pose_stamped_msg);
    }

    ROS_INFO_STREAM("Retreived plan size (Number of poses in plan): "<<plan.size());

    return true;
}

void DijkstraGraphPlanner::getPointsInPlan(std::set<uint32_t> &points_in_plan)
{
    points_in_plan.clear();

    for(const uint32_t &v_id: m_points_in_plan_v)
        points_in_plan.emplace(v_id);
}

void DijkstraGraphPlanner::getPointsInPlan(std::vector<uint32_t> &points_in_plan)
{
    points_in_plan = m_points_in_plan_v;
}

double DijkstraGraphPlanner::getPlanDistance()
{
    return m_plan_distance;
}

bool DijkstraGraphPlanner::dijkstraPlanner(const uint32_t &src_point_id, const uint32_t &dst_point_id, 
                                        std::vector<uint32_t> &points_in_path)
{
    // Clear the points in path just in case
    points_in_path.clear();

    ROS_INFO_STREAM("Planning from source point "<<src_point_id<<" to destination point "<<dst_point_id);

    //TODO: Funtion to check whether point or edge is part of the graph or not is required
    if(m_graph->find(src_point_id) == m_graph->end() && m_graph->find(dst_point_id) == m_graph->end())
    {
        ROS_ERROR_STREAM("Source point "<<src_point_id<<" and destination point "<<dst_point_id<<" was not found in the graph");
        return false;
    }

    if(m_graph->find(src_point_id) == m_graph->end())
    {
        ROS_ERROR_STREAM("Source point "<<src_point_id<<" was not found in the graph");
        return false;
    }

    if(m_graph->find(dst_point_id) == m_graph->end())
    {
        ROS_ERROR_STREAM("Destination point "<<dst_point_id<<" was not found in the graph");
        return false;
    }

    if(src_point_id == dst_point_id)
    {
        //When the source point and destination point are equal
        ROS_INFO_STREAM("Source and destination points are same");
        points_in_path.push_back(dst_point_id);
        return true;
    }

    if(!m_graph)
    {
        ROS_ERROR_STREAM("The graph planner does not have a valid graph. A valid graph needs to be set");
        return false;
    }

    if(m_graph->empty())
    {
        ROS_ERROR_STREAM("The current graph in graph planner is empty");
        return false;
    }

    //Clear required variables
    m_point_distance.clear();
    m_previous_point.clear();

    //Reserve according to graph size
    m_point_distance.reserve(m_graph->size());
    m_previous_point.reserve(m_graph->size());

    //The source point is given 0 distance value
    m_point_distance.emplace(src_point_id, 0);

    //Initialization of the point distances and previous points of each point that was discovered
    for(GlobalPathGraph::AdjacencySetMap::const_iterator it = m_graph->begin(); it != m_graph->end(); ++it)
    {
        if(it->first == src_point_id) continue;

        m_point_distance.emplace(it->first, UINT32_MAX);
    }

    //The priority queue that holds the best point to expand and search for the destination point
    std::priority_queue<PointCost, std::vector<PointCost>, PointCostCompare> point_pq;

    //Initialize the priority queue with source point
    point_pq.emplace(src_point_id, m_point_distance.at(src_point_id)); //This cannot fail

    while(!point_pq.empty())
    {
        //While the queue is not empty pop a new point from the queue
        PointCost current_pc = point_pq.top(); //Current point and distance
        point_pq.pop(); //Remove the point from the queue

        if(current_pc.point_id == dst_point_id) //When the destination point is reached
        {
            ROS_INFO_STREAM("Found a path to destination point "<<dst_point_id);
            
            //Retreive the shortest path
            points_in_path.push_back(dst_point_id);

            uint32_t current_point_id = dst_point_id;

            while(m_previous_point.find(current_point_id) != m_previous_point.end())
            {
                points_in_path.push_back(m_previous_point.at(current_point_id));
                current_point_id = points_in_path.back();
            }

            std::reverse(points_in_path.begin(), points_in_path.end());            

            return true;
        }

        if(m_graph->find(current_pc.point_id) == m_graph->end())
        {
            ROS_ERROR_STREAM("The point "<<current_pc.point_id<<" does not exist in the graph");
            return false;
        }

        //TODO: A function to get neighbouring points is required prefereably in the form of a pointer to a set in global path graph
        GlobalPathGraph::AdjacencyPointSet current_point_neighbours = m_graph->at(current_pc.point_id);

        ROS_DEBUG_STREAM_NAMED("graph_dijkstra_planner", "Currently exploring point with ID: "<<current_pc.point_id<<" (Running cost: "<<current_pc.cost<<")");

        if(current_point_neighbours.size() == 0)
        {
            ROS_DEBUG_STREAM_NAMED("graph_dijkstra_planner", "No neighbouring point for current point with ID: "<<current_pc.point_id);
            continue;
        }

        for(const uint32_t neighbour_point_id: current_point_neighbours)
        {
            GlobalPathGraph::PathPtr path_ptr = m_gpg.getPathData({current_pc.point_id,  neighbour_point_id});

            if(!path_ptr)
            {
                ROS_ERROR_STREAM("Cannot retreive path data between points "<<current_pc.point_id<<" and "<<neighbour_point_id);
                return false;
            }

            ROS_DEBUG_STREAM_NAMED("graph_dijkstra_planner", "Current point "<<current_pc.point_id<<" to neighbour point "<<neighbour_point_id
                                   <<" distance is "<<path_ptr->length<<" (Cost factor: "<<path_ptr->cost_factor<<")");

            double cost_factor = 1.0f;

            if(m_enable_cost_factor)
            {
                if(cost_factor < 0.0f)
                {
                    ROS_WARN_STREAM("Cost factor cannot be less than zero");
                    ROS_WARN_STREAM("Using a cost factor of 1 for path between current point "<<current_pc.point_id<<" and neigbour point "<<neighbour_point_id);
                    cost_factor = 1.0f;
                }

                if(cost_factor == 0.0f) ROS_WARN_ONCE("Cost factor of zero is being used. This message will be printed only once");

                cost_factor = path_ptr->cost_factor; //Working on the copy of the cost factor
            }

            double new_distance = m_point_distance.at(current_pc.point_id) + (path_ptr->length * cost_factor);

            if(new_distance < m_point_distance.at(neighbour_point_id))
            {
                m_point_distance.at(neighbour_point_id) = new_distance;
                m_previous_point[neighbour_point_id] = current_pc.point_id;
                point_pq.emplace(neighbour_point_id, new_distance);

                ROS_DEBUG_STREAM_NAMED("graph_dijkstra_planner", "Neighbour point ID: "<<neighbour_point_id<<" (Running cost: "<<new_distance<<")");
                ROS_DEBUG_STREAM_NAMED("graph_dijkstra_planner", "Previous point: "<<m_previous_point.at(neighbour_point_id));
            }
        }
    }

    ROS_ERROR_STREAM("No path between source point "<<src_point_id<<" and destination point "<<dst_point_id<<" was found");
    return false;
}

void DijkstraGraphPlanner::enableCostFactor(bool enable)
{
    m_enable_cost_factor = enable;
}

bool DijkstraGraphPlanner::isCostFactorEnabled()
{
    return m_enable_cost_factor;
}

void DijkstraGraphPlanner::printPointsInPlan()
{
    //Path debug information
    std::stringstream ss;
    ss<<"[ ";
    for(const uint32_t &point_id: m_points_in_plan_v)
    {
        ss<<point_id<<" ";
    }
    ss<<"]\n";
    ROS_INFO_STREAM("Current plan in term of points: "<<ss.str());
}
