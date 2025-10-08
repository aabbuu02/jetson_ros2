#pragma once

#include <memory>
#include <vector>
#include <unordered_set>
#include <unordered_map>

#include <ros/ros.h>
#include <graph_msgs/Vertex.h>
#include <graph_msgs/Edge.h>
#include <graph_msgs/Graph.h>

class GlobalPathGraph
{
    public:

    //Hash and compare functions where the pair ordering does not matter i.e (1, 2) is equivalent to (2, 1)
    struct PairKeyUnorderedHash
    {
        size_t operator()(const std::pair<uint32_t, uint32_t> &p) const
        {
            return std::hash<uint32_t>()(p.first) + std::hash<uint32_t>()(p.second);
        }
    };

    struct PairKeyUnorderedCompare
    {
        bool operator()(const std::pair<uint32_t, uint32_t> &lhs, const std::pair<uint32_t, uint32_t> &rhs) const
        {
            return (lhs.first == rhs.first && lhs.second == rhs.second) ||
                   (lhs.first == rhs.second && lhs.second == rhs.first);
        }
    };

    //Hash and compare functions where the pari order does matter i.e (1, 2) is not equivalent to (2, 1)
    struct PairKeyOrderedHash
    {
        size_t operator()(const std::pair<uint32_t, uint32_t> &p) const
        {
            return (size_t) p.first << sizeof(uint32_t) | p.second;
        }
    };

    struct PairKeyOrderedCompare
    {
        bool operator()(const std::pair<uint32_t, uint32_t> &lhs, const std::pair<uint32_t, uint32_t> &rhs) const
        {
            return lhs.first == rhs.first && lhs.second == rhs.second;
        }
    };

    //Graph point and path pointers
    typedef std::shared_ptr<graph_msgs::Vertex> PointPtr;
    typedef std::shared_ptr<graph_msgs::Edge> PathPtr;

    //Graph point and path maps
    typedef std::unordered_map<uint32_t, PointPtr> PointsMap;
    typedef std::unordered_map<std::pair<uint32_t, uint32_t>, PathPtr, PairKeyOrderedHash, PairKeyOrderedCompare> PathsMap;

    //Graph point and path maps pointers
    typedef std::shared_ptr<PointsMap> PointMapPtr;
    typedef std::shared_ptr<PathsMap> PathMapPtr;

    //Graph adjecency list using set and map
    typedef std::unordered_set<uint32_t> AdjacencyPointSet;
    typedef std::unordered_map<uint32_t, AdjacencyPointSet> AdjacencySetMap;

    //Graph adjacency list pointers
    typedef std::shared_ptr<AdjacencyPointSet> AdjacencyPointSetPtr;
    typedef std::shared_ptr<AdjacencySetMap> AdjacencySetMapPtr;

    GlobalPathGraph();

    //Points
    bool addPoint(const uint32_t &point_id, const graph_msgs::Vertex &point);
    bool deletePoint(const uint32_t &point_id);
    PointPtr getPointData(const uint32_t &point_id);

    
    //Paths
    bool addPath(const std::pair<uint32_t, uint32_t> &point_ids, const graph_msgs::Edge &path);
    bool deletePath(const std::pair<uint32_t, uint32_t> &point_ids);
    PathPtr getPathData(const std::pair<uint32_t, uint32_t> &point_ids);

    //Graph
    AdjacencySetMapPtr getGraphFromMsg(const graph_msgs::Graph &graph);
    AdjacencySetMapPtr getGraph();
    void clear();
    bool isEmpty();

    //Debug
    void printPathGraph(bool verbose=false);
    void printAdditionalInfo();
    void printPoints();
    void printPaths();

    private:

    //Graph 
    PointsMap m_points;
    PathsMap m_paths;

    AdjacencySetMap m_adjacency_sm;

};