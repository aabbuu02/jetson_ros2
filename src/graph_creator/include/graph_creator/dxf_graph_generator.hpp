#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <graph_msgs/Vertex.h>
#include <graph_msgs/Edge.h>
#include <graph_creator_msgs/GraphGeneratorConfig.h>

#include <dxflib/dl_dxf.h>
#include <dxflib/dl_creationadapter.h>

#include <boost/filesystem.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <graph_creator/graph_data_types.hpp>
#include <graph_creator/graph_visuals.hpp>
#include <graph_creator/graph_transformers.hpp>


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

//TODO: This should be derived from base class for graph generators
class DxfGraphGenerator: public DL_CreationAdapter
{
    public:

    DxfGraphGenerator();
    DxfGraphGenerator(const graph_creator_msgs::GraphGeneratorConfig &config);
    
    bool getDxfFilePath(const std::string &in_file_path, std::string &out_file_path);
    bool loadDxfFile();
    bool loadDxfFile(const std::string &file_path); //Can accept both relative and absolute paths
    
    bool generateVerticesAndEdgesListFromDxf(Graph::VertexList &vl, Graph::EdgeList &el);
    bool generateVerticesAndEdgesListFromDxf(const graph_creator_msgs::GraphGeneratorConfig &config, Graph::VertexList &vl, Graph::EdgeList &el);
    
    bool checkDxfGraphGeneratorConfig(const graph_creator_msgs::GraphGeneratorConfig &config);

    private:

    Graph::Transform::VertexOrientationTransform m_vot;

    uint32_t m_vertex_counter = 0;

    typedef bg::model::point<double, 2, bg::cs::cartesian> VertexPoint;
    typedef bg::model::box<VertexPoint> VertexBox;
    typedef std::pair<VertexPoint, int> VertexIndex;
    
    std::vector<VertexIndex> m_nearest_vertex_point;
    bgi::rtree<VertexIndex, bgi::dynamic_quadratic> m_vertex_rtree;

    double m_box_size_x = 0.025;
    double m_box_size_y = 0.025;

    void createVertexTreeQueryBox(const geometry_msgs::Pose &pose, double box_size_x, double box_size_y, VertexBox &query_box);

    graph_creator_msgs::GraphGeneratorConfig m_config;

    Graph::VertexList m_vl;
    Graph::EdgeList m_el;

    std::unordered_map<uint32_t, Graph::Vertex> m_vertex_map;
    std::unordered_set<std::string> m_edge_set;

    DL_Dxf m_dxf;

    bool addVertexFromPose(const geometry_msgs::Pose &p, const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, Graph::Vertex &v);
    void computeVertexOrientationForLine(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, geometry_msgs::Quaternion &q) const;

    bool addEdgeForLine(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2);
    
    //Overriden functions for DXF file parsing
    void addPoint(const DL_PointData &data) override final;

    void addLine(const DL_LineData &data) override final;
    void addPolyline(const DL_PolylineData &data) override final;

    void addSpline(const DL_SplineData &data) override final;
    void addControlPoint(const DL_ControlPointData &data) override final;
    void addKnot(const DL_KnotData &data) override final;

    void addDimAlign(const DL_DimensionData &dd, const DL_DimAlignedData &dad);
    void addDimLinear(const DL_DimensionData &dd, const DL_DimLinearData &dld);
    void addDimRadial(const DL_DimensionData &dd, const DL_DimRadialData &drd);
    void addDimDiametric(const DL_DimensionData &dd, const DL_DimDiametricData &ddd);
    void addDimAngular(const DL_DimensionData &dd, const DL_DimAngularData &dad);
    void addDimAngular3P(const DL_DimensionData &dd, const DL_DimAngular3PData &dad);
    
};