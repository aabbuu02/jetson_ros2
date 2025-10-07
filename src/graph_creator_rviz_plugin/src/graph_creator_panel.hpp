#pragma once

#include <vector>
#include <string>
#include <unordered_set>

#include <QString>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QMessageBox>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/package.h>
#include <rviz/panel.h>
#endif

#include <graph_msgs/Graph.h>
#include <graph_msgs/Vertex.h>
#include <graph_msgs/Edge.h>
#include <graph_msgs/Property.h>
#include <graph_msgs/LoadGraph.h>
#include <graph_msgs/SaveGraph.h>
#include <std_srvs/SetBool.h>
#include <graph_creator_msgs/GraphCreatorState.h>
#include <graph_creator_msgs/SetGraphCreatorMode.h>
#include <graph_creator_msgs/SetEdgeCreationType.h>
#include <graph_creator_msgs/SetEdgeDirectionCreationType.h>
#include <graph_creator_msgs/GetGraphCreatorState.h>

#include "properties_dialog_delegate.hpp"
#include "combo_box_delegate.hpp"
#include "double_spin_box_delegate.hpp"
#include "graph_creator_panel_ui.hpp"
#include "graph_creator_panel.hpp"

namespace graph_creator_rviz_plugin
{

class GraphCreatorPanel: public rviz::Panel
{
  //TODO: This file needs to be placed in an include directory but is causing complile errors
  //TODO: .ui files needs to automatically generate header files on complile time and then included
  Q_OBJECT
  public:

  GraphCreatorPanel( QWidget* parent = 0 );
  ~GraphCreatorPanel();

  //RViz configuration load and save
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  public Q_SLOTS:

  void onCreateVerticesClicked();
  void onCreateEdgesClicked();
  void onEdgeCreationTypeIndexChanged(int index);
  void onEdgeDirectionTypeIndexChanged(int index);
  void onLockAllVerticesClicked();
  void onLockAllEdgesClicked();

  void ongraphElementTableValueChanged(QTableWidgetItem *item);

  void onOperateOnAppTypeIndexChanged(int index); //Whether load or save operation needs to be done on graph creator or server
  void onLoadGraphClicked();
  void onSaveGraphClicked();

  void onRefreshClicked();
  
  private:

  //Nodehandles
  ros::NodeHandle m_nh;

  //Messages
  graph_msgs::Vertex m_cur_vertex_msg;
  graph_msgs::Edge m_cur_edge_msg;
  graph_creator_msgs::GraphCreatorState m_cur_gc_state_msg;

  //Publishers
  ros::Publisher m_updated_vertex_pub;
  ros::Publisher m_updated_edge_pub;

  //Subscribers
  ros::Subscriber m_current_vertex_sub;
  ros::Subscriber m_current_edge_sub;
  ros::Subscriber m_gc_state_sub;

  //Subscriber callbacks
  void currentVertexCallback(const graph_msgs::Vertex &vertex_msg);
  void currentEdgeCallback(const graph_msgs::Edge &edge_msg);
  void currentGraphCreatorStateCallback(const graph_creator_msgs::GraphCreatorState &gc_state_msg);

  //Service clients
  ros::ServiceClient m_set_gc_mode_clt;
  ros::ServiceClient m_set_et_clt, m_set_ed_clt;
  ros::ServiceClient m_set_vert_lck_clt, m_set_edge_lck_clt;
  ros::ServiceClient m_get_gc_state_clt;
  ros::ServiceClient m_load_graph_creator_clt, m_save_graph_creator_clt;
  ros::ServiceClient m_load_graph_server_clt;

  std::vector<std::string> m_vertex_types = {"Normal", "Park", "Charge", "Control", "Report"};
  std::vector<std::string> m_edge_type = {"Straight Line", "Cubic Bezier"};
  std::vector<std::string> m_edge_direction_type = {"Forward", "Reverse", "Bidirectional"};

  //UI
  enum GraphElementTypes
  {
    GRAPH_ELEMENT_VERTEX = 1,
    GRAPH_ELEMENT_EDGE = 2,
    GRAPH_ELEMENT_UNKNOWN = 255
  };

  uint8_t m_cur_graph_elem_type = GRAPH_ELEMENT_UNKNOWN;

  enum GraphAppType
  {
    GRAPH_CREATOR = 0,
    GRAPH_SERVER = 1
  };

  uint8_t m_cur_graph_app_type = GRAPH_CREATOR;
  std::string m_cur_graph_app_type_str = "graph creator";

  Ui::graphCreatorPanelForm *m_ui_ptr;

  void createVertexTable(const graph_msgs::Vertex &vertex_msg);
  void createEdgeTable(const graph_msgs::Edge &edge_msg);

  void updateTableVertexValueAndPublish(QTableWidgetItem *item);
  void updateTableEdgeValueAndPublish(QTableWidgetItem *item);

  std::unordered_set<int> m_delegate_rows;
  void resetItemDelegate(int current_row);

  //Graph file
  std::string m_graph_name; //Graph file name
  std::string m_graph_format;
  std::string m_package_name;
  std::string m_graph_folder;

  //Whether to load or save the graph file from or to local storage where RViz is being run
  bool m_load_save_graph_from_local = false;

  bool loadGraphConfirmMessage();
  bool saveGraphConfirmMessage();

  void getGraphFileLoadSaveInformation();
  bool getGraphFileExtensionFromFormat(const std::string &format, std::string &extension);
  bool checkGraphFileLoadSaveInformation();

  void setGraphCreatorState(const graph_creator_msgs::GraphCreatorState &gc_state_msg);
};

}

