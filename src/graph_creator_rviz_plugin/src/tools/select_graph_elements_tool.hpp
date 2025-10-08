#pragma once

#include <string>
#include <unordered_set>

#ifndef Q_MOC_RUN
#include <QObject>
#include <QCursor>
#include <QVariant>
#include <QWidget>

#include <ros/ros.h>
#include <rviz/tool.h>
#include <rviz/default_plugin/tools/selection_tool.h>
#endif

#include <rviz/view_manager.h>
#include <rviz/display_context.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/property_tree_model.h>

#include <graph_creator_msgs/SelectedGraphElements.h>

#include <OGRE/OgreCamera.h>

namespace graph_creator_rviz_plugin
{

class SelectGraphElementsTool: public rviz::SelectionTool
{

  Q_OBJECT

public:

  SelectGraphElementsTool();
  ~SelectGraphElementsTool() override;

  void onInitialize() override;
  void activate() override;

  int processMouseEvent(rviz::ViewportMouseEvent& event) override;
  int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

private Q_SLOTS:

  void updateTopic();
  void updateAutoDeactivate();

private:

  //Selection variables
  bool selecting_ = false;
  int selected_marker_count_ = 0;
  std::unordered_set<std::string> selected_marker_set_;

  //Nodehandles
  ros::NodeHandle nh_;

  //Messages
  graph_creator_msgs::SelectedGraphElements selected_elements_msg_;

  //Publishers
  ros::Publisher selection_pub_;

  //Properties
  rviz::StringProperty* topic_property_;
  rviz::BoolProperty* auto_deactivate_property_;

  void processSelectedArea();
};

}