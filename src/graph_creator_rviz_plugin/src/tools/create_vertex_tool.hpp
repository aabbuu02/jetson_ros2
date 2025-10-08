#pragma once

#ifndef Q_MOC_RUN 
#include <QObject>

#include <ros/ros.h>
#include <rviz/default_plugin/tools/pose_tool.h>
#endif

#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include <rviz/geometry.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/load_resource.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>


namespace graph_creator_rviz_plugin
{

class CreateVertexTool: public rviz::PoseTool
{

  Q_OBJECT

public:

  CreateVertexTool();
  ~CreateVertexTool() override;

  void onInitialize() override;

  void activate() override;

  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

protected:
  
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  
  void updateTopic();
  void updateAutoDeactivate();

private:
  
  //Node handles
  ros::NodeHandle nh_;
  
  //Publishers
  ros::Publisher vertex_pub_;

  //Properties
  rviz::StringProperty* topic_property_;
  rviz::BoolProperty* auto_deactivate_property_;
};

}