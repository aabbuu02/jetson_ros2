#include "create_vertex_tool.hpp"

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(graph_creator_rviz_plugin::CreateVertexTool, rviz::Tool)

namespace graph_creator_rviz_plugin
{

CreateVertexTool::CreateVertexTool()
{
  shortcut_key_ = 'v';

  topic_property_ = new rviz::StringProperty("Topic", "add_vertex_pose", "The topic adding vertex with pose",
                                              getPropertyContainer(), SLOT(updateTopic()), this);

  auto_deactivate_property_ = new rviz::BoolProperty("Single click", false, "Switch away from this tool after one click.", 
                                                      getPropertyContainer(), SLOT(updateAutoDeactivate()), this);
}

CreateVertexTool::~CreateVertexTool()
{
  delete topic_property_;
  delete auto_deactivate_property_;
}

void CreateVertexTool::onInitialize()
{
  arrow_ = new rviz::Arrow(scene_manager_, NULL, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(1.0f, 0.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
  setName("Create Vertex");
  updateTopic();
}

void CreateVertexTool::activate()
{
  setStatus("Click and drag mouse to set position and orientation for the vertex.");
  state_ = Position;
}

void CreateVertexTool::updateTopic()
{
  try
  {
    vertex_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1);
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("create_vertex_tool", e.what());
  }
}

void CreateVertexTool::updateAutoDeactivate()
{

}

void CreateVertexTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  geometry_msgs::PoseStamped goal;
  goal.pose.orientation = tf2::toMsg(quat);
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.header.frame_id = fixed_frame;
  goal.header.stamp = ros::Time::now();
  
  ROS_INFO("Create vertex: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = "
           "Angle: %.3f\n",
           fixed_frame.c_str(), goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
           goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z,
           goal.pose.orientation.w, theta);
  
  vertex_pub_.publish(goal);
}

int CreateVertexTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  int flags = 0;

  if(event.leftDown())
  {
    ROS_ASSERT(state_ == Position);

    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
    if(rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, intersection))
    {
      pos_ = intersection;
      arrow_->setPosition( pos_ );

      state_ = Orientation;
      flags |= Render;
    }
  }
  else if(event.type == QEvent::MouseMove && event.left())
  {
    if(state_ == Orientation)
    {
      //Compute angle in x-y plane
      Ogre::Vector3 cur_pos;
      Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
      if(rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, cur_pos))
      {
        double angle = atan2( cur_pos.y - pos_.y, cur_pos.x - pos_.x );
        arrow_->getSceneNode()->setVisible(true);

        //We need base_orient, since the arrow goes along the -z axis by default (for historical reasons)
        Ogre::Quaternion orient_x = Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);

        arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);

        flags |= Render;
      }
    }
  }
  else if(event.leftUp())
  {
    if(state_ == Orientation)
    {
      //Compute angle in x-y plane
      Ogre::Vector3 cur_pos;
      Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
      if(rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, cur_pos ))
      {
        double angle = atan2( cur_pos.y - pos_.y, cur_pos.x - pos_.x );
        onPoseSet(pos_.x, pos_.y, angle);
        if (auto_deactivate_property_->getBool() )
        {
            flags |= (Finished|Render);
        }
        else if (!auto_deactivate_property_->getBool())
        {
            state_ = Position;
            arrow_->getSceneNode()->setVisible(false);
        }
      }
    }
  }
  
  return flags;
}

}