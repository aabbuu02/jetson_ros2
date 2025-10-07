#include "select_graph_elements_tool.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(graph_creator_rviz_plugin::SelectGraphElementsTool, rviz::Tool)


namespace graph_creator_rviz_plugin
{

SelectGraphElementsTool::SelectGraphElementsTool()
{
  topic_property_ = new rviz::StringProperty("Topic", "graph_creator/selected_graph_elements", "The topic for selected graph elements",
                                              getPropertyContainer(), SLOT(updateTopic()), this);
  
  auto_deactivate_property_ = new rviz::BoolProperty("Single selection", false, "Switch away from this tool after selection.", 
                                                      getPropertyContainer(), SLOT(updateAutoDeactivate()), this);
}

SelectGraphElementsTool::~SelectGraphElementsTool()
{
  delete topic_property_;
}

void SelectGraphElementsTool::onInitialize()
{
  rviz::SelectionTool::onInitialize();
  setName("Select Graph Elements");
  updateTopic();
}

void SelectGraphElementsTool::activate()
{
  rviz::SelectionTool::activate();
  setStatus("Click and drag to select graph elements on the screen.");
}

void SelectGraphElementsTool::updateTopic()
{
  try
  {
    selection_pub_ = nh_.advertise<graph_creator_msgs::SelectedGraphElements>(topic_property_->getStdString(), 1);
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("selected_graph_elements_tool", e.what());
  }

  selected_marker_count_ = 0;
}

int SelectGraphElementsTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
{
  return Render;
}

int SelectGraphElementsTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  int flags = rviz::SelectionTool::processMouseEvent(event);
  
  if(event.alt()) selecting_ = false;
  else if(event.leftDown()) selecting_ = true;

  if(selecting_ && event.leftUp()) 
  {
    processSelectedArea();
    if(!auto_deactivate_property_->getBool()) flags |= Finished | Render;
  }

  return flags;
}

void SelectGraphElementsTool::updateAutoDeactivate()
{

}

void SelectGraphElementsTool::processSelectedArea()
{
  selected_elements_msg_.selected_elements.clear();
  selected_marker_set_.clear();

  rviz::SelectionManager* selection_manager = context_->getSelectionManager();
  rviz::M_Picked selection = selection_manager->getSelection();
  rviz::PropertyTreeModel* model = selection_manager->getPropertyModel();

  int i = 0;
  while(model->hasIndex(i, 0))
  {
    QModelIndex child_index = model->index(i, 0);

    rviz::Property* child = model->getProp(child_index);
    auto splitStrLst = child->getName().split(" ");

    std::string type_str = "";
    std::string marker_name_str = "";

    if(splitStrLst.size() > 1)
    {
      type_str = splitStrLst.front().toStdString();
      marker_name_str = splitStrLst.back().split("/").front().toStdString();
    }

    if(marker_name_str.size() > 0 && type_str == "Marker")
    {
      ROS_DEBUG_STREAM_NAMED("selected_area_processing", "Selected marker(s): "<<marker_name_str);
      if(selected_marker_set_.emplace(marker_name_str).second)
      {
        ROS_DEBUG_STREAM_NAMED("selected_area_processing","Added unique marker with name "<<marker_name_str);
        selected_elements_msg_.selected_elements.push_back(marker_name_str);
      }
      else ROS_DEBUG_STREAM_NAMED("selected_area_processing", "Already added marker with name "<<marker_name_str);
    }

    i++;
  }

  selected_marker_count_ = selected_marker_set_.size();
  
  ROS_INFO_STREAM_NAMED("selected_area_processing",
                        "Number of elements in the selected area: "<<selected_marker_count_
                        <<" (Unfiltered selection count: "<<i<<")");
  
  if(selected_elements_msg_.selected_elements.size() > 0)
    selection_pub_.publish(selected_elements_msg_);
}

}