#pragma once

#include <string>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <graph_control/base_graph_control.hpp>

class GraphControlPluginLoader
{
    public:

    GraphControlPluginLoader();

    bool isGraphControlPluginLoaded();
    bool loadGraphControlPlugin(const std::string &graph_control_plugin_name);
    boost::shared_ptr<graph_control::BaseGraphControl> getGraphControlInstance();

    private:

    bool m_is_plugin_loaded = false;
    pluginlib::ClassLoader<graph_control::BaseGraphControl> m_graph_control_loader;
    boost::shared_ptr<graph_control::BaseGraphControl> m_graph_control_inst;
};