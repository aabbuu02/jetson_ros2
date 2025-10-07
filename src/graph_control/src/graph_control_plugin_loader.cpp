#include <graph_control/graph_control_plugin_loader.hpp>

GraphControlPluginLoader::GraphControlPluginLoader():
    m_graph_control_loader("graph_control", "graph_control::BaseGraphControl")
{

}

bool GraphControlPluginLoader::isGraphControlPluginLoaded()
{
    return m_is_plugin_loaded;
}

bool GraphControlPluginLoader::loadGraphControlPlugin(const std::string &graph_control_plugin_name)
{
    m_is_plugin_loaded = false;

    try
    {
        m_graph_control_inst = m_graph_control_loader.createInstance(graph_control_plugin_name);
        m_is_plugin_loaded = true;
        return true;
    }
    catch(pluginlib::PluginlibException& e)
    {
        ROS_ERROR_STREAM("Cannot load "<<graph_control_plugin_name<<" graph control plugin. Failed with exception: "<<e.what());
        m_is_plugin_loaded = false;
        return false;
    }
}

boost::shared_ptr<graph_control::BaseGraphControl> GraphControlPluginLoader::getGraphControlInstance()
{
    return m_graph_control_inst;
}