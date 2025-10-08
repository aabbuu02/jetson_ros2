#include "graph_creator/graph_file_utils.hpp"

namespace Graph::File {

    /**
     * @brief Get the graph file path using either absolute path or a relative package.
     * For absolute paths, this function checks whether it is a valid absolute path else
     * a full absolute path with respect to a package is retrieved. 
     * 
     * Usage examples are provided below
     * For absolute path: /path/to/graph.graphml
     * For relative path: package/path/to/graph.graphml
     * 
     * @param in_file_path The input file path either absolute or relative to a package
     * @param out_file_path The absolute path to graph file
     * @return true If path was retrieved successfully
     * @return false If an invalid absolute or relative path was given
     */

    bool getGraphFilePath(const std::string &in_file_path, std::string &out_file_path)
    {
        //TODO: An extension check needs to be done
        out_file_path = "";

        if(in_file_path.size() == 0)
        {
            ROS_ERROR_STREAM("The file path provided is empty");
            return false;
        }

        boost::filesystem::path graph_file_path(in_file_path);
        if(!graph_file_path.is_absolute())
        {
            std::string package_name = graph_file_path.begin()->string();
            ROS_INFO_STREAM("Retrieving path relative to package: "<<package_name);
            std::string package_path_str = ros::package::getPath(package_name);
            if(package_path_str.size() == 0)
            {
                ROS_ERROR_STREAM("Could not locate graph relative to package");
                return false;
            }

            boost::filesystem::path package_path(package_path_str);
            out_file_path = (package_path.parent_path()/graph_file_path).string();
            return true;
        }
       
        out_file_path = graph_file_path.string();
        return true;
    }
}