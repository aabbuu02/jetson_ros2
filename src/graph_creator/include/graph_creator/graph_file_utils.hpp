#pragma once

#include <string>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>

namespace Graph::File {

   bool getGraphFilePath(const std::string &in_file_path, std::string &out_file_path); //An absolute path or a package relative path can be given

}