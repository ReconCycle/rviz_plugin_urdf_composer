
#ifndef TOOLS_HPP
#define TOOLS_HPP

    #include <stdio.h>
    #include <iostream>
    #include <fstream>
    #include <ros/ros.h>
    #include <kdl/chainfksolverpos_recursive.hpp>
    #include <kdl/frames.hpp>

    #include "robot_state_publisher/robot_state_publisher.h"

    std::string stringURDFfromYAMLconfig(std::string urdf_path, std::string composition_yaml = "None");

    bool writeModuleToParameters(std::shared_ptr<ros::NodeHandle> nh_ptr, std::string module_namespace, std::string package, std::string urdf_name, std::string parrent_name, 
        double x, double y, double z, double roll, double pitch, double yaw);

    KDL::Frame calculateTransformKDLTree(KDL::Tree kdl_tree, std::string link_name);

    class GuiRobotStatePublisher: public robot_state_publisher::RobotStatePublisher
    {

        
        public:

            GuiRobotStatePublisher(const KDL::Tree& tree, const urdf::Model& model = urdf::Model()); 

            std::vector<std::string> getSegmentList();
            std::vector<std::string> getTfNames();

    };

#endif /* !TOOLS_HPP */