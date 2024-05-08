#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "tools.hpp"

#include <ros/package.h>


bool add(std_srvs::Trigger::Request  &req,
         std_srvs::Trigger::Response &res)
{
  //res.sum = req.a + req.b;
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}


class ModuleSelector
{

    public:
        ModuleSelector(): nh_{}
        {
            service_ =  nh_.advertiseService("add_two_ints", add);

            /*std::vector<std::string> current_active_components{};

            nh_.setParam(assembly_urdf_namespace_+ "/active_description_elements",current_active_components);
            std::string systemCommand = "rosparam dump " + yaml_path + " " + assembly_urdf_namespace_; //" -o " + outputFile;*/

            std::string package_name = "rviz_plugin_urdf_composer";  // Replace with your actual package name

            std::string package_path = ros::package::getPath(package_name);
            workspace_path_ = package_path ;


            std::string yaml_path = workspace_path_ + "/config/active_config.yaml";

           

            //PRELOAD EMPTY ASEMBLY
            std::string urdf_path = workspace_path_ + "/urdf/robotic_cell.urdf.xacro";
      

            // Use the constructor of std::string to load the file content
            std::string fileContent = stringURDFfromYAMLconfig(urdf_path, yaml_path); 
            
            std::string base_filename = urdf_path.substr(urdf_path.find_last_of("/") + 1);

            std::string param_name_namespace = "test";

            std::string param_name = "/" + param_name_namespace + "/robot_description";

            nh_.setParam(param_name,fileContent);
            nh_.setParam("robot_description", "my_string");
        }

    private:
        ros::NodeHandle nh_;
        ros::ServiceServer service_;
        std::string workspace_path_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_server");
  
    ModuleSelector node;
    ROS_INFO("Ready to add two ints.");
    ros::spin();

    return 0;
}