#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "tools.hpp"


#include <ros/package.h>


#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <rviz_plugin_urdf_composer/ManageModules.h>

class ModuleSelector
{

    public:
        ModuleSelector(): nh_{}
        {
            service_ =  nh_.advertiseService("manage_modules", &ModuleSelector::service_callback, this);

            /*std::vector<std::string> current_active_components{};

            nh_.setParam(assembly_urdf_namespace_+ "/active_description_elements",current_active_components);
            std::string systemCommand = "rosparam dump " + yaml_path + " " + assembly_urdf_namespace_; //" -o " + outputFile;*/

            std::string package_name = "rviz_plugin_urdf_composer";  // Replace with your actual package name

            std::string package_path = ros::package::getPath(package_name);
            workspace_path_ = package_path ;

            timer_ = nh_.createTimer(ros::Duration(0.1), &ModuleSelector::timerCallback, this);

            updateURDFfromYAML("base_config");

        }

        void timerCallback(const ros::TimerEvent& event)
        {   
            
            if(!robot_state_publisher_) return;

            bool use_tf_static{false};
            robot_state_publisher_->publishFixedTransforms( tf_namespace_, use_tf_static);

            if(joint_names_.size()!=0)
            {
                std::map<std::string, double> joint_positions;
                for (std::string joint_name : joint_names_) {
                    joint_positions.insert(std::make_pair(joint_name,0));
                }
                robot_state_publisher_->publishTransforms(joint_positions, ros::Time::now(), tf_namespace_); 
            }


        }

        void updateURDFfromYAML(std::string config_name)
        {
            std::string yaml_path = workspace_path_ + "/config/"+ config_name + ".yaml";
            std::string urdf_path = workspace_path_ + "/urdf/robotic_cell.urdf.xacro";
      

            // Use the constructor of std::string to load the file content
            std::string fileContent = stringURDFfromYAMLconfig(urdf_path, yaml_path); 
            
            std::string base_filename = urdf_path.substr(urdf_path.find_last_of("/") + 1);

            std::string param_name_namespace = "test";

            std::string param_name = "/" + param_name_namespace + "/robot_description";

            nh_.setParam(param_name,fileContent);
            nh_.setParam("robot_description", "my_string");



            // fill robot_state_publisher

            urdf::Model urdf_model;

            urdf_model.initString(fileContent);

            KDL::Tree tree;
            if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
                ROS_ERROR("Failed to extract kdl tree from xml robot description");

            }

            joint_names_ = std::vector<std::string>{};
            for (const auto& joint_pair : urdf_model.joints_) {
                const urdf::Joint& joint = *joint_pair.second;
                joint_names_.push_back(joint.name);
            }
            std::string systemCommand = "rosparam load " + yaml_path + " " + assembly_urdf_namespace_; //" -o " + outputFile;
            int result = std::system(systemCommand.c_str());


            robot_state_publisher_ = std::make_shared<GuiRobotStatePublisher>(tree, urdf_model);

        }

        bool service_callback(rviz_plugin_urdf_composer::ManageModules::Request  &req,
                rviz_plugin_urdf_composer::ManageModules::Response &res)
        {
            // read current elements
            std::vector<std::string> current_active_components;
            nh_.getParam(assembly_urdf_namespace_+ "/active_description_elements",current_active_components);

            std::string module_name = req.module_name;

            std::vector<std::string> new_active_components;
            
            std::string package_name = "reconcycle_module_camera_desk";
            std::string parrent = "robotic_cell_base";
            std::string urdf_name =  "reconcycle_module_camera_desk.urdf.xacro";

            std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>(nh_);

            writeModuleToParameters(nh_ptr, assembly_urdf_namespace_ + "/" + module_name+ "/", 
                    package_name, urdf_name, parrent, 
                    0, 0, 0,
                    0, 0, 0);

    

            if(req.operation == req.CONNECT)
            {
                for(std::string active_component : current_active_components)
                {
  
                    new_active_components.push_back(active_component);

                }

                new_active_components.push_back(module_name);
            }
            else if(req.operation == req.DISCONNECT)
            {
                                
                for(std::string active_component : current_active_components)
                {
                    if(active_component!=module_name)
                    {
                        new_active_components.push_back(active_component);
                    }
                    
                }
            }



            nh_.setParam(assembly_urdf_namespace_+ "/active_description_elements",new_active_components);

            std::string yaml_path = workspace_path_ + "/config/active_config.yaml";
        
            std::string systemCommand = "rosparam dump " + yaml_path + " " + assembly_urdf_namespace_; //" -o " + outputFile;

            int result = std::system(systemCommand.c_str());


            updateURDFfromYAML("active_config");

            return true;
        }




    private:
        ros::NodeHandle nh_;
        ros::ServiceServer service_;
        std::shared_ptr<GuiRobotStatePublisher> robot_state_publisher_;
        std::string workspace_path_;
        ros::Timer timer_;

        std::string assembly_urdf_namespace_{"reconcycle_modules"};
        std::string tf_namespace_{"test"};
        std::vector<std::string> joint_names_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "module_manager");
  
    ModuleSelector node;
    ROS_INFO("Ready to add two ints.");
    ros::spin();

    return 0;
}