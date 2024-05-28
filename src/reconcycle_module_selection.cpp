#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "tools.hpp"


#include <ros/package.h>


#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

#include <rviz_plugin_urdf_composer/ManageModules.h>

class ModuleSelector
{

    public:
        ModuleSelector(): nh_{}
        {
            service_ =  nh_.advertiseService("manage_modules", &ModuleSelector::service_callback, this);
            service_client_update_ = nh_.serviceClient<std_srvs::Trigger>("update_vis_urdf");

            /*std::vector<std::string> current_active_components{};

            nh_.setParam(assembly_urdf_namespace_+ "/active_description_elements",current_active_components);
            std::string systemCommand = "rosparam dump " + yaml_path + " " + assembly_urdf_namespace_; //" -o " + outputFile;*/

            std::string package_name = "rviz_plugin_urdf_composer";  // Replace with your actual package name

            std::string package_path = ros::package::getPath(package_name);
            workspace_path_ = package_path ;

            timer_ = nh_.createTimer(ros::Duration(0.1), &ModuleSelector::timerCallback, this);



            updateURDFfromYAML("base_config");

            // Wait for the service to become available
            ROS_INFO("Waiting for the update_vis_urdf service to become available...");
            service_client_update_.waitForExistence();
            ROS_INFO("Service is now available.");
            std_srvs::Trigger srv;
            service_client_update_.call(srv);


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

            std::string param_name_namespace = tf_namespace_;

            std::string param_name = "/" + param_name_namespace + "/robot_description";//

            nh_.setParam(param_name,fileContent);


            // fill robot_state_publisher

            urdf::Model urdf_model;

            urdf_model.initString(fileContent);

            if (!kdl_parser::treeFromUrdfModel(urdf_model, composition_kdl_tree_)) {
                ROS_ERROR("Failed to extract kdl tree from xml robot description");

            }

            joint_names_ = std::vector<std::string>{};
            for (const auto& joint_pair : urdf_model.joints_) {
                const urdf::Joint& joint = *joint_pair.second;
                joint_names_.push_back(joint.name);
            }
            std::string systemCommand = "rosparam load " + yaml_path + " " + assembly_urdf_namespace_; //" -o " + outputFile;
            int result = std::system(systemCommand.c_str());


            robot_state_publisher_ = std::make_shared<GuiRobotStatePublisher>(composition_kdl_tree_, urdf_model);

        }

        bool service_callback(rviz_plugin_urdf_composer::ManageModules::Request  &req,
                rviz_plugin_urdf_composer::ManageModules::Response &res)
        {
            // read current elements
            std::vector<std::string> current_active_components;
            nh_.getParam(assembly_urdf_namespace_+ "/active_description_elements",current_active_components);

            std::string module_name = req.module_name;

            std::vector<std::string> new_active_components;
            
            std::string package_name = req.package_name ;// "reconcycle_module_camera_desk";
            std::string parrent = "robotic_cell_base";
            std::string parrent_module_name = req.parrent_module_name; // "module_cnc";
            std::string urdf_name = req.urdf_name ;// "reconcycle_module_camera_desk.urdf.xacro";

            std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>(nh_);

            int module_plug_id = req.module_plug_id;
            int parrent_module_plug_id = req.parrent_module_plug_id;

            std::string parrent_plug_id = std::to_string(parrent_module_plug_id);
            ROS_ERROR_STREAM(parrent_module_plug_id);
            if(parrent_module_plug_id==3)
            {
                parrent_plug_id = "";
            }

          
 
            std::string parrent_tf_name =  parrent_module_name + "_pnpf" + parrent_plug_id;
            std::string module_tf_name = "pnpm" + std::to_string(module_plug_id); //tf_namespace_ + "/" + module_name + "_pnpm" + std::to_string(module_plug_id);

            ROS_ERROR_STREAM(parrent_tf_name);
 ROS_ERROR_STREAM(module_tf_name);
            ///////////////////////////////


            
            KDL::Frame eeFrameComp = calculateTransformKDLTree(composition_kdl_tree_, parrent_tf_name);

            //Calculte component plug tf
            std::string package_path = ros::package::getPath(package_name);
            
            std::string urdf_path = package_path + "/urdf/" + urdf_name ; 

            std::string component_string_urdf = stringURDFfromYAMLconfig(urdf_path);

            urdf::Model urdf_model;
            urdf_model.initString(component_string_urdf);
            KDL::Tree kdl_tree;

            if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
                ROS_ERROR("Failed to extract kdl tree from xml robot description");

            }

            KDL::Frame eeFrame = calculateTransformKDLTree(kdl_tree,module_tf_name);
            KDL::Frame total_trans = KDL::Frame();
            total_trans = eeFrameComp*eeFrame.Inverse();

            double roll, pitch, yaw;    
            total_trans.M.GetRPY(roll, pitch, yaw);


            writeModuleToParameters(nh_ptr, assembly_urdf_namespace_ + "/" + module_name+ "/", 
                    package_name, urdf_name, parrent, 
                    total_trans.p.x(), total_trans.p.y(), total_trans.p.z(),
                    roll, pitch, yaw);

            ROS_INFO_STREAM(total_trans.p.x());
            ROS_INFO_STREAM(total_trans.p.y());
            ROS_INFO_STREAM(total_trans.p.z());
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
            std_srvs::Trigger srv;
            service_client_update_.call(srv);

            return true;
        }




    private:
        ros::NodeHandle nh_;
        ros::ServiceServer service_;
        ros::ServiceClient service_client_update_;
        std::shared_ptr<GuiRobotStatePublisher> robot_state_publisher_;
        std::string workspace_path_;
        ros::Timer timer_;

         KDL::Tree composition_kdl_tree_;

        std::string assembly_urdf_namespace_{"reconcycle_modules"};

        std::string tf_namespace_{"reconcycle_cell"};
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