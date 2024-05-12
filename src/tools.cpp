#include "tools.hpp"

std::string stringURDFfromYAMLconfig(std::string urdf_path, std::string composition_yaml)
{   
    std::ifstream file(urdf_path);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << urdf_path << std::endl;
        
    }
    
    std::string base_filename = urdf_path.substr(urdf_path.find_last_of("/") + 1);
    
    bool xacro_process{false};
    if(base_filename.find("xacro") != std::string::npos)
    { 
        xacro_process = true;
    }

        if(xacro_process)
        {
        // Construct the xacro command
        std::string xacroCommand = "xacro "+ urdf_path ; 

        if(composition_yaml!="None")
        {
            xacroCommand = xacroCommand + " yaml_path:='" + composition_yaml + "' ";
        }
        
        xacroCommand = xacroCommand + " > tmp.urdf"; //" -o " + outputFile;
        std::cerr << "Command: " << xacroCommand << std::endl;
        // Execute the xacro command
        int result = std::system(xacroCommand.c_str());
        
        file = std::ifstream("tmp.urdf");
        if (!file.is_open()) 
        {
            std::cerr << "Error: Unable to open file " << "tmp.urdf" << std::endl;
        }

        std::remove("tmp.urdf");
            
        }

    
    // Use the constructor of std::string to load the file content
    std::string fileContent((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return fileContent;
}


bool writeModuleToParameters(std::shared_ptr<ros::NodeHandle> nh_ptr, std::string module_namespace,
        std::string package, std::string urdf_name, std::string parrent_name, 
        double x, double y, double z, double roll, double pitch, double yaw)
{

  nh_ptr->setParam(module_namespace+ "x", x);
  nh_ptr->setParam(module_namespace+ "y", y);
  nh_ptr->setParam(module_namespace+ "z", z);


  nh_ptr->setParam(module_namespace+ "ep",pitch);
  nh_ptr->setParam(module_namespace+ "er",roll);
  nh_ptr->setParam(module_namespace+ "ey",yaw);


  nh_ptr->setParam(module_namespace+ "package_name",package);
  nh_ptr->setParam(module_namespace+ "parrent", parrent_name);
  nh_ptr->setParam(module_namespace+ "urdf_name", urdf_name);

  return true;
}



GuiRobotStatePublisher::GuiRobotStatePublisher(const KDL::Tree& tree, const urdf::Model& model) : robot_state_publisher::RobotStatePublisher( tree, model)
{

}

std::vector<std::string> GuiRobotStatePublisher::getSegmentList()
{
    std::vector<std::string> segment_list{};

    for( auto segment_pair : segments_fixed_)
    {
        segment_list.push_back(segment_pair.first);
    }

    for( auto segment_pair : segments_)
    {
        segment_list.push_back(segment_pair.first);
    }

    return segment_list;
}

std::vector<std::string> GuiRobotStatePublisher::getTfNames()
{
    std::vector<std::string> segment_list{};

    for( auto segment_pair : segments_fixed_)
    {
        std::vector<std::string> both_ends{segment_pair.second.root,segment_pair.second.tip};

        for( auto name : both_ends)
        {
            if (std::find(segment_list.begin(), segment_list.end(), name) == segment_list.end()) {
                segment_list.push_back(name);        
            }
        }

    }


    for( auto segment_pair : segments_)
    {
        std::vector<std::string> both_ends{segment_pair.second.root,segment_pair.second.tip};

        for( auto name : both_ends)
        {
            if (std::find(segment_list.begin(), segment_list.end(), name) == segment_list.end()) {
                segment_list.push_back(name);        
            }
        }
    }

    return segment_list;
}


