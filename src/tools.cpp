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

