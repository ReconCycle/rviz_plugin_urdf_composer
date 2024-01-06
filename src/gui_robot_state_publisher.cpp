
#include <stdio.h>
#include <math.h>

#include "gui_robot_state_publisher.h"
namespace rviz_urdf_composer
{

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

} // end namespace rviz_plugin_tutorials
