
#ifndef DRIVE_WIDGET_H
#define DRIVE_WIDGET_H


#include "robot_state_publisher/robot_state_publisher.h"

namespace rviz_urdf_composer
{

// BEGIN_TUTORIAL
// DriveWidget implements a control which translates mouse Y values
// into linear velocities and mouse X values into angular velocities.
//
// For maximum reusability, this class is only responsible for user
// interaction and display inside its widget.  It does not make any
// ROS or RViz calls.  It communicates its data to the outside just
// via Qt signals.
class GuiRobotStatePublisher: public robot_state_publisher::RobotStatePublisher
{

    
    public:

        GuiRobotStatePublisher(const KDL::Tree& tree, const urdf::Model& model = urdf::Model()); 

        std::vector<std::string> getSegmentList();
        std::vector<std::string> getTfNames();

};
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials


#endif // DRIVE_WIDGET_H
