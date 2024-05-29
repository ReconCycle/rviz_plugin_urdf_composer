/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */




#include "urdf_vis_update.h"




//#include <tf2/LinearMath/Transform.h>

namespace rviz_urdf_composer
{



UrdfVisUpdate::UrdfVisUpdate( QWidget* parent )
  : rviz::Panel( parent )
{
  ROS_INFO_STREAM("test!");

    // // Add URDF display to VisualizationManager
    // rviz::RobotModelDisplay urdf_display;
    // rviz::Display* display = urdf_display.createDisplay();
    // if (display) {
    //     vis_manager_->addDisplay(display, true);
    // }
    // rviz::DisplayGroup * 	 displays1 = vis_manager_->getRootDisplayGroup();
    //urdf_display_ =  vis_manager_->createDisplay("rviz/RobotModel", "reconcycleCell", false);
    service_ =  nh_.advertiseService("update_vis_urdf", &UrdfVisUpdate::service_callback, this);
    ROS_INFO_STREAM("test2!");

  
}

UrdfVisUpdate::~UrdfVisUpdate(){}


void UrdfVisUpdate::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  /*config.mapSetValue( "Topic", output_topic_ );*/
}

// Load all configuration data for this panel from the given Config object.
void UrdfVisUpdate::load( const rviz::Config& config )
{
 rviz::Panel::load( config );
  /*QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    //output_topic_editor_->setText( topic );
    //updateTopic();
  }*/
}

bool UrdfVisUpdate::service_callback(std_srvs::Trigger::Request  &req,
            std_srvs::Trigger::Response &res)
{
  ROS_INFO_STREAM("test!");

    std::string display_name = "reconcycleCell";
    rviz::DisplayGroup * 	 displays = vis_manager_->getRootDisplayGroup();
    int num_of_display = displays->numDisplays();
    for(int i=0;i<num_of_display;i++)
    { 
      std::string panel_name = displays->getDisplayAt(i)->getName().toStdString();
      if(panel_name == display_name)
      { 
        displays->getDisplayAt(i)->setEnabled(false);
        displays->getDisplayAt(i)->setEnabled(true);

        return true;
      }

      //ROS_INFO_STREAM("dis: " << panel_name);
    }
    
    vis_manager_->setFixedFrame("world");///reconcycle_cell/robotic_cell_base
    urdf_display_ =  vis_manager_->createDisplay("rviz/RobotModel", QString::fromStdString(display_name), true);
    urdf_display_->subProp("Robot Description")->setValue(QString("/reconcycle_generic/robot_description"));// /reconcycle_cell/robot_description
    urdf_display_->subProp("TF Prefix")->setValue(QString("reconcycle_generic"));// reconcycle_cell

    ROS_INFO_STREAM("test2!");
  return true;
}
 



} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_urdf_composer::UrdfVisUpdate, rviz::Panel )
// END_TUTORIAL
