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

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QComboBox>
#include <QString>
#include <QFileDialog>
#include <QPushButton>

#include <iostream>
#include <fstream>

#include <geometry_msgs/Twist.h>

#include "drive_widget.h"
#include "teleop_panel.h"

#include "robot_state_publisher/robot_state_publisher.h"

namespace rviz_urdf_composer
{

// BEGIN_TUTORIAL
// Here is the implementation of the TeleopPanel class.  TeleopPanel
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  /*QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Output1 Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );*/

  QComboBox* comboBox = new QComboBox(this);

  // Get the list of parameters in the global namespace
  std::vector<std::string> param_names;
  nh_.getParamNames(param_names);
 

  for( std::string param_name : param_names )
  { 
    QString param_name_q = QString::fromStdString(param_name);
    comboBox->addItem(param_name_q);
  }


  // Loading base urdf

  QPushButton *button = new QPushButton("Choose File", this);
  QHBoxLayout* base_urdf_layout = new QHBoxLayout;
  base_urdf_layout->addWidget( button );
  connect(button, &QPushButton::clicked, this, &TeleopPanel::openFileDialog);


  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  //layout->addLayout( topic_layout );
  layout->addLayout( base_urdf_layout );
  layout->addWidget( comboBox );
  setLayout( layout );

  // Create a timer for sending the output.  Motor controllers want to
  // be reassured frequently that they are doing the right thing, so
  // we keep re-sending velocities even when they aren't changing.
  // 
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this TeleopPanel
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.
  QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  //connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendTFs() ));

  // Start the timer.
  output_timer->start(1000);





}

// setVel() is connected to the DriveWidget's output, which is sent
// whenever it changes due to a mouse event.  This just records the
// values it is given.  The data doesn't actually get sent until the
// next timer callback.


void TeleopPanel::openFileDialog() {

    QString fileName = QFileDialog::getOpenFileName(this, "Open File", QDir::homePath(), "All Files (*.*)");

    urdf_path_assembly_ = fileName.toStdString();

    
    std::ifstream file(urdf_path_assembly_);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << urdf_path_assembly_ << std::endl;
        
    }

    // Use the constructor of std::string to load the file content
    std::string fileContent((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    if (!fileName.isEmpty()) {
        //qDebug() << "Selected file: " << fileName;
        // Do something with the selected file...
    }

    nh_.setParam("/assembly_urdf_model/robot_description",fileContent);
    assembly_urdf_model_.initString(fileContent);

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(assembly_urdf_model_, tree)) {
      ROS_ERROR("Failed to extract kdl tree from xml robot description");
  
    }

    // Get the names of all joints in the robot model

    assembly_joint_names_ = std::vector<std::string> {};
    for (const auto& joint_pair : assembly_urdf_model_.joints_) {
        const urdf::Joint& joint = *joint_pair.second;
        assembly_joint_names_.push_back(joint.name);
    }



    state_publisher_ = std::make_shared<robot_state_publisher::RobotStatePublisher>(tree, assembly_urdf_model_);
    assembly_urdf_model_ready_ = true;

    bool use_tf_static{true};
    state_publisher_->publishFixedTransforms( "assembly_urdf_model", use_tf_static);
  }


void TeleopPanel::sendTFs()
{

  ROS_INFO("TREST");

  if(assembly_urdf_model_ready_)
  {
    std::map<std::string, double> joint_positions;

    

    for (std::string joint_name : assembly_joint_names_) {
      joint_positions.insert(std::make_pair(joint_name,0));
    }

    state_publisher_->publishTransforms(joint_positions, ros::Time::now(), "assembly_urdf_model"); //
  }
}

void TeleopPanel::save( rviz::Config config ) const
{
  /*rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );*/
}

// Load all configuration data for this panel from the given Config object.
void TeleopPanel::load( const rviz::Config& config )
{
 /*rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    //output_topic_editor_->setText( topic );
    //updateTopic();
  }*/
}



} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_urdf_composer::TeleopPanel,rviz::Panel )
// END_TUTORIAL
