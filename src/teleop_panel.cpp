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


#include <iostream>
#include <fstream>




#include "teleop_panel.h"

#include <rviz/visualization_manager.h>
#include <rviz/display_factory.h>
#include <rviz/display_group.h>

#include <kdl/chainfksolverpos_recursive.hpp>


#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2/convert.h>

#include "marker_functions.h"
//#include <tf2/LinearMath/Transform.h>

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

  QVBoxLayout* root_layout = new QVBoxLayout;

  marker_tf_transform_ = geometry_msgs::TransformStamped();;

  marker_tf_transform_.transform.rotation.w=1;

  UrdfManager urdf_manager_assembly;
  UrdfManager urdf_manager_component;


  urdf_manager_assembly.urdf_display_name = "RobotModelBase";
  urdf_manager_component.urdf_display_name = "RobotModelComponent";



  urdf_managers_["component_urdf_model"] = urdf_manager_component;
  urdf_managers_["assembly_urdf_model"] = urdf_manager_assembly;

  QComboBox* comboBox = new QComboBox(this);

  // Get the list of parameters in the global namespace
  std::vector<std::string> param_names;
  nh_.getParamNames(param_names);
 

  for( std::string param_name : param_names )
  { 
    QString param_name_q = QString::fromStdString(param_name);
    comboBox->addItem(param_name_q);
  }




  for(std::string urdf_namespace : std::vector<std::string>{"assembly_urdf_model","component_urdf_model"})
  {
    
    urdf_managers_[urdf_namespace].load_urdf_button  = new QPushButton("Choose urdf file", this);
    connect(urdf_managers_[urdf_namespace].load_urdf_button, &QPushButton::clicked, this, [this, urdf_namespace]{loadURDFtoParam(urdf_namespace );});

    QComboBox*  chose_urdf_tf_combo_box = new QComboBox(this);
    chose_urdf_tf_combo_box->addItem("urdf not chosen yet");
    urdf_managers_[urdf_namespace].tf_combo_box = chose_urdf_tf_combo_box;
    urdf_managers_[urdf_namespace].qt_control_layout = new QVBoxLayout;


    //Set TF
    
    
    urdf_managers_[urdf_namespace].pose_transforms = std::vector<geometry_msgs::TransformStamped>{};
    geometry_msgs::TransformStamped tf_transform = createInitTF("map" , "/" +  urdf_namespace +"/base" ); 
    urdf_managers_[urdf_namespace].pose_transforms.push_back(tf_transform);

  }

  urdf_managers_["assembly_urdf_model"].qt_control_layout->addWidget( new QLabel( "DEFINITION OF BASE URDF:" ));
  urdf_managers_["component_urdf_model"].qt_control_layout->addWidget( new QLabel( "DEFINITION OF COMPONENT URDF:" ));

  for(std::string urdf_namespace : std::vector<std::string>{"assembly_urdf_model","component_urdf_model"})
  {
    
    QHBoxLayout* chose_urdf_layout = new QHBoxLayout;
    chose_urdf_layout->addWidget( urdf_managers_[urdf_namespace].load_urdf_button );
    chose_urdf_layout->addWidget( new QLabel( "Not chosen yet" ));

    QHBoxLayout* chose_urdf_tf_layout = new QHBoxLayout;
    chose_urdf_tf_layout->addWidget( new QLabel( "Chose base tf:" ));
    chose_urdf_tf_layout->addWidget( urdf_managers_[urdf_namespace].tf_combo_box );


    urdf_managers_[urdf_namespace].qt_control_layout->addLayout(chose_urdf_layout);
    urdf_managers_[urdf_namespace].qt_control_layout->addLayout(chose_urdf_tf_layout);

    root_layout->addLayout( urdf_managers_[urdf_namespace].qt_control_layout );

  }



  connect(urdf_managers_["assembly_urdf_model"].tf_combo_box, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &TeleopPanel::onComboBoxIndexChangedBase);
  connect(urdf_managers_["component_urdf_model"].tf_combo_box, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &TeleopPanel::onComboBoxIndexChangedComponent);
  


  QPushButton *button_save_urdf = new QPushButton("save urdf", this);

  QHBoxLayout* save_urdf_layout = new QHBoxLayout;
  save_urdf_layout->addWidget( button_save_urdf );

  // Lay out the topic field above the control widget.

  //layout->addLayout( topic_layout );

  root_layout->addLayout( save_urdf_layout );


  root_layout->addWidget( comboBox );
  setLayout( root_layout );

  // Create a timer for sending the output.  Motor controllers want to
  // be reassured frequently that they are doing the right thing, so
  // we keep re-sending velocities even when they aren't changing.
  // 
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this TeleopPanel
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.

  // INTERACITE MARKER
  interactive_marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("basic_controls","",false );
  tf2::Vector3 position = tf2::Vector3( 0, 0, 0);
  make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );
  interactive_marker_server_->applyChanges();

   ROS_INFO_STREAM("out");

  QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  //connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendTFs() ));

  // Start the timer.
  output_timer->start(100);





}

void  TeleopPanel::loadURDFtoParam(std::string param_name_namespace)
{

  QString fileName = QFileDialog::getOpenFileName(this, "Open File", QDir::homePath(), "All Files (*.*)");

  std::string urdf_path = fileName.toStdString();

  
  std::ifstream file(urdf_path);
  if (!file.is_open()) {
      std::cerr << "Error: Unable to open file " << urdf_path << std::endl;
      
  }

  // Use the constructor of std::string to load the file content
  std::string fileContent((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  if (!fileName.isEmpty()) {
      //qDebug() << "Selected file: " << fileName;
      // Do something with the selected file...
  }

  std::string param_name = "/" + param_name_namespace + "/robot_description";

  nh_.setParam(param_name,fileContent);

  //
  urdf::Model urdf_model;

  urdf_model.initString(fileContent);

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");

  }

  urdf_managers_[param_name_namespace].root_segment_name =  GetTreeElementSegment(tree.getRootSegment()->second).getName();

  geometry_msgs::TransformStamped tf_transform = createInitTF("map" , "/" +  param_name_namespace +  "/" + urdf_managers_[param_name_namespace].root_segment_name ); 

  urdf_managers_[param_name_namespace].pose_transforms.clear();
  urdf_managers_[param_name_namespace].pose_transforms.push_back(tf_transform);

  urdf_managers_[param_name_namespace].state_publisher = std::make_shared<GuiRobotStatePublisher>(tree, urdf_model);

  urdf_managers_[param_name_namespace].tf_prefix = param_name_namespace;

  urdf_managers_[param_name_namespace].kdl_tree = tree;
  // Get the names of all joints in the robot model

 std::vector<std::string> joint_names{};
  for (const auto& joint_pair : urdf_model.joints_) {
      const urdf::Joint& joint = *joint_pair.second;
      joint_names.push_back(joint.name);
  }

  urdf_managers_[param_name_namespace].joint_names = joint_names;
  urdf_managers_[param_name_namespace].ready = true;


  setEnabledDisplay(urdf_managers_[param_name_namespace].urdf_display_name,false);
  setEnabledDisplay(urdf_managers_[param_name_namespace].urdf_display_name,true);

  bool use_tf_static{true};
  urdf_managers_[param_name_namespace].state_publisher->publishFixedTransforms( param_name_namespace, use_tf_static);


  //Check avalibel tfs:

  std::vector<std::string> tf_names = urdf_managers_[param_name_namespace].state_publisher->getTfNames();

  //QComboBox* combo_box;
  /*ROS_INFO_STREAM("search");
  //findWidgetByName<QComboBox*>("combo_box_tf", urdf_managers_[param_name_namespace].qt_control_layout, combo_box);
  QWidget* widget = findWidgetByName1("combo_box_tf", urdf_managers_[param_name_namespace].qt_control_layout);
  ROS_INFO_STREAM("found");

  QComboBox* combo_box = widget->findChild<QComboBox*>("combo_box_tf");
  ROS_INFO_STREAM("box");*/


  QComboBox* combo_box = urdf_managers_[param_name_namespace].tf_combo_box;
  combo_box->clear();
  //ROS_INFO_STREAM("box2");
  for(std::string name : tf_names)
  {
    QString param_name_q = QString::fromStdString(name);
    combo_box->addItem(param_name_q);
  }
   // 
}

void TeleopPanel::onComboBoxIndexChangedBase(int index) {

  QString selectedItemText = urdf_managers_["assembly_urdf_model"].tf_combo_box->currentText();
  base_tf_name_ =  selectedItemText.toStdString();

  //change marker base
  InteractiveMarker int_marker;
  interactive_marker_server_->get("new_component_pose",int_marker);
  int_marker.header.frame_id = "/assembly_urdf_model/" + base_tf_name_;
  //interactive_marker_server_->insert(int_marker);
  interactive_marker_server_->applyChanges();


  updateComponentsTFs();
}



void TeleopPanel::onComboBoxIndexChangedComponent(int index) {

  setEnabledDisplay("MoveComponent",true);

  QString selectedItemText = urdf_managers_["component_urdf_model"].tf_combo_box->currentText();
  component_tf_name_ = selectedItemText.toStdString();
  updateComponentsTFs();

}

geometry_msgs::TransformStamped TeleopPanel::createInitTF(std::string parrent, std::string child ) 
{
  geometry_msgs::TransformStamped tf_transform; 
  tf_transform.transform.rotation.w = 1.0;
  tf_transform.header.frame_id = parrent;//prefix_frame(tf_prefix, seg->second.root);
  tf_transform.child_frame_id = child;//prefix_frame(tf_prefix, seg->second.tip);

  return tf_transform;

}

bool  TeleopPanel::updateComponentsTFs()
{

  urdf_managers_["component_urdf_model"].pose_transforms.clear() ;

  geometry_msgs::TransformStamped tf_transform = createInitTF("/assembly_urdf_model/" + base_tf_name_, "intermidiate" );
  ROS_INFO_STREAM("base: " << base_tf_name_);

  tf_transform.transform =  marker_tf_transform_.transform;

  geometry_msgs::TransformStamped tf_transform_intermidiate = createInitTF("intermidiate",  "/component_urdf_model/" + urdf_managers_["component_urdf_model"].root_segment_name); // component_tf_name_ 


  KDL::Chain kdlChain = KDL::Chain();
  ROS_INFO_STREAM("compnent: " << component_tf_name_);
  urdf_managers_["component_urdf_model"].kdl_tree.getChain(urdf_managers_["component_urdf_model"].root_segment_name,component_tf_name_,kdlChain);
  // Joint Angles
  //
  int number_of_joints = kdlChain.getNrOfJoints();
   ROS_INFO_STREAM("joinst: " <<number_of_joints);
  KDL::JntArray jointAngles = KDL::JntArray(number_of_joints);
  for(int i = 0; i<number_of_joints; i++)
  {
    jointAngles(0) = 0.0;
  }

  //
  // Perform Forward Kinematics
  //

  KDL::ChainFkSolverPos_recursive FKSolver = KDL::ChainFkSolverPos_recursive(kdlChain);
  KDL::Frame eeFrame;
  FKSolver.JntToCart(jointAngles, eeFrame);


  geometry_msgs::TransformStamped tf_cal = tf2::kdlToTransform(eeFrame.Inverse());
  /*ROS_INFO_STREAM("compnent: " << eeFrame.p[0]);
   ROS_INFO_STREAM("compnent: " << eeFrame.p[1]);
   ROS_INFO_STREAM("compnent: " << eeFrame.p[2]);

    ROS_INFO_STREAM("compnent: " << tf_cal.transform.translation.x);*/

  tf_transform_intermidiate.transform = tf_cal.transform ;
  

  urdf_managers_["component_urdf_model"].pose_transforms.push_back(tf_transform) ;
  urdf_managers_["component_urdf_model"].pose_transforms.push_back(tf_transform_intermidiate) ;
  return true;
}

bool TeleopPanel::setEnabledDisplay(std::string name, bool enabled)
{
     // Access all displays
    //QList<rviz::Display*> displays = vis_manager_->getDisplayFactory()->getDisplays();

    rviz::DisplayGroup * 	 displays =vis_manager_->getRootDisplayGroup();
    int num_of_display = displays->numDisplays();
    for(int i=0;i<num_of_display;i++)
    { 
      std::string panel_name = displays->getDisplayAt(i)->getName().toStdString();
      if(panel_name == name)
      {
        displays->getDisplayAt(i)->setEnabled(enabled);
        break;
      }

      //ROS_INFO_STREAM("dis: " << panel_name);
    }
    
    return true;
    //vis_manager_->getDisplayWrapper("RobotModelBase");//->getDisplay()->disable();
    //vis_manager_->getDisplayWrapper("RobotModelBase");//->getDisplay()->enable();


}



void TeleopPanel::sendTFs()
{

  for(auto map_pair : urdf_managers_)
  {
    //key, 
    UrdfManager urdf_manager = map_pair.second;
    if(urdf_manager.ready)
    {
      std::map<std::string, double> joint_positions;
      for (std::string joint_name : urdf_manager.joint_names) {
        joint_positions.insert(std::make_pair(joint_name,0));
      }

      urdf_manager.state_publisher->publishTransforms(joint_positions, ros::Time::now(), urdf_manager.tf_prefix); 

      std::vector<geometry_msgs::TransformStamped> out_tf = std::vector<geometry_msgs::TransformStamped>{};

      for(geometry_msgs::TransformStamped tf_transform : urdf_manager.pose_transforms)//= tf2::kdlToTransform(seg->second.segment.pose(jnt->second));
      {
        tf_transform.header.stamp =  ros::Time::now();
        out_tf.push_back(tf_transform);
      }
      

      //tf_transforms.push_back(tf_transform);
      tf_broadcaster_.sendTransform(out_tf);

    }

    frameCallback();

    //ROS_INFO_STREAM("TREST" << urdf_manager.qt_control_layout->count());
  }

}

void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  /*config.mapSetValue( "Topic", output_topic_ );*/
}

// Load all configuration data for this panel from the given Config object.
void TeleopPanel::load( const rviz::Config& config )
{
 rviz::Panel::load( config );
  /*QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    //output_topic_editor_->setText( topic );
    //updateTopic();
  }*/
}

QWidget* TeleopPanel::findWidgetByName1(const std::string& widgetName_str, QLayout* layout) {
  QString widgetName = QString::fromStdString(widgetName_str);
  
  for (int i = 0; i < layout->count(); ++i) {
      QLayoutItem* item = layout->itemAt(i);
      if (item->widget() && item->widget()->objectName() == widgetName) {
          return item->widget();
      } else if (item->layout()) {
          QWidget* widget = findWidgetByName1(widgetName_str, item->layout());
          if (widget) {
              return widget;
          }
      }
  }
  return nullptr;
}


template<typename widget_type>
bool TeleopPanel::findWidgetByName(const std::string& widgetName_str, QLayout* layout, widget_type widget) 
{
  
  QString widgetName = QString::fromStdString(widgetName_str);
  
  for (int i = 0; i < layout->count(); ++i) {
      ROS_INFO_STREAM("Start" << i);
      QLayoutItem* item = layout->itemAt(i);
      if (item->widget() && item->widget()->objectName() == widgetName) {
          ROS_INFO("hierh");
          widget = layout->widget()->findChild<widget_type>(widgetName);
          return true;
      } else if (item->layout()) {
          ROS_INFO_STREAM("newcall");
          bool widget_present = findWidgetByName<widget_type>(widgetName_str, item->layout(), widget);
          if (widget_present) {
            return widget_present;
          }
      }
  }

  return false;
}





} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_urdf_composer::TeleopPanel,rviz::Panel )
// END_TUTORIAL
