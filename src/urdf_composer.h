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
#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <QComboBox>

#include <QVBoxLayout>

//#include "robot_state_publisher/robot_state_publisher.h"
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>



#include <QPainter>
#include <QLineEdit>

#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <QString>
#include <QFileDialog>
#include <QPushButton>

#include <QSpinBox>

#include <interactive_markers/interactive_marker_server.h>


#include <tf2/LinearMath/Vector3.h>

#include "tools.hpp"

namespace rviz_urdf_composer
{



struct UrdfManager
{

  QVBoxLayout* qt_control_layout;
  QComboBox* tf_combo_box;
  QPushButton* load_urdf_button;
  QLabel* id_name;

  std::shared_ptr<GuiRobotStatePublisher> state_publisher;
  std::string tf_prefix;

  bool ready{false};

  std::string urdf_display_name;

  std::string urdf_file_name;
  std::string urdf_package;

  std::vector<std::string> joint_names;
  std::vector<geometry_msgs::TransformStamped> pose_transforms;
  std::string root_segment_name;

  KDL::Tree kdl_tree;

};


class UrdfComposer: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:

  
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  UrdfComposer( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the


  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  void selectUrdfFile(std::string param_name_namespace);
  void loadURDFtoParam(std::string urdf_path, std::string param_name_namespace, std::string composition_yaml = "None");
  void loadCompositionURDFconfig();

  void initEmptyUrdf();
  void updateAssemblyUrdf(std::string path, std::string yaml_path = "None");
  void updateAssemblyComponentList();
  void addComponentToUrdf();
  void saveGeneratedUrdf();
  void deleteSelectedElement();

  bool setEnabledDisplay(std::string name, bool enabled);
  void onComboBoxIndexChangedBase(int index);
  void onComboBoxIndexChangedComponent(int index); 
  void onComboBoxIndexChangedComponentsList(int index); 
  void changedSpinBox(std::string parameter_name, double value);

  geometry_msgs::TransformStamped createInitTF(std::string parrent, std::string child);

  bool updateComponentsTFs();

  QWidget* findWidgetByName1(const std::string& widgetName_str, QLayout* layout) ;

  template<typename widget_type>
  bool findWidgetByName(const std::string& widgetName_str, QLayout* layout, widget_type widget) ;

  visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg );
  visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg );
  void frameCallback();
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf2::Vector3& position, bool show_6dof );
  double calculateAbsoluteAngle(KDL::Rotation rotation_matrix,  int axis_index);
  KDL::Vector getUnitVector(KDL::Rotation rotation_matrix,  int axis_index);

  QHBoxLayout* createInteractiveSpinBox(std::string name, std::string units, double min, double max );


  // Next come a couple of public Qt slots.
public Q_SLOTS:

  void openFileDialog();


  // Here we declare some internal slots.
protected Q_SLOTS:
  // sendvel() publishes the current velocity values to a ROS
  // topic.  Internally this is connected to a timer which calls it 10
  // times per second.
  void sendTFs();


  // Then we finish up with protected member variables.
protected:



  // The ROS node handle.
  ros::NodeHandle nh_;

  QString file_loading_path_{""};

  QComboBox*  composition_components_combo_box_;
  std::string chosen_component_{"None"};
  
  std::string urdf_path_assembly_;
  std::string urdf_path_new_component_;

  std::string assembly_absolute_base_tf_name_{"map"};
  std::string component_absolute_base_tf_name_{"map"};

  std::string base_tf_name_;
  std::string component_tf_name_;

  QLineEdit *namespace_lineEdit_ ;

  std::string assembly_urdf_namespace_{"/assembly_urdf_namespace"};
  std::string workspace_path_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;

  std::map<std::string, int> cart_to_ind_map_;


  std::map<std::string, QDoubleSpinBox*> pose_control_layout_boxes_;

  std::map<std::string, UrdfManager> urdf_managers_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  geometry_msgs::TransformStamped marker_tf_transform_;


};


} // end namespace rviz_plugin_tutorials

#endif // TELEOP_PANEL_H






