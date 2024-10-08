

#include <ros/ros.h>


#include <interactive_markers/menu_handler.h>

#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"


#include <geometry_msgs/Quaternion.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//#include "tf2_ros/transform_broadcaster.h"

#include <math.h>

#include "urdf_composer.h"

using namespace visualization_msgs;



namespace rviz_urdf_composer
{

// %Tag(Box)%
Marker UrdfComposer::makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& UrdfComposer::makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(frameCallback)%
void UrdfComposer::frameCallback()
{


  ros::Time time = ros::Time::now();

  tf2::Stamped<tf2::Transform> transform;
  transform.stamp_ = time;
  transform.frame_id_ = component_absolute_base_tf_name_; //"map";

  transform.setOrigin(tf2::Vector3(0.0, 0.0, sin(0/ 140.0) * 2.0));
  transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  geometry_msgs::TransformStamped transform_msg;
  transform_msg = tf2::toMsg(transform);

  transform_msg.child_frame_id = "marker_base_frame";
  tf_broadcaster_.sendTransform(transform_msg);

  /*transform_msg.child_frame_id = "moving_frame";
  tf_broadcaster_.sendTransform(transform_msg);

  transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0 / 140.0, 0.0);
  transform.setRotation(quat);
  transform_msg = tf2::toMsg(transform);
  transform_msg.child_frame_id = "rotating_frame";
  tf_broadcaster_.sendTransform(transform_msg);*/


}
// %EndTag(frameCallback)%

// %Tag(processFeedback)%
void UrdfComposer::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)// )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  KDL::Frame kdl_frame;
  double roll, pitch, yaw;

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      marker_tf_transform_ = geometry_msgs::TransformStamped();

      // Set translation from Pose
      marker_tf_transform_.transform.translation.x = feedback->pose.position.x;
      marker_tf_transform_.transform.translation.y = feedback->pose.position.y;
      marker_tf_transform_.transform.translation.z = feedback->pose.position.z;

      marker_tf_transform_.transform.rotation.x = feedback->pose.orientation.x;
      marker_tf_transform_.transform.rotation.y = feedback->pose.orientation.y;
      marker_tf_transform_.transform.rotation.z = feedback->pose.orientation.z;
      marker_tf_transform_.transform.rotation.w = feedback->pose.orientation.w;
 
      kdl_frame = tf2::transformToKDL(marker_tf_transform_);
      
      kdl_frame.M.GetRPY(roll, pitch, yaw);


      pose_control_layout_boxes_["X translation:"]->setValue(marker_tf_transform_.transform.translation.x);
      pose_control_layout_boxes_["Y translation:"]->setValue(marker_tf_transform_.transform.translation.y);
      pose_control_layout_boxes_["Z translation:"]->setValue(marker_tf_transform_.transform.translation.z);

      pose_control_layout_boxes_["RX rotation:"]->setValue(roll*180/M_PI);
      pose_control_layout_boxes_["RY rotation:"]->setValue(pitch*180/M_PI);
      pose_control_layout_boxes_["RZ rotation:"]->setValue(yaw*180/M_PI);




      /*pose_control_layout_boxes_["RX rotation:"]->setValue(calculateAbsoluteAngle(kdl_frame.M, 0)*180/M_PI);
      pose_control_layout_boxes_["RY rotation:"]->setValue(calculateAbsoluteAngle(kdl_frame.M, 1)*180/M_PI);
      pose_control_layout_boxes_["RZ rotation:"]->setValue(calculateAbsoluteAngle(kdl_frame.M, 2)*180/M_PI);*/


      //KDL::Rotation lift_matrix = KDL::Rotation::Rot 	( 	const Vector &  	rotvec,double  	angle) ;


      
      //updateComponentsTFs();

      /*ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );*/
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  interactive_marker_server_->applyChanges(); /**/
}

double UrdfComposer::calculateAbsoluteAngle(KDL::Rotation rotation_matrix,  int axis_index)
{
  ROS_INFO_STREAM( "Index" <<axis_index );
  KDL::Vector direction_vector{0,0,0};
  direction_vector[axis_index] = 1;
  KDL::Vector unitVector = getUnitVector(rotation_matrix,axis_index);
  KDL::Vector rot_vector = unitVector*direction_vector;


  double rotation_angle = std::acos(float(unitVector[axis_index]));
  if(unitVector[axis_index]>1)
  {
    rotation_angle = 0;
  }else if(unitVector[axis_index]<-1)
  {
    rotation_angle = M_PI;
  }
  ROS_INFO_STREAM( "Angle" <<rotation_angle );


  
  //KDL::Rotation lift_matrix = KDL::Rotation::Rot(rot_vector, rotation_angle) ;
  KDL::Rotation rotated_matrix = KDL::Rotation::Rot(rot_vector, rotation_angle) *rotation_matrix;
  ROS_INFO_STREAM( "unit size" << getUnitVector(rotated_matrix,axis_index)[axis_index]);
  KDL::Rotation  rotated_matrix2  = KDL::Rotation::Rot(rot_vector, -rotation_angle) *rotation_matrix;
  ROS_INFO_STREAM( "unit size" << getUnitVector(rotated_matrix2,axis_index)[axis_index]);

  int next_index = axis_index +1 ;
  if(next_index>2)
  {
    next_index= 0;
  }
  ROS_INFO_STREAM( "second size" << getUnitVector(rotated_matrix,next_index)[next_index]);
  double angle = std::acos(getUnitVector(rotated_matrix,next_index)[next_index]);

  if(getUnitVector(rotated_matrix,next_index)[next_index]>1)
  {
    angle = 0;
  }else if(getUnitVector(rotated_matrix,next_index)[next_index]<-1)
  {
    angle = M_PI;
  }
  ROS_INFO_STREAM( "calculated angle: " << angle);
  return angle;
}

KDL::Vector UrdfComposer::getUnitVector(KDL::Rotation rotation_matrix,  int axis_index)
{ 

  if(axis_index == 0)
  {
    return rotation_matrix.UnitX();
  }else if(axis_index == 1)
  {
    return rotation_matrix.UnitY();
  }else if(axis_index == 2)
  {
    return rotation_matrix.UnitZ();
  }

  return KDL::Vector();
}
// %EndTag(processFeedback)%

// %Tag(alignMarker)%
void UrdfComposer::alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  /*geometry_msgs::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x-0.5)+0.5;
  pose.position.y = round(pose.position.y-0.5)+0.5;

  ROS_INFO_STREAM( feedback->marker_name << ":"
      << " aligning position = "
      << feedback->pose.position.x
      << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z
      << " to "
      << pose.position.x
      << ", " << pose.position.y
      << ", " << pose.position.z );

  interactive_marker_server_->setPose( feedback->marker_name, pose );
  interactive_marker_server_->applyChanges();*/
}
// %EndTag(alignMarker)%

////////////////////////////////////////////////////////////////////////////////////

// %Tag(6DOF)%
void UrdfComposer::make6DofMarker( bool fixed, unsigned int interaction_mode, const tf2::Vector3& position, bool show_6dof )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "marker_base_frame";// "map";
  int_marker.pose.position.x = position[0];
  int_marker.pose.position.y = position[1];
  int_marker.pose.position.z = position[2];
  int_marker.scale = 1;

  int_marker.name = "new_component_pose";
  int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = "MOVE COMPONENT";
  }

  if(show_6dof)
  {
    tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.orientation.x = 0.0;

    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  interactive_marker_server_->insert(int_marker);
  interactive_marker_server_->setCallback(int_marker.name, std::bind(&UrdfComposer::processFeedback, this, std::placeholders::_1));//

  //interactive_marker_server_->setCallback(int_marker.name, &UrdfComposer::processFeedback);

  
  //if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
   // menu_handler.apply( *interactive_marker_server_, int_marker.name );

}
// %EndTag(6DOF)%

}