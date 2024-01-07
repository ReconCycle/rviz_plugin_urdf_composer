

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

#include "teleop_panel.h"

using namespace visualization_msgs;



namespace rviz_urdf_composer
{

// %Tag(Box)%
Marker TeleopPanel::makeBox( InteractiveMarker &msg )
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

InteractiveMarkerControl& TeleopPanel::makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(frameCallback)%
void TeleopPanel::frameCallback()
{




  ros::Time time = ros::Time::now();

  tf2::Stamped<tf2::Transform> transform;
  transform.stamp_ = time;
  transform.frame_id_ = "map";
  transform.setOrigin(tf2::Vector3(0.0, 0.0, sin(0/ 140.0) * 2.0));
  transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  geometry_msgs::TransformStamped transform_msg;
  transform_msg = tf2::toMsg(transform);
  transform_msg.child_frame_id = "moving_frame";
  tf_broadcaster_.sendTransform(transform_msg);



  transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0 / 140.0, 0.0);
  transform.setRotation(quat);
  transform_msg = tf2::toMsg(transform);
  transform_msg.child_frame_id = "rotating_frame";
  tf_broadcaster_.sendTransform(transform_msg);


}
// %EndTag(frameCallback)%

// %Tag(processFeedback)%
void TeleopPanel::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)// )
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

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
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
          << feedback->header.stamp.nsec << " nsec" );
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
// %EndTag(processFeedback)%

// %Tag(alignMarker)%
void TeleopPanel::alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
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
void TeleopPanel::make6DofMarker( bool fixed, unsigned int interaction_mode, const tf2::Vector3& position, bool show_6dof )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.pose.position.x = position[0];
  int_marker.pose.position.y = position[1];
  int_marker.pose.position.z = position[2];
  int_marker.scale = 1;

  int_marker.name = "simple_6dof";
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
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
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
  ROS_INFO_STREAM("z");
  interactive_marker_server_->insert(int_marker);
  interactive_marker_server_->setCallback(int_marker.name, std::bind(&TeleopPanel::processFeedback, this, std::placeholders::_1));//
   ROS_INFO_STREAM("2");
  //interactive_marker_server_->setCallback(int_marker.name, &TeleopPanel::processFeedback);

  
  //if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
   // menu_handler.apply( *interactive_marker_server_, int_marker.name );
}
// %EndTag(6DOF)%

}