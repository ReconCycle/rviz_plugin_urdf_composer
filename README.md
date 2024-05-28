# rviz_plugin_urdf_composer
ROS package with rviz plugin for asembling multiple xacro-urdf files to single xacro-urdf file.

 ## Run rviz config
 
 rviz -d test2.rviz


xacro robot_arm.urdf_xacro > tmp.urdf


 rosservice call /manage_modules "package_name:reconcycle_module_camera_desk urdf_name:reconcycle_module_camera_desk.urdf.xacro module_name:camera_module module_plug_id:1 parrent_module_name:module_cnc parrent_module_plug_id:1 operation: 1"


 rosservice call /manage_modules reconcycle_module_camera_desk reconcycle_module_camera_desk.urdf.xacro camera_module 1 module_cnc 1 1

  rosservice call /manage_modules reconcycle_module_camera_desk reconcycle_module_camera_desk.urdf.xacro camera_module2 2 module_cnc 2 1

 rosservice call /manage_modules reconcycle_module_camera_desk reconcycle_module_camera_desk.urdf.xacro camera_module 1 module_cnc 1 2