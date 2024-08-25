# rviz_plugin_urdf_composer
ROS package with rviz plugin for asembling multiple xacro-urdf files to single xacro-urdf file.

 ## Run rviz config
 
 rviz -d test2.rviz


xacro robot_arm.urdf_xacro > tmp.urdf


 rosservice call /manage_modules "package_name:reconcycle_module_camera_desk urdf_name:reconcycle_module_camera_desk.urdf.xacro module_name:camera_module module_plug_id:1 parrent_module_name:module_cnc parrent_module_plug_id:1 operation: 1"


 rosservice call /manage_modules reconcycle_module_camera_desk reconcycle_module_camera_desk.urdf.xacro camera_module 1 module_cnc 1 1

  rosservice call /manage_modules reconcycle_module_camera_desk reconcycle_module_camera_desk.urdf.xacro camera_module2 2 module_cnc 2 1

 rosservice call /manage_modules reconcycle_module_camera_desk reconcycle_module_camera_desk.urdf.xacro camera_module 1 module_cnc 1 2

 rosrun rviz_plugin_urdf_composer reconcycle_module_selection

rosservice call /manage_modules reconcycle_module_robot_desk reconcycle_module_robot_desk.urdf.xacro module_robot1 1 module_cnc 1 1


module_robot1:
    er: 0.0
    ep: -1.5707963267948886
    ey: -3.2029886157149005e-14
    package_name: reconcycle_module_robot_desk
    parent: module_cnc_pnpf2
    urdf_name: reconcycle_module_robot_desk.urdf.xacro
    x: -0.06699999999999115
    y: -0.13499999999999734
    z: -0.6000000000000014


ett
