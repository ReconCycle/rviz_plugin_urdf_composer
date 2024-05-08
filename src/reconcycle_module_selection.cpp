#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "tools.hpp"

bool add(std_srvs::Trigger::Request  &req,
         std_srvs::Trigger::Response &res)
{
  //res.sum = req.a + req.b;
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}


class ModuleSelector
{

    public:
        ModuleSelector(): nh_{}
        {
            service_ =  nh_.advertiseService("add_two_ints", add);
            nh_.setParam("robot_description", "my_string");
        }

    private:
        ros::NodeHandle nh_;
        ros::ServiceServer service_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_server");
  
    ModuleSelector node;
    ROS_INFO("Ready to add two ints.");
    ros::spin();

    return 0;
}