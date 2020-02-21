#include <ros/ros.h>
#include <ros_queue_to_disk/ros_queue_to_disk.hpp>
// #include "ros_queue_to_disk.hpp"
#include <ros_queue_to_disk/TestInt.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_int_client");
  
  ros::NodeHandle nh;
  ros_queue_to_disk::ServiceClient<ros_queue_to_disk::TestInt> client(&nh, "test_int");
  ros_queue_to_disk::TestInt srv;
  for(uint i=0; i<100; i++)
  {
    srv.request.msg = i;
    if (client.call(srv))
    {
      ROS_INFO("Success %u",i);
    }
    else
    {
      ROS_ERROR("Failed %u",i);
    }
  }

  return 0;
}