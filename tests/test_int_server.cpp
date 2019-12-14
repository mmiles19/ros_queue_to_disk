#include <ros/ros.h>
// #include <ros_queue_to_disk.hpp>
#include "ros_queue_to_disk.hpp"
#include <ros_queue_to_disk/TestInt.h>

std::vector<int> ints;

bool ReceiveInt(ros_queue_to_disk::TestInt::Request  &req,
         ros_queue_to_disk::TestInt::Response &res)
{
  printf("ReceiveInt got %u - ",req.msg);

  if(req.msg>=20 && req.msg<=30) // only consider returning false ack if between 20 and 30
  {
    if(std::find(ints.begin(), ints.end(), req.msg) != ints.end()) // check if this msg int value has already been received
    {
      printf("returning true\n");
      return true;
    }
    else // if not, add to list and return false ack
    {
      ints.push_back(req.msg);
      printf("returning false\n");
      return false;
    }
  }
  else
  {
    printf("returning true\n");
    return true;
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_int_server");

  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("test_int", ReceiveInt);
  ros::spin();

  return 0;
}