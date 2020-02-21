#include <ros/ros.h>
#include <ros_queue_to_disk/ros_queue_to_disk.hpp>
// #include "ros_queue_to_disk.hpp"
#include <ros_queue_to_disk/TestImage.h>

std::vector<int> store;

bool Receive(ros_queue_to_disk::TestImage::Request &req,
         ros_queue_to_disk::TestImage::Response &res)
{
  printf("Receive got %u - ",req.test_img.header.seq);

  if(req.test_img.header.seq>=2 && req.test_img.header.seq<=3) // only consider returning false ack if between 20 and 30
  {
    if(std::find(store.begin(), store.end(), req.test_img.header.seq) != store.end()) // check if this msg int value has already been received
    {
      printf("returning true\n");
      return true;
    }
    else // if not, add to list and return false ack
    {
      store.push_back(req.test_img.header.seq);
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
  ros::init(argc, argv, "test_image_server");

  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("test_image", Receive);
  ros::spin();
}