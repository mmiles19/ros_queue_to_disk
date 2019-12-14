#include <ros/ros.h>
// #include <ros_queue_to_disk.hpp>
#include "ros_queue_to_disk.hpp"
#include <ros_queue_to_disk/TestImage.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_image_client");
  
  ros::NodeHandle nh;
  ros_queue_to_disk::ServiceClient<ros_queue_to_disk::TestImage> client(&nh, "test_image");
  ros_queue_to_disk::TestImage srv;

  std::string filedir = "/home/mike/Documents/mnist/"; 
  
  for(uint i=0; i<10; i++)
  {
    std::string filename = "mnist_" + std::to_string(i) + ".png";
    std::string filepath = filedir + filename;
    cv::Mat img = cv::imread( filepath, cv::IMREAD_COLOR );
#ifdef DEBUG_TEST_IMAGE
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", img );
    cv::waitKey(0); 
#endif

    std_msgs::Header header; 
    header.seq = i; 
    header.stamp = ros::Time::now(); 
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    sensor_msgs::Image img_msg; 
    img_bridge.toImageMsg(img_msg);

    ros_queue_to_disk::TestImage srv;
    srv.request.test_img = img_msg;
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