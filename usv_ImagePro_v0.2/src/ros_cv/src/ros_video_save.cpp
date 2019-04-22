// xx
// 订阅camera/image后，间隔一秒的时间保存为图片的形式。

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

bool saveCloud(false);

using std::cout;
using std::endl;
using std::stringstream;
using std::string;
unsigned int fileNum = 1;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);

    if(true)
    {
        stringstream stream;
        stringstream stream1;
        stream <<"Goal RgbImage" << fileNum<<".jpg";
        stream1 <<"/home/xx/Pictures/pic/" << fileNum <<".jpg";
        string filename = stream.str();
        string filename1 = stream1.str();
        cv::imwrite(filename1,cv_bridge::toCvShare(msg)->image);
        saveCloud = false;
        fileNum++;
        sleep(1);
        cout << filename << " had Saved."<< endl;
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}