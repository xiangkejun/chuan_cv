#ifndef _YOLO_TRACKER_H
#define _YOLO_TRACKER_H
//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
//ros
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
//control_flag msg
#include <xx_msgs/Flag.h>

#define MIN_DIST        0.05
#define MAX_LINEAR_VEL  0.2
#define MAX_ANGULAR_VEL 0.5
#define MAX_TRY_YOLO    100  //失败后连续尝试yolo最大次数

bool gainControl_flag = false;//控制权标志位
bool yoloFindTarget = false;  //yolo状态标志位
double lineSpeed = 0;         //线速度
double angularVelocity = 0;   //角速度
int tryYoloCount = 0;         //yolo尝试次数计数
ros::Publisher vel_pub;       //速度消息发布
ros::Publisher ctrl_pub;      //控制权限消息发布
ros::Publisher tuolian_pub;      //拖链消息发布


void gainControlCB(const xx_msgs::Flag::ConstPtr& msg);
void calcSpeed(cv::Rect &bBox,cv::Point &referencePoint,double &lineSpeed,double &angularVelocity);
void imageCB(const sensor_msgs::ImageConstPtr& msg);
void yoloBboxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg);

#endif
