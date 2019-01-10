#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include "yolo_tracker.h"

using namespace std;
using namespace cv;

//Init Rect
Rect initRect(0,0,0,0);
Point referencePoint(320,480);//参考点
Rect yoloBbox(0,0,0,0);

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "yolo_tracker");
    ros::NodeHandle n1;
	image_transport::ImageTransport it(n1);
    image_transport::Subscriber img_sub = it.subscribe("/camera/image", 10, imageCB);//订阅图像消息
	ros::Subscriber Bbox_sub = n1.subscribe("/darknet_ros/bounding_boxes", 10, yoloBboxCB);//订阅boundingbox消息
	ros::Subscriber controlFlag_sub = n1.subscribe<xx_msgs::Flag>("flag_nav_to_cv",10,gainControlCB);//订阅控制权限标志
	vel_pub = n1.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",10);//发布速度消息
	ctrl_pub=n1.advertise<xx_msgs::Flag>("flag_cv_to_nav",1);

    ROS_INFO("waiting control...");
	ros::Rate rate(30.0);
	while (ros::ok())
    {
        if(cv::waitKey(10)==27)
         {
            std::cout<<"ESC!"<<std::endl;
            return -1;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

//yolo bbox消息回调函数
void yoloBboxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg)
{ 
  if(gainControl_flag)
  {
	ROS_INFO("yoloBboxCB...");
	Point bBoxCenetr1 = cv::Point(0,0);
	bool init = true;
	for(size_t i = 0; i < box_msg->bounding_boxes.size();i++)
    {
		string className = box_msg->bounding_boxes[i].Class;
		double probability = box_msg->bounding_boxes[i].probability;
		if(className=="leaf"||className=="cup"||className=="person")//目标类别
		{ 
			int xmin = box_msg->bounding_boxes[i].xmin;
			int ymin = box_msg->bounding_boxes[i].ymin;
			int xmax = box_msg->bounding_boxes[i].xmax;
			int ymax = box_msg->bounding_boxes[i].ymax;
			if(init)
			  {
				init = false;
				yoloBbox = cv::Rect(xmin,ymin,xmax-xmin,ymax-ymin);
				bBoxCenetr1 = cv::Point((xmax+xmin)/2,ymax);
			  }
			else
			{
				Point bBoxCenetr2((xmax+xmin)/2,ymax);
				int dist1 = abs(bBoxCenetr1.x-referencePoint.x)+abs(bBoxCenetr1.y-referencePoint.y);//街区距离
           	 	int dist2 = abs(bBoxCenetr2.x-referencePoint.x)+abs(bBoxCenetr2.y-referencePoint.y);//街区距离
				if(dist1 > dist2)
				{
					yoloBbox = Rect(xmin,ymin,xmax-xmin,ymax-ymin);
					bBoxCenetr1 = Point((xmax+xmin)/2,ymax);
				}
			}
			//cout<<"bBoxCenetr1: "<<bBoxCenetr1<<endl;
		}
    }
    if(yoloBbox.height>0&&yoloBbox.width>0)
    {
        yoloFindTarget=true;
    }
    else
    {
       yoloFindTarget=false; 
    }
  }
}

void imageCB(const sensor_msgs::ImageConstPtr& msg)
{	
    cv::Mat frame = cv_bridge::toCvShare(msg,"bgr8")->image;
	if(gainControl_flag)
	{	
		if(yoloFindTarget)
		{
			calcSpeed(yoloBbox,referencePoint,lineSpeed,angularVelocity);
			ROS_INFO("lineSpeed: %f  angularVelocity: %f",lineSpeed,angularVelocity);
			cv::line(frame,referencePoint,cv::Point(yoloBbox.x+yoloBbox.width/2,yoloBbox.y+yoloBbox.height),cv::Scalar(255,0,0),2); 			
			
			cv::rectangle(frame, cv::Point(yoloBbox.x, yoloBbox.y),
				 	  	cv::Point(yoloBbox.x+yoloBbox.width, 
					  	yoloBbox.y+yoloBbox.height), 
					  	cv::Scalar(0, 0, 255));
            yoloBbox = Rect(0,0,0,0);//yolobbox使用完之后归零
            yoloFindTarget = false;
            tryYoloCount=0;
		}
		//速度平滑
		lineSpeed=lineSpeed > 0.02?(lineSpeed-0.01):.0;
		angularVelocity=angularVelocity>0.05?angularVelocity-0.05:angularVelocity<(-0.05)?angularVelocity+0.05:.0;
		//发布速度信息
		geometry_msgs::Twist vel_msg;
    	vel_msg.linear.x = lineSpeed;
    	vel_msg.angular.z =angularVelocity;
    	vel_pub.publish(vel_msg);

		tryYoloCount++;
		if(tryYoloCount>=MAX_TRY_YOLO)
		{
			//交控制权
			ROS_INFO("ImagePro lost control");
            gainControl_flag = false;
			tryYoloCount = 0;
            xx_msgs::Flag flag_cv_to_nav;
	        flag_cv_to_nav.flag = "nav start,cv stop";
	        ctrl_pub.publish(flag_cv_to_nav);   //发布图像控制标志
		}
	}
    cv::imshow("tracker frame",frame);
    cv::waitKey(1);
}

//控制权消息回调函数
void  gainControlCB(const xx_msgs::Flag::ConstPtr& msg)
{
   std::string control_msg = msg->flag;
   std::cout<<"control_msg: "<<control_msg<<std::endl;
   if(control_msg == "nav stop,cv start")
   {
      ROS_INFO("ImagePro gain control.");
      gainControl_flag = true;
   }
   else if(control_msg == "nav start,cv stop")
   {
      ROS_INFO("ImagePro lost control.");
      gainControl_flag = false;
   }
   else
   {
      ROS_INFO("ImagePro lost control.");
      gainControl_flag = false;
   }
}

//根据bbox计算线速度和角速度
void calcSpeed(cv::Rect &bBox,cv::Point &referencePoint,double &lineSpeed,double &angularVelocity)
{
    cv::Point bBoxCenetr = cv::Point(bBox.x+bBox.width/2,bBox.y+bBox.height/2);
    float offset_x = referencePoint.x - bBoxCenetr.x;
    float offset_y = referencePoint.y - bBoxCenetr.y;
    float dist_to_ref = 0.001*sqrt(pow(offset_x,2)+pow(offset_y,2)); 
    float angle_to_ref = 0.65*atan(offset_x/offset_y); 
    if(offset_y > 0 && dist_to_ref > MIN_DIST)//bounding box在参考点前方
    {
        lineSpeed = MAX_LINEAR_VEL < dist_to_ref ? MAX_LINEAR_VEL : dist_to_ref;//0~0.2
        angularVelocity= -MAX_ANGULAR_VEL > angle_to_ref ? -MAX_ANGULAR_VEL : MAX_ANGULAR_VEL < angle_to_ref ? MAX_ANGULAR_VEL : angle_to_ref;//-0.5~0.5 
    }
    else //停止
    {
        lineSpeed=0;
        angularVelocity=0;
    }
}
