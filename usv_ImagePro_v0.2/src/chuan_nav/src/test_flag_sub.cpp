#include "ros/ros.h"

//xx
#include <xx_msgs/Flag.h>

#include <iostream>
using namespace std;

ros::Subscriber recv_flag_sub;
ros::Subscriber recv_flag_sub1;
std::string flag_cv;
int flag_nav;   //导航标志
void recv_flag_callback(const xx_msgs::Flag::ConstPtr& msg)
{
	flag_cv = msg->flag;
	std::cout<<flag_cv<<std::endl;
	if(flag_cv == "nav stop,cv start")
	{
		flag_nav =0;
		cout<<"nav did stop"<<endl;
	}

	if(flag_cv == "nav start,cv stop")
	{
		flag_nav =1;
		cout<<"nav do start"<<endl;
	}	
}
void recv_flag_callback1(const xx_msgs::Flag::ConstPtr& msg)
{
	flag_cv = msg->flag;
	std::cout<<flag_cv<<std::endl;
	if(flag_cv == "nav stop,cv start")
	{
		flag_nav =0;
		cout<<"nav did stop1111"<<endl;
	}

	if(flag_cv == "nav start,cv stop")
	{
		flag_nav =1;
		cout<<"nav do start1111"<<endl;
	}	
}
int main(int argc,char** argv)
{
	ros::init(argc, argv, "test_flag_sub");
    ros::NodeHandle n;	

	
	recv_flag_sub = n.subscribe<xx_msgs::Flag>("flag_nav_to_cv",1,recv_flag_callback);
	recv_flag_sub1 = n.subscribe<xx_msgs::Flag>("flag_cv_to_nav",1,recv_flag_callback1);

	ros::spin();	
    return 0;
}