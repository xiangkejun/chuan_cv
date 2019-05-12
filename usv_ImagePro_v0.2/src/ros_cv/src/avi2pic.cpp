#include <iostream>
#include "cv.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

// 描述：将视频帧转成图片输出
int main(int argc, char** argv)
{
	// 获取视频文件
	VideoCapture cap("/home/xx/video_xx/1.avi");

	// 获取视频总帧数
	long totalFrameNumber = cap.get(CV_CAP_PROP_FRAME_COUNT);
	cout << "total frames: " << totalFrameNumber << endl;

	Mat frame;
	bool flags = true;
	long currentFrame = 0;

	while (flags){
		// 读取视频每一帧
		cap.read(frame);

		stringstream str;
		str  << currentFrame/2 + 1<< ".jpg";
		cout << "正在处理第" << currentFrame/2 << "帧" << endl;
		printf("\n");

		// 设置每30帧获取一次帧
		if (currentFrame % 2 == 0){
			// 将帧转成图片输出
			imwrite("/home/xx/video_xx/0000" + str.str(), frame);
		}
		// 结束条件
		if (currentFrame >= totalFrameNumber){
			flags = false;
		}
		currentFrame++;
	}
	
	system("pause");
}
