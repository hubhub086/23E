#include <iostream>
#include <fstream>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

#include "uart.h"

using namespace std;
using namespace cv;

Mat frame,hsv_frame;
Mat redMask1, redMask2, greenMask1, greenMask2;
vector<Mat> hsvSplit;
int fd;

Scalar redLow1 = {0, 43, 56};
Scalar redHigh1 = {150, 255, 255};
Scalar redLow2 = {0, 43, 56};
Scalar redHigh2 = {150, 255, 255};
Scalar greenLow1 = {0, 43, 56};
Scalar greenHigh1 = {150, 255, 255};
Scalar greenLow2 = {0, 43, 56};
Scalar greenHigh2 = {150, 255, 255};

int main()
{
    cout << "###  23E  ###" << endl;
    VideoCapture capture(1, CAP_V4L2);
	if (!capture.isOpened())
	{
		cout << "camera not open!" << endl;
		exit(1);
	}
    // camera config
    capture.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.set(CAP_PROP_FRAME_WIDTH,640);
    capture.set(CAP_PROP_FRAME_HEIGHT,480);
    capture.set(CAP_PROP_FPS,60);
    capture.set(CAP_PROP_BUFFERSIZE, 1);
    capture.set(CAP_PROP_AUTO_WB, 1);
    // capture.set(CAP_PROP_AUTO_EXPOSURE, 0);
    // capture.set(CAP_PROP_WB_TEMPERATURE, 4000);
    cout << "CAP_PROP_FPS" << capture.get(CAP_PROP_FPS) << endl;
    cout << "CAP_PROP_BUFFERSIZE=" << capture.get(CAP_PROP_BUFFERSIZE) << endl;
    cout << "CAP_PROP_AUTO_WB=" << capture.get(CAP_PROP_AUTO_WB) << endl;
	cout << "CAP_PROP_WB_TEMPERATURE=" << capture.get(CAP_PROP_WB_TEMPERATURE) << endl;
    cout << "CAP_PROP_FRAME_SIZE=(" << capture.get(CAP_PROP_FRAME_WIDTH) << "," << capture.get(CAP_PROP_FRAME_HEIGHT) << ")" << endl;

    //uart init
    fd = uart_init("/dev/ttyTHS1", 115200, 8, 'N', 1);
    uart_receiveThread_touch();
	uart_send(fd, "main init finish\n");

    clock_t start, end;
    while(1)
    {   
		cout << "### 23E  ###" << endl;
        start = clock();
        capture >> frame;
        cvtColor(frame, hsv_frame, COLOR_BGR2HSV); 
		split(hsv_frame, hsvSplit);	
        equalizeHist(hsvSplit[2], hsvSplit[2]); 
        merge(hsvSplit, hsv_frame);	

        inRange(hsv_frame, redLow1, redHigh1, redMask1);//二值化识别颜色
 
        //开操作 (去除一些噪点)
        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(redMask1, redMask1, MORPH_OPEN, element);
    
        //闭操作 (连接一些连通域)
        morphologyEx(redMask1, redMask1, MORPH_CLOSE, element);
        //  Canny(g_grayImage, g_cannyMat_output, 80, 80 * 2, 3);
    
        // 寻找轮廓
        vector<vector<Point>> contours;
	    vector<Vec4i> hierarchy;
        findContours(redMask1, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    
        //假设contours是用findContours函数所得出的边缘点集
        RotatedRect box;
        Point center = {0,0};
        int maxarea, maxAreaIdx;
    
        // if(contours.size()!=0)
        // {
        //     for (int index =0; index< contours.size(); index++)
        //     {
        //         double tmparea = fabs(contourArea(contours[index]));
        //         if (tmparea > maxarea)
        //         {
        //             maxarea = tmparea;
        //             maxAreaIdx = index;//记录最大轮廓的索引号
        //         }
        //     }
    
        //     box = minAreaRect(contours[maxAreaIdx]);
        //     rectangle(frame,box.boundingRect(),Scalar(0,0,255),2);
        //     center = box.center;
        // }

        imshow("frame", frame);
        imshow("frame_hsv", hsv_frame);
        imshow("mask", redMask1);
        end = clock();
        cout << "fps=" << 1/(double(end-start)/CLOCKS_PER_SEC) << endl;
        int key = waitKey(1);
        if (key == 27)
        {
            break;
        }
    }
}