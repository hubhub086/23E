#include <iostream>
#include <fstream>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "tft.h"
#include "uart.h"

using namespace std;
using namespace cv;

Mat frame,hsv_frame;
Mat rotation,frame_warp;
Mat red, blue, green;
Mat red_mask, green_mask;
Mat red_out, green_out;
vector<Mat> channels;
int fd;

Point get_white_center(Mat* img);
vector<Point> get_blobs_center(Mat img, int min_area, int max_area);
vector<Point> get_map_pos(Mat* img);
vector<Point> get_routh_point(vector<Point> map, int subPointNum);
vector<Point> get_black_point(Mat img, int min_area, int max_area);
int get_closest_point(vector<Point> points, Point start);
Point getTransformPoint(Point pt_origin, Mat warpMatrix);

Point zero = {272,288};
vector<Point> map_positions;
vector<Point2f> map_pos_2f;
int status = 0;
int subPointNum = 60;
vector<Point2f> dst_points = {Point2f(0.0, 0.0),
							  Point2f(0.0, 560.0),
				              Point2f(560.0, 0.0),
				              Point2f(560.0, 560.0)};
vector<Point> map_border = {
    Point(30,30),
    Point(530,30),
    Point(530,530),
    Point(30,530)
};
int init_count = 0;

int tarCount = 0;
int tempTarCount = 0;
int maincount = 0;
vector<Point> globalRouth;
vector<Point> blackPoint;
string msg;
extern vector<Mat> show_buffer;

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
    capture.set(CAP_PROP_AUTO_EXPOSURE, 1);
    capture.set(CAP_PROP_EXPOSURE, 200);
    // capture.set(CAP_PROP_WB_TEMPERATURE, 90);
    cout << "CAP_PROP_FPS" << capture.get(CAP_PROP_FPS) << endl;
    cout << "CAP_PROP_BUFFERSIZE=" << capture.get(CAP_PROP_BUFFERSIZE) << endl;
    cout << "CAP_PROP_AUTO_WB=" << capture.get(CAP_PROP_AUTO_WB) << endl;
	cout << "CAP_PROP_WB_TEMPERATURE=" << capture.get(CAP_PROP_WB_TEMPERATURE) << endl;
    cout << "CAP_PROP_FRAME_SIZE=(" << capture.get(CAP_PROP_FRAME_WIDTH) << "," << capture.get(CAP_PROP_FRAME_HEIGHT) << ")" << endl;

    //uart init
    fd = uart_init("/dev/ttyUSB0", 115200, 8, 'N', 1);
    uart_receiveThread_touch();
	uart_send(fd, "main init finish\n");
    // lcd_init();

    // pthread_t tft_id;
    // int ret = pthread_create(&tft_id, NULL, lcd_show_process, NULL);
    // pthread_detach(tft_id);

    clock_t start, end;

    Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
    while(1)
    {   
		cout << "### 23E  ###" << endl;
        start = clock();
        capture >> frame;
        cout << "status = " << status << endl;
        if (status == 0)  // 初始化状态，标定坐标点
        {
            if (capture.get(CAP_PROP_EXPOSURE) <= 100)
            {
                capture.set(CAP_PROP_EXPOSURE, 200);
            }
            //识别绿色标定块
            split(frame, channels);
            red = channels.at(2);
            green = channels.at(1);
            blue = channels.at(0);

            green_mask = (max(green-blue, 0)) * 3;
            threshold(green_mask, green_out, 80, 255, THRESH_BINARY);
            dilate(green_out, green_out, kernel);

            // Point nowzero = {0,0};
            map_positions = get_map_pos(&green_out);
            // for (int i = 0; i < map_positions.size(); i++) 
            // {
            //     nowzero += map_positions[i];
            //     cout << map_positions[i] << endl;;
            // }
            // nowzero.x = int(nowzero.x / map_positions.size());
            // nowzero.y = int(nowzero.y / map_positions.size());
            // zero = nowzero;
            // cout << "zero = " << zero << endl;
            if (map_positions.size() == 4)
            {
                map_pos_2f.clear();
                for (int i=0; i<map_positions.size(); i++)
                {
                    map_pos_2f.push_back(Point2f(map_positions[i].x, map_positions[i].y));
                }
                rotation = getPerspectiveTransform(map_pos_2f,dst_points);
                cout << "--- reset rotation ---" << endl;
            }
            // imshow("green_out", green_out);
        }
        else if (status == 10)
        {
            // 确认标定(空状态)
            // 清空标定信息外到题目变量
            maincount = 0;
            tempTarCount = 0;
            tarCount = 0;
            globalRouth.clear();
            blackPoint.clear();
            cout << "map = " << map_pos_2f << endl;
        }
        else if (status == 101)
        {
            // 循环发送结束本题信号直到收到回复（超时自动退出）
            msg = "end";
            uart_send(fd, msg.c_str());
            usleep(1000);
            maincount++;
            if (maincount == 60)
            {
                maincount = 0;
                status = 10;
            }

        }
        else if (status == 11 || status == 15)
        {
            // 基础一，--复位, 识别绿色标定块  -------  发挥一检测系统同基础一复位
            //识别红色激光点
            cout << map_pos_2f.size() << endl;
			
			warpPerspective(frame, frame_warp, rotation, Size(560, 560));

            split(frame_warp, channels);
            red = channels.at(2);
            blue = channels.at(0);

            red_mask = (max(red-blue, 0)) * 2;
            threshold(red_mask, red_out, 128, 255, THRESH_BINARY);
            dilate(red_out, red_out, kernel);

            imshow("red_out", red_out);

            Point tar = get_white_center(&red_out);
            // tar = getTransformPoint(tar, rotation);
            Point offset = zero - tar;
            if (abs(offset.x) < 2 &&  abs(offset.y) < 2)  // 结束判定
            {
                msg = "end";
                uart_send(fd, msg.c_str());
                status = 101;
                continue;
            }
            
            cout << offset << endl;
            if (tar.x + tar.y == 0)
            {
                msg = "0x";
                uart_send(fd, msg.c_str());
                usleep(600); // sleep 1 ms
                msg = "0y";
                uart_send(fd, msg.c_str());
            }
            else{
                msg = to_string(offset.x) + "x";
                uart_send(fd, msg.c_str());
                usleep(600); // sleep 1 ms
                msg = to_string(offset.y) + "y";
                uart_send(fd, msg.c_str());
            }
            
            // circle(frame_warp, tar, 5, Scalar(255,0,0));
            putText(frame,"nowPos=("+to_string(offset.x) + ","+to_string(offset.y)+")",Point(20,60),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),1,2,false);
            imshow("red_out_b1", red_out);
            // imshow("frame_warp", frame_warp);
        }
        else if (status == 12)
        {
            // 基础2--寻板子边框
            //识别红色激光点

			warpPerspective(frame, frame_warp, rotation, Size(560, 560));

            split(frame_warp, channels);
            red = channels.at(2);
            green = channels.at(1);

            red_mask = (max(red-green, 0)) * 2;
            threshold(red_mask, red_out, 152, 255, THRESH_BINARY);
            dilate(red_out, red_out, kernel);

            imshow("red_out", red_out);

            Point now = get_white_center(&red_out);
            // now = getTransformPoint(now, rotation);
            // 计算路径上所有细分点
            if (globalRouth.size() == 0)
            {
                int start_point_idx = get_closest_point(map_border, now); 
                // globalRouth = get_routh_point(map_border, subPointNum);
                for (int i=0; i < map_border.size(); i++)
                {
                    globalRouth.push_back(map_border[(start_point_idx+i)%map_border.size()]);
                }
                // globalRouth = map_border;
                globalRouth.push_back(map_border[start_point_idx]);
            }
            else{
                Point tar = globalRouth[tempTarCount];
                Point offset = tar - now;

                if (abs(offset.x) < 3 &&  abs(offset.y) < 3)  // 达到中间路径点
                {
                    tempTarCount++;
                }

                if (now.x + now.y == 0)
                {
                    msg = "0x";
                    uart_send(fd, msg.c_str());
                    usleep(600); // sleep 1 ms
                    msg = "0y";
                    uart_send(fd, msg.c_str());
                }
                else{
                    msg = to_string(offset.x) + "x";
                    uart_send(fd, msg.c_str());
                    usleep(600); // sleep 1 ms
                    msg = to_string(offset.y) + "y";
                    uart_send(fd, msg.c_str());
                }

                if (tempTarCount == globalRouth.size())  // 运动结束
                {
                    msg = "end";
                    uart_send(fd, msg.c_str());
                    status = 101;
                    continue;
                }
                rectangle(frame_warp, cv::Point(30, 30), cv::Point(530, 530), cv::Scalar(0, 255, 255),2);
                imshow("frame_warp", frame_warp);
                putText(frame,"now=("+to_string(now.x) + ","+to_string(now.y)+")",Point(20,65),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),1,2,false);
                putText(frame,"tar=("+to_string(tar.x) + ","+to_string(tar.y)+")",Point(20,90),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),1,2,false);
            }
        }
        else if (status == 13 || status == 14 || status == 16)
        {
            // 基础三、四--识别黑色电工胶布并沿电工胶布前进
			warpPerspective(frame, frame_warp, rotation, Size(560, 560));

            if (blackPoint.size() == 0)
            {
                if (capture.get(CAP_PROP_EXPOSURE) <= 100)
                {
                    capture.set(CAP_PROP_EXPOSURE, 200);
                    continue;
                }
                blackPoint = get_black_point(frame_warp, 5000, 80000);
                // blackPoint.clear();
                cout << "getting blackPoint" << endl;
            }
            else
            {
                // 检测红色激光点位置并发送偏差
                split(frame_warp, channels);
                red = channels.at(2);
                green = channels.at(1);

                red_mask = (max(red-green, 0)) * 2;
                threshold(red_mask, red_out, 152, 255, THRESH_BINARY);
                dilate(red_out, red_out, kernel);

                imshow("red_out", red_out);

                Point now = get_white_center(&red_out);
                circle(frame_warp, now, 3, Scalar(255,0,255));

                if (globalRouth.size() == 0)
                {
                    int start_point_idx = get_closest_point(blackPoint, now); 
                    // globalRouth = get_routh_point(map_border, subPointNum);
                    vector<Point> black_sort;
                    for (int i=0; i < blackPoint.size(); i++)
                    {
                        black_sort.push_back(blackPoint[(start_point_idx+i+1)%blackPoint.size()]);
                    }
                    globalRouth = get_routh_point(black_sort, subPointNum);
                    // globalRouth = blackPoint;
                    // globalRouth.push_back(blackPoint[0]);
                }
                else{
                    // cout << globalRouth.size() << endl;
                    
                    Point tar = globalRouth[tempTarCount];
                    Point offset = tar - now;

                    Point medianOffset = {0,0};
                    if (tempTarCount > 1 && globalRouth[tempTarCount+5] != blackPoint[1] 
                        && globalRouth[tempTarCount+5] != blackPoint[2] && tempTarCount < globalRouth.size()-5)
                    {
                        medianOffset = globalRouth[tempTarCount+3] - now;
                        circle(frame_warp, globalRouth[tempTarCount+3], 3, Scalar(255,0,255));
                    }
                    else{
                        medianOffset = offset;
                        circle(frame_warp, globalRouth[tempTarCount], 3, Scalar(255,0,255));
                    }
                    // for (int i = 0)
                    
                    if (tempTarCount > 1 && tempTarCount < globalRouth.size()-1)
                    {
                        if (abs(offset.x) < 12 &&  abs(offset.y) < 12)  // 达到中间路径点
                        {
                            tempTarCount++;
                        }
                    }
                    else{
                        if (abs(offset.x) < 3 &&  abs(offset.y) < 3)  // 达到中间路径点
                        {
                            tempTarCount++;
                        }
                    }

                    if (now.x + now.y == 0)
                    {
                        msg = "0x";
                        uart_send(fd, msg.c_str());
                        usleep(600); // sleep 1 ms
                        msg = "0y";
                        uart_send(fd, msg.c_str());
                    }
                    else{
                        if (medianOffset.x > -4 && medianOffset.x < 0)
                        {
                            medianOffset.x = -4;
                        }
                        else if(medianOffset.x < 4 && medianOffset.x >= 0)
                        {
                            medianOffset.x = 4;
                        }
                        if (medianOffset.y > -4 && medianOffset.y < 0)
                        {
                            medianOffset.y = -4;
                        }
                        else if(medianOffset.y < 4 && medianOffset.y >= 0)
                        {
                            medianOffset.y = 4;
                        }
                        cout << "send offset" << endl;
                        msg = to_string(medianOffset.x) + "x";
                        uart_send(fd, msg.c_str());
                        usleep(600); // sleep 1 ms
                        msg = to_string(medianOffset.y) + "y";
                        uart_send(fd, msg.c_str());
                    }

                    if (tempTarCount == globalRouth.size())  // 运动结束
                    {
                        msg = "end";
                        uart_send(fd, msg.c_str());
                        status = 101;
                        continue;
                    }

                    // putText(frame,"now=("+to_string(now.x) + ","+to_string(now.y)+")",Point(20,65),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),1,2,false);
                    // putText(frame,"tar=("+to_string(tar.x) + ","+to_string(tar.y)+")",Point(20,90),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),1,2,false);
                    // putText(frame,"now=("+to_string(medianOffset.x) + ","+to_string(medianOffset.y)+")",Point(20,100),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),1,2,false);
                    // putText(frame,"tar=("+to_string(offset.x) + ","+to_string(offset.y)+")",Point(20,130),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),1,2,false);
                }
            }
            cout << "blackPoint=" << blackPoint.size() << endl;
            cout << blackPoint << endl;
            // for (int i=0; i < globalRouth.size(); i++)
            // {
            //     circle(frame_warp, globalRouth[i], 3, Scalar(255,0,255));
            // }
            imshow("frame_warp", frame_warp);
        }

        // split(frame, channels);
        // red = channels.at(2);
        // green = channels.at(1);
        // blue = channels.at(0);

        // // blackPoint =  get_black_point(frame, 4000, 50000);

        // red_mask = (max(red-green, 0)) * 2;
        // threshold(red_mask, red_out, 152, 255, THRESH_BINARY);
        // dilate(red_out, red_out, kernel);
        // imshow("red_out", red_out);

        // green_mask = (max(green-red, 0)) * 3;
        // threshold(green_mask, green_out, 119, 255, THRESH_BINARY);
        // dilate(green_out, green_out, kernel);

        // Point red_tar = get_white_center(&red_out);
        // cout << red_tar << endl;
        // get_blobs_center(red_out, 100, 1000);
        // circle(frame, red_tar, 5, Scalar(255,0,0));
        end = clock();
        double fps = 1/(double(end-start)/CLOCKS_PER_SEC);
        putText(frame,"status="+to_string(status),Point(20,40),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),1,1,false);
        putText(frame,"fps="+to_string(int(fps)),Point(20,20),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),1,1,false);
        circle(frame, zero, 5, Scalar(255,0,0));

        if (show_buffer.size() == 0)
        {
            show_buffer.push_back(frame);
        }
        
        imshow("frame", frame);
        // cout << "put img for tft" << endl;
        //else{
           // show_buffer[0] = frame;
        //}
        
        // imshow("red", red_out);
        // imshow("green", green_out);
        
        cout << "fps=" << fps << endl;
        int key = waitKey(1);
        if (key == 27)
        {
            break;
        }
    }
}

Point getTransformPoint(Point pt_origin, Mat warpMatrix)
{
	Mat_<double> mat_pt(3, 1);
	mat_pt(0, 0) = pt_origin.x;
	mat_pt(1, 0) = pt_origin.y;
	mat_pt(2, 0) = 1;
	Mat mat_pt_view = warpMatrix * mat_pt;
	double a1 = mat_pt_view.at<double>(0, 0);
	double a2 = mat_pt_view.at<double>(1, 0);
	double a3 = mat_pt_view.at<double>(2, 0);
	return Point(a1 * 1.0 / a3, a2 * 1.0 / a3);
}

int get_closest_point(vector<Point> points, Point start)
{
    vector<double> distance;
    for(int i=0; i < points.size(); i++)
    {
        distance.push_back(sqrtf((start.x-points[i].x)*(start.x-points[i].x)+(start.y-points[i].y)*(start.y-points[i].y)));
    }
    int smallest = min_element(distance.begin(), distance.end()) - distance.begin();
    return smallest;
}

vector<Point> get_black_point(Mat img, int min_area, int max_area)
{
    vector<vector<Point>> blackpoint;
    Mat gray, img_threshold;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    threshold(gray, img_threshold, 130, 255, THRESH_BINARY);
    threshold(gray, img_threshold, 0, 255, THRESH_BINARY | THRESH_OTSU);
    imshow("black", img_threshold);
    
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
	morphologyEx(img_threshold, img_threshold, MORPH_CLOSE, kernel);
    vector<vector<Point>> contours;
    vector<Point> contour_ploy;
	vector<Vec4i> hierarchy;
    // Mat img_cut = img_threshold(Rect(0,0,432, 480));
    // imshow("img_cut", img_cut);
	findContours(img_threshold, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    drawContours(img, contours, -1, Scalar(255,255,0), 3);
    for (int i = 0; i < contours.size(); i++)
    {
        int area = contourArea(contours[i]);
        // cout << "area=" << area << endl;
		if (area > min_area && area < max_area)
		{
            cout << "area=" << area << endl;
            double cnt_len = arcLength(contours[i], true);
            approxPolyDP(contours[i], contour_ploy, 0.02*cnt_len, true);
            if (contour_ploy.size() >= 4)
            {
                blackpoint.push_back(contour_ploy);
                drawContours(img, blackpoint, -1, Scalar(0,255,255), 3);
            }
        }
    }
    
    for (int i=0; i < blackpoint.size(); i++)
    {
        vector<int> temp;
        vector<Point> rec_pos_sort;
		for (int j=0; j<blackpoint[i].size(); j++)
		{
			int xandy = blackpoint[i][j].x + blackpoint[i][j].y;
			temp.push_back(xandy);
		}
		int max_index = max_element(temp.begin(), temp.end()) - temp.begin();
		int min_index = min_element(temp.begin(), temp.end()) - temp.begin();
		rec_pos_sort.push_back(blackpoint[i][min_index]);
		rec_pos_sort.push_back(blackpoint[i][max_index]);
		int r1=-1, r2=-1;
		for (int i=0; i<4; i++)
		{
			if (i != min_index && i != max_index)
			{
				if (r1==-1)  r1 = i;
				else  r2 = i;
			}
		}
		if (blackpoint[i][r1].y < blackpoint[i][r2].y)
		{
			rec_pos_sort.push_back(blackpoint[i][r1]);
			rec_pos_sort.push_back(blackpoint[i][r2]);
		}
		else
		{
			rec_pos_sort.push_back(blackpoint[i][r2]);
			rec_pos_sort.push_back(blackpoint[i][r1]);
		}
		Point tempPoint = rec_pos_sort[1];
		rec_pos_sort[1] = rec_pos_sort[3];
		rec_pos_sort[3] = tempPoint;
        tempPoint = rec_pos_sort[2];
		rec_pos_sort[2] = rec_pos_sort[3];
		rec_pos_sort[3] = tempPoint;

        blackpoint[i] = rec_pos_sort;
    }
    vector<Point> center_black;
    if (blackpoint.size() >= 2)
    {
        for (int i =0; i < 4; i++)
        {
            Point temp;
            temp.x = int((blackpoint[0][i].x + blackpoint[1][i].x)/2);
            temp.y = int((blackpoint[0][i].y + blackpoint[1][i].y)/2);
            center_black.push_back(temp);
        }
    }

    return center_black;
}

Point get_white_center(Mat* img)
{
    Point center_points = {0, 0}; 
	int count = 0, R = 0;
    for (int row=0; row < img->rows; row++)
    {
        for (int col=0; col < img->cols; col++)
        {
            R = img->at<uchar>(row, col);
            if (R == 255)
            {
                center_points.y += row;
                center_points.x += col;
                count++;
            }
        }
    }
	if (count > 0)
    {
        center_points.y = int(center_points.y / count);
        center_points.x = int(center_points.x / count);
    }
    return center_points;
}


vector<Point> get_blobs_center(Mat img, int min_area, int max_area)
{
	vector<Point> center_points;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	for (int i = 0; i < contours.size(); i++)
	{
		int area = contourArea(contours[i]);
        // drawContours(frame, contours, i, Scalar(0, 255, 120), 2, 8, hierarchy);
		if (area > min_area && area < max_area)
		{
			Rect bound = boundingRect(contours[i]);
			Point center = {0,0};
			int count = 0;
			for (int j=0; j<bound.width; j++)
			{
				for (int k=0; k< bound.height; k++)
				{
					if (img.at<uchar>(bound.y+k, bound.x+j) == 255)
					{
						center += {bound.x+j, bound.y+k};
						count++;
					}
				}
			}
			center.x /= count;
			center.y /= count;
			center_points.push_back(center);
			circle(frame, center, 3, Scalar(0, 255, 120), -1);
			// circle(frame, center, 5, Scalar(0, 255, 120), -1);
			rectangle(frame, bound.tl(), bound.br(), Scalar(0, 255, 0), 5);
		}
	}
	return center_points;
}

vector<Point> get_map_pos(Mat* img)
{
	vector<Point> rec_pos_sort;
	vector<Point> rec_pos = get_blobs_center(*img, 500, 4000);

	if (rec_pos.size() == 4)
	{
		vector<int> temp;
		for (int i=0; i<rec_pos.size(); i++)
		{
			int xandy = rec_pos[i].x + rec_pos[i].y;
			temp.push_back(xandy);
		}
		int max_index = max_element(temp.begin(), temp.end()) - temp.begin();
		int min_index = min_element(temp.begin(), temp.end()) - temp.begin();
		rec_pos_sort.push_back(rec_pos[min_index]);
		rec_pos_sort.push_back(rec_pos[max_index]);
		int r1=-1, r2=-1;
		for (int i=0; i<4; i++)
		{
			if (i != min_index && i != max_index)
			{
				if (r1==-1)  r1 = i;
				else  r2 = i;
			}
		}
		if (rec_pos[r1].y < rec_pos[r2].y)
		{
			rec_pos_sort.push_back(rec_pos[r1]);
			rec_pos_sort.push_back(rec_pos[r2]);
		}
		else
		{
			rec_pos_sort.push_back(rec_pos[r2]);
			rec_pos_sort.push_back(rec_pos[r1]);
		}
		Point tempPoint = rec_pos_sort[1];
		rec_pos_sort[1] = rec_pos_sort[3];
		rec_pos_sort[3] = tempPoint;
	}
	return rec_pos_sort;
}

vector<Point> get_routh_point(vector<Point> map, int subPointNum)
{
    vector<Point> routh;
    Point start, end, temp;

    for (int i=map.size()-1; i >= 0; i--)
    {
        if (i == 0)
        {
            start = map[i];
            end = map[map.size()-1];
        }
        else{
            start = map[i];
            end = map[i-1];
        }
        cout << "start" << start << endl;
        cout << "end" << end << endl;
        // subPointNum = abs((end.x - start.x)/3);
        float stepx = float((end.x - start.x)) / subPointNum;
        float stepy = float((end.y - start.y)) / subPointNum;
        cout << "stepx" << stepx << endl;
        cout << "stepy" << stepy << endl;
        for (int j=0; j < subPointNum; j++)
        {
            temp.x = int(j*stepx + start.x);
            temp.y = int(j*stepy + start.y);
            // if (temp.x > end.x || temp.y > end.y || temp.x < start.x || temp.y < start.y)
            // {
            //     break;
            // }
            cout << temp << endl;
            routh.push_back(temp);
        }
        routh.push_back(end);
    }
    return routh;
}
