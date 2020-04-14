#pragma once

#ifndef _CIRCLEFIT  
#define _CIRCLEFIT   _declspec(dllexport)  
#else  
#define _CIRCLEFIT   _declspec(dllimport)  
#endif
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

/*
 Cols 图片的列
 Rows 图片的行
 inputDepth_mat depth01.txt
 cali_path 校准文件地址
 z_no_invalid_data 不用关心
 xy_no_invalid_data 不用关心
 point_pair 输入的点对

*/

_CIRCLEFIT double circleFit(int Cols, int Rows,  const char* inputDepth_mat, string cali_path, 
	vector<double>& z_no_invalid_data, vector<double>& xy_no_invalid_data, vector<pair<double, double>> point_pair);
