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
 Cols ͼƬ����
 Rows ͼƬ����
 inputDepth_mat depth01.txt
 cali_path У׼�ļ���ַ
 z_no_invalid_data ���ù���
 xy_no_invalid_data ���ù���
 point_pair ����ĵ��

*/

_CIRCLEFIT double circleFit(int Cols, int Rows,  const char* inputDepth_mat, string cali_path, 
	vector<double>& z_no_invalid_data, vector<double>& xy_no_invalid_data, vector<pair<double, double>> point_pair);
