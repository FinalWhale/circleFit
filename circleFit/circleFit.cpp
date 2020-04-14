/*vs2017 opencv2.4.13*/

#include "circleFit.h"


class Caliparam
{
public:
	float start_point[2];
	float Calib_alpha_in[4];
	float Calib_alpha_out[4];
	float xyc;
	float LA;
	float LB;
	float PMR;
	float mag_px;
	float s_o;
};

vector<double> cal_outline_d(Mat Depth_mat, vector<pair<double, double>> point_pair, string cali_path,  int Cols);
Mat txt_transfer_mat(const char* inputDepth_mat, int Rows, int Cols);
vector<double> Parallax_depth_z(vector<double > z, string cali_path, int Cols);

vector<double> Parallax_depth_xy(vector<pair<double, double>> point_pair, vector<double>z_real, string cali_path, int Cols);

void delete_invalid_data(vector<double> z, vector<double> xy, vector<double>& z_no_invalid_data, vector<double>& xy_no_invalid_data);

double circleLeastFit(vector<double> z, vector<double> xy);


//int main()
//{
//	int Cols = 800;
//	int Rows = 545;
//	
//	const char* inputDepth_mat = "D:\\SAIGUI2X\\depth\\depth01.txt";
//	string cali_path = "D:\\SAIGUI2X\\cali.txt";
//	
//	vector<pair<double, double>> point_pair;
//	pair<double, double> point1;
//	pair<double, double> point2;
//	point1.first = 709;
//	point1.second = 411;
//	point_pair.push_back(point1);
//	point2.first = 691;
//	point2.second = 227;
//	point_pair.push_back(point2);
//	/*È¥³ýµô-999*/
//	vector<double> z_no_invalid_data;
//	vector<double> xy_no_invalid_data;
//
//	double rad=circleFit( Cols, Rows,   inputDepth_mat,  cali_path,  z_no_invalid_data,  xy_no_invalid_data,  point_pair);
//	cout << "rad=" << rad << endl;
//	
//
//	return 0;
//}


vector<double> Parallax_depth_z(vector<double> z, string cali_path, int Cols)
{

	vector<double> z_temp;

	Caliparam caliparam;
	//¶ÁÈ¡Ð£×¼²ÎÊý
	float * paramlist = new float[16];
	//string name = _str + "cali.txt";
	char filename[100];
	strcpy(filename, cali_path.c_str());
	FILE *fp;
	fp = fopen(filename, "rb");//
	/*if (fp == NULL)
	{
		return 0;
	}*/
	fread(paramlist, sizeof(float), 16, fp);
	fclose(fp);
	caliparam.start_point[0] = paramlist[0];
	caliparam.start_point[1] = paramlist[1];
	for (int i = 0; i < 4; i++)
	{
		caliparam.Calib_alpha_in[i] = paramlist[i + 2];
	}
	for (int i = 0; i < 4; i++)
	{
		caliparam.Calib_alpha_out[i] = paramlist[i + 2];
	}
	caliparam.xyc = paramlist[10];
	caliparam.LA = paramlist[11];
	caliparam.LB = paramlist[12];
	caliparam.PMR = paramlist[13];
	caliparam.mag_px = paramlist[14] * 800 / Cols;
	caliparam.s_o = paramlist[15];

	for (int i = 0; i < z.size(); i++)
	{
		double CZ;
		if (z[i] == -999) {
			CZ = -999;
		}

		else
		{
			CZ = 1.0 / (caliparam.LA*z[i] * caliparam.PMR*caliparam.PMR*(caliparam.PMR / caliparam.mag_px)*(caliparam.PMR / caliparam.mag_px) / caliparam.Calib_alpha_in[0] + caliparam.LB) - caliparam.Calib_alpha_in[3];
			CZ -= paramlist[15];
			CZ = CZ;
		}
		z_temp.push_back(CZ);
	}
	return z_temp;
}


/*·µ»Øxy_distance ÕæÊµ³¤¶È¼°z·½ÏòµÄÕæÊµ³¤¶È*/
vector<double> cal_outline_d(Mat Depth_mat, vector<pair<double,double>> point_pair, string cali_path,  int Cols) {

	vector<double > z;
	double xy_distance_pixel;

	double u1 = point_pair[0].first;
	double v1 = point_pair[0].second;
	double u2 = point_pair[1].first;
	double v2 = point_pair[1].second;

	int cols = Depth_mat.cols;
	int rows = Depth_mat.rows;
	if (u1 == u2)
	{
		xy_distance_pixel = abs(v1 - v2);
		if (v1 > v2) {
			for (int i = 0; i < (abs(v2 - v1)); i++)
			{
				double temp = Depth_mat.at<float>(v2 + i, u1);
				z.push_back(temp);
			}

		}

		else if (v1 < v2) {
			for (int i = 0; i < (abs(v2 - v1)); i++)
			{
				double temp = Depth_mat.at<float>(v1 + i, u1);
				z.push_back(temp);
			}

		}

	}
	else if (v1 == v2)
	{
		xy_distance_pixel = abs(u1 - u2);

		if (u1 > u2) {
			for (int i = 0; i < (abs(u2 - u1)); i++)
			{
				double temp = Depth_mat.at<float>(v1, u2 + i);

				z.push_back(temp);
			}
		}

		else if (u1 < u2) {
			for (int i = 0; i < (abs(u2 - u1)); i++)
			{
				double temp = Depth_mat.at<float>(v1, u1 + i);
				z.push_back(temp);
			}
		}

	}
	else
	{
		xy_distance_pixel = sqrt(pow((u1 - u2), 2) + pow((v1 - v2), 2));
		int num = sqrt(pow((u1 - u2), 2) + pow((v1 - v2), 2));
		double per_lenght_i = (double(u1 - u2) / (num));
		double per_lenght_j = (double(v1 - v2) / (num));
		for (double k = 0; k < num; k++)
		{
			int temp_i = int(u1 - k * per_lenght_i + 0.5);
			int temp_j = int(v1 - k * per_lenght_j + 0.5);
			if (temp_i > rows)
				temp_i = rows;
			if (temp_j > cols)
				temp_j = cols;

			z.push_back(Depth_mat.at<float>(temp_j, temp_i));
		}

	}

	//cout << xy_distance_pixel << endl;


	//xy_distance = Parallax_depth_xy(xy_distance_pixel, cali_path, Cols);
	//vector<double> z_d = Parallax_depth_z(z, cali_path, Cols);
	return z;
}

Mat txt_transfer_mat(const char* inputDepth_mat, int Rows, int Cols)
{

	FILE * pFile;
	float * depth;
	size_t result;
	errno_t err;

	err = fopen_s(&pFile, inputDepth_mat, "rb");
	if (pFile == NULL) { fputs("Depth File error", stderr); exit(-1); }
	depth = new float[Rows * Cols];
	if (depth == NULL) { fputs("Memory error", stderr); exit(-2); }
	result = fread(depth, 4, Rows * Cols, pFile);
	if (result != Rows * Cols) { fputs("Depth Reading error", stderr); exit(-3); }
	fclose(pFile);

	Mat Depth_mat = Mat(1, Rows * Cols, CV_32F, depth);

	Depth_mat = Depth_mat.reshape(0, Rows);



	return  Depth_mat;

}


vector<double> Parallax_depth_xy(vector<pair<double,double>> point_pair, vector<double>z_real, string cali_path, int Cols)
{
	Caliparam caliparam;
	//¶ÁÈ¡Ð£×¼²ÎÊý
	float * paramlist = new float[16];
	//string name = _str + "cali.txt";
	char filename[100];
	strcpy(filename, cali_path.c_str());
	FILE *fp;
	fp = fopen(filename, "rb");//
	/*if (fp == NULL)
	{
		return 0;
	}*/
	fread(paramlist, sizeof(float), 16, fp);
	fclose(fp);
	caliparam.start_point[0] = paramlist[0];
	caliparam.start_point[1] = paramlist[1];
	for (int i = 0; i < 4; i++)
	{
		caliparam.Calib_alpha_in[i] = paramlist[i + 2];
	}
	for (int i = 0; i < 4; i++)
	{
		caliparam.Calib_alpha_out[i] = paramlist[i + 2];
	}
	caliparam.xyc = paramlist[10];
	caliparam.LA = paramlist[11];
	caliparam.LB = paramlist[12];
	caliparam.PMR = paramlist[13];
	caliparam.mag_px = paramlist[14] * 800 / Cols;
	caliparam.s_o = paramlist[15];

	/**/
	double u1 = point_pair[0].first;
	double v1 = point_pair[0].second;
	double u2 = point_pair[1].first;
	double v2 = point_pair[1].second;

	double xy_distance_pixel = sqrt(pow((u1 - u2), 2)+ pow((v1 - v2),2));
	double xy_distance_real=  xy_distance_pixel * caliparam.xyc*caliparam.s_o*caliparam.mag_px;

	vector<double> xy_vec;
	double temp;
	double distance_per = xy_distance_real / z_real.size();

	for (int i = 0; i < z_real.size(); i++)
	{
		temp = i * distance_per;
		xy_vec.push_back(temp);

	}

	return xy_vec;
}

void delete_invalid_data(vector<double> z, vector<double> xy, vector<double>& z_no_invalid_data, vector<double>& xy_no_invalid_data)
{
	
	
		for (int i = 0; i < z.size(); i++)
		{
			if (z[i] != -999)
			{
				z_no_invalid_data.push_back(z[i]);
				xy_no_invalid_data.push_back(xy[i]);
			}

		}

		
}

double circleLeastFit(vector<double> z, vector<double> xy)
{
	double radius;

	

	
		double sum_x = 0.0f, sum_y = 0.0f;
		double sum_x2 = 0.0f, sum_y2 = 0.0f;
		double sum_x3 = 0.0f, sum_y3 = 0.0f;
		double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;
		
		int N = z.size();
		for (int j = 0; j < z.size(); j++)
		{
			double x = xy[j];
			double y = z[j];
			double x2 = x * x;
			double y2 = y * y;
			sum_x += x;
			sum_y += y;
			sum_x2 += x2;
			sum_y2 += y2;
			sum_x3 += x2 * x;
			sum_y3 += y2 * y;
			sum_xy += x * y;
			sum_x1y2 += x * y2;
			sum_x2y1 += x2 * y;
		}
		double C, D, E, G, H;
		double a, b, c;
		C = N * sum_x2 - sum_x * sum_x;
		D = N * sum_xy - sum_x * sum_y;
		E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
		G = N * sum_y2 - sum_y * sum_y;
		H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
		a = (H * D - E * G) / (C * G - D * D);
		b = (H * C - E * D) / (D * D - G * C);
		c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;
		//center.x = a / (-2);
		//center.y = b / (-2);
		radius = sqrt(a * a + b * b - 4 * c) / 2;

		return radius;
	}


double circleFit(int Cols, int Rows,  const char* inputDepth_mat,string cali_path, vector<double>& z_no_invalid_data, vector<double>& xy_no_invalid_data, vector<pair<double, double>> point_pair)
{
	Mat Depth_mat = txt_transfer_mat(inputDepth_mat, Rows, Cols);
	vector<double> outline_point = cal_outline_d(Depth_mat, point_pair, cali_path, Cols);
	vector<double> depth_z_vec = Parallax_depth_z(outline_point, cali_path, Cols);

	vector<double>xy_vec = Parallax_depth_xy(point_pair, depth_z_vec, cali_path, Cols);

	delete_invalid_data(depth_z_vec, xy_vec, z_no_invalid_data, xy_no_invalid_data);

	double rad = circleLeastFit(z_no_invalid_data, xy_no_invalid_data);

	return rad;
}


