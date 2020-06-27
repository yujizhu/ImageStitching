#include"stiching.h"
#include<ctime>
using namespace cv;
using namespace std;


int main() {
	Mat L_Img_Ori_0, R_Img_Ori_0, dst_0;
	Mat L_Img_Ori_1, R_Img_Ori_1, dst_1;
	Mat L_Img_Ori_2, R_Img_Ori_2, dst_2;
	Mat L_Img_Ori_3, R_Img_Ori_3, dst_3;
	L_Img_Ori_0 = imread("L000.jpg");
	R_Img_Ori_0= imread("R000.jpg");
	L_Img_Ori_1 = imread("L001.bmp");
	R_Img_Ori_1 = imread("R001.bmp");
	L_Img_Ori_2 = imread("L002.bmp");
	R_Img_Ori_2 = imread("R002.bmp");
	L_Img_Ori_3 = imread("L004.bmp");
	R_Img_Ori_3 = imread("R004.bmp");
	//imshow("aa", L_Img_Ori_0);
	//imshow("bb", R_Img_Ori_0);
	//cout << L_Img_Ori_0.rows << " " << L_Img_Ori_0.cols << endl;
	waitKey(10);
	clock_t startTime, endTime;
	Stiching s0,s1,s2,s3;
	startTime = clock();
	s0.stiching(L_Img_Ori_0, R_Img_Ori_0, dst_0);
	s1.stiching(L_Img_Ori_1, R_Img_Ori_1, dst_1);
	s2.stiching(L_Img_Ori_2, R_Img_Ori_2, dst_2);
	s3.stiching(L_Img_Ori_3, R_Img_Ori_3, dst_3);
	//s2.stiching()
	endTime = clock();
	//cout << "Time: " << (endTime - startTime)  << "ms" << endl;
	imshow("0_dst", dst_0);
	imshow("1_dst", dst_1);
	imshow("2_dst", dst_2);
	imshow("3_dst", dst_3);
	imwrite("0_dst.jpg", dst_0);
	imwrite("1_dst.bmp", dst_1);
	imwrite("2_dst.bmp", dst_2);
	imwrite("3_dst.bmp", dst_3);
	cv::waitKey(10);

	int a;
	cin >> a;
}