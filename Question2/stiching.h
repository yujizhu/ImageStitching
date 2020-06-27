#pragma once
#include<opencv2/opencv.hpp>
#include<opencv2/xfeatures2d.hpp>
#include<vector>
//#include<iostream>

//using namespace std;

class Corners {
public:
	Corners() = default;
	Corners(const Corners& corners) :leftUp(corners.leftUp), leftDown(corners.leftDown), rightUp(corners.rightUp), rightDown(corners.rightDown)
	{ }
	Corners(const cv::Mat& H, const cv::Mat& src);
	~Corners() = default;
	void reCalcCorners(const cv::Mat& H, const cv::Mat& src);
	cv::Point2f leftUp;
	cv::Point2f leftDown;
	cv::Point2f rightUp;
	cv::Point2f rightDown;
private:
	void CalcCorners(const cv::Mat& H, const cv::Mat& src);
};

class Stiching {
public:
	Stiching() = default;
	Stiching(const Stiching&) = delete;
	Stiching(const Stiching&&) = delete;
	~Stiching() = default;
	void stiching(cv::Mat& L_Img_Ori, cv::Mat& R_Img_Ori, cv::Mat& dst);
	
private:
	void initial();
	//void detect(const cv::Mat& img, std::vector <cv::KeyPoint>& KeyPoints);
	//void compute();
	void matchByLowe() {};
	void matchByBF(std::vector<cv::Point2f>& L_Points, std::vector<cv::Point2f>& R_Points);
	void OptimizeSeam(cv::Mat& img1, cv::Mat& trans, cv::Mat& dst);
	Corners corners;
	static const  double superpositionLeftThread;
	static const  double superpositionRightThread;
	cv::Ptr<cv::xfeatures2d::SIFT> sift;
	std::vector<cv::KeyPoint> L_KeyPts, R_KeyPts;
	cv::Mat descriptors_L, descriptors_R;
};



