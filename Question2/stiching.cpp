#include"stiching.h"

void Corners::CalcCorners(const cv::Mat& H, const cv::Mat& src) {
	
	//对图像左上角点进行单映性变换
	cv::Mat corner_ori = cv::Mat::zeros(3, 1, CV_64F);
	corner_ori.at<double>(2, 0) = 1;
	//cout << corner_ori << endl;
	cv::Mat corner_homo = H * corner_ori;
	leftUp.x = corner_homo.at<double>(0, 0) / corner_homo.at<double>(2, 0);
	leftUp.y = corner_homo.at<double>(1, 0) / corner_homo.at<double>(2, 0);
	//std::cout << leftUp.x << " " << leftUp.y << endl;

	//对图像左下角点进行单映性变换
	corner_ori.at<double>(0, 0) = 0;
	corner_ori.at<double>(1, 0) = src.rows;
	corner_ori.at<double>(2, 0) = 1;
	//cout << corner_ori << endl;
	corner_homo = H * corner_ori;
	leftDown.x = corner_homo.at<double>(0, 0) / corner_homo.at<double>(2, 0);
	leftDown.y = corner_homo.at<double>(1, 0) / corner_homo.at<double>(2, 0);
	//std::cout << leftDown.x << " " << leftDown.y << endl;

	//对图像右上角点进行单映性变换
	corner_ori.at<double>(0, 0) = src.cols;
	corner_ori.at<double>(1, 0) = 0;
	corner_ori.at<double>(2, 0) = 1;
	//cout << corner_ori << endl;
	corner_homo = H * corner_ori;
	rightUp.x = corner_homo.at<double>(0, 0) / corner_homo.at<double>(2, 0);
	rightUp.y = corner_homo.at<double>(1, 0) / corner_homo.at<double>(2, 0);
	//std::cout << rightUp.x << " " << rightUp.y << endl;

	//对图像右下角点进行单映性变换
	corner_ori.at<double>(0, 0) = src.cols;
	corner_ori.at<double>(1, 0) = src.rows;
	corner_ori.at<double>(2, 0) = 1;
	//cout << corner_ori << endl;
	corner_homo = H * corner_ori;
	rightDown.x = corner_homo.at<double>(0, 0) / corner_homo.at<double>(2, 0);
	rightDown.y = corner_homo.at<double>(1, 0) / corner_homo.at<double>(2, 0);
	//std::cout << rightDown.x << " " << rightDown.y << endl;


}

Corners::Corners(const cv::Mat& H, const cv::Mat& src) {
	CalcCorners(H, src);
}

void Corners::reCalcCorners(const cv::Mat& H, const cv::Mat& src) {
	CalcCorners(H, src);
}

const double Stiching::superpositionLeftThread = 0.92;
const double Stiching::superpositionRightThread = 0.1;

void Stiching::stiching(cv::Mat& L_Img_Ori, cv::Mat& R_Img_Ori, cv::Mat& dst) {

	cv::Mat L_Img, R_Img;
	cv::cvtColor(L_Img_Ori, L_Img, CV_RGB2GRAY);
	cv::cvtColor(R_Img_Ori, R_Img, CV_RGB2GRAY);
	//std::cout << L_Img.rows << " " << R_Img.cols << std::endl;
	auto L_row = L_Img.rows;
	auto L_col = L_Img.cols;
	auto R_row = R_Img.rows;
	auto R_col = R_Img.cols;
	//std::cout << L_row << L_col << " " << R_row << R_col << endl;
	initial();
	//cv::Ptr<cv::xfeatures2d::SIFT> tsift = cv::xfeatures2d::SIFT::create(1600);
	std::vector<cv::KeyPoint> L_KeyPoints, R_KeyPoints;
	sift->detect(L_Img, L_KeyPoints);
	sift->detect(R_Img, R_KeyPoints);
	int thread_L = L_col * superpositionLeftThread;
	int thread_R = R_col * superpositionRightThread;
	//std::cout << thread_L << " " << thread_R << std::endl;
	for (int i = 0; i < L_KeyPoints.size(); i++) {
		cv::KeyPoint tmp = L_KeyPoints.at(i);
		if (tmp.pt.x > thread_L) {
			L_KeyPts.push_back(tmp);
		}
	}
	for (int i = 0; i < R_KeyPoints.size(); i++) {
		cv::KeyPoint tmp = R_KeyPoints.at(i);
		if (tmp.pt.x < thread_R) {
			R_KeyPts.push_back(tmp);
		}
	}
	//std::cout << L_KeyPts.size() << " " << R_KeyPts.size() << std::endl;
	sift->compute(L_Img, L_KeyPts, descriptors_L);
	sift->compute(R_Img, R_KeyPts, descriptors_R);
	std::vector<cv::Point2f> imagePoints_L, imagePoints_R;
	matchByBF(imagePoints_L, imagePoints_R);
	//std::cout << imagePoints_L.size() << " " << imagePoints_R.size() << std::endl;
	cv::Mat homo;

	//建立单映性矩阵
	if (imagePoints_L.size() > 30)
		homo = cv::findHomography(imagePoints_R, imagePoints_L, CV_RANSAC);
	else {
		double h[] = { 0.7731306674544454, 0.002603557890442372, 593.9808144299284,
					-0.1078747736838854, 0.9974778502779783, -7.538690884299091,
					-0.000358802476608091, 1.88412002496368e-05, 1 };
	homo = cv::Mat(3, 3, CV_64FC1, h);
	}
	//cv::Mat homo = cv::findHomography(imagePoints_R, imagePoints_L, CV_RANSAC);
	//cout << homo << endl;
	
	corners.reCalcCorners(homo, R_Img);
	cv::Mat imageTransform1;
	//std::cout << homo << std::endl;

	/*
	cv::warpPerspective(R_Img_Ori, imageTransform1, homo, cv::Size(MAX(corners.rightUp.x, corners.rightDown.x), corners.rightDown.y));
	int dst_width = imageTransform1.cols;
	int dst_height = corners.rightDown.y;
	dst = cv::Mat(dst_height, dst_width, CV_8UC3);
	dst.setTo(0);
	imageTransform1.copyTo(dst(cv::Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
	L_Img_Ori.copyTo(dst(cv::Rect(0, 0, L_Img.cols, L_Img.rows)));
	cv::imshow("aa", dst);
	cv::waitKey(10);
	OptimizeSeam(L_Img_Ori, imageTransform1, dst);
	*/

	//Change Homo
	
	homo.at<double>(1, 2) = homo.at<double>(1, 2) - corners.rightUp.y; //-1
	warpPerspective(R_Img_Ori, imageTransform1, homo, cv::Size(MAX(corners.rightUp.x, corners.rightDown.x), corners.rightDown.y - corners.rightUp.y/*L_Img_Ori.rows*/), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

	cv::waitKey(10);
	//创建拼接后的图,需提前计算图的大小
	int dst_width = imageTransform1.cols;  //取最右点的长度为拼接图的长度
	int dst_height = imageTransform1.rows;
	dst = cv::Mat(dst_height, dst_width, CV_8UC3);
	dst.setTo(0);
	imageTransform1.copyTo(dst(cv::Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
	//imshow("imgTran", imageTransform1);
	L_Img_Ori.copyTo(dst(cv::Rect(0, -corners.rightUp.y + 1, L_Img_Ori.cols, L_Img_Ori.rows)));
	cv::Mat L_Img_Tmp(dst_height, L_Img_Ori.cols, CV_8UC3);
	L_Img_Tmp.setTo(0);
	L_Img_Ori.copyTo(L_Img_Tmp(cv::Rect(0, -corners.rightUp.y + 1, L_Img_Ori.cols, L_Img_Ori.rows)));
	//imshow("cc", L_Img_Tmp);
	//imshow("ss", dst);
	cv::waitKey(10);
	OptimizeSeam(L_Img_Tmp, imageTransform1, dst);
	

}

void Stiching::initial() {
	sift = cv::xfeatures2d::SIFT::create(1600);
}

void Stiching::matchByBF(std::vector<cv::Point2f>& L_Points, std::vector<cv::Point2f>& R_Points) {
	cv::BFMatcher  matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(descriptors_L, descriptors_R, matches);

	sort(matches.begin(), matches.end());
	std::vector< cv::DMatch > good_matches;
	int ptsPairs = matches.size();

	
	for (int i = 0; i < ptsPairs; i++)
	{
		good_matches.push_back(matches[i]);//距离最小的50个压入新的DMatch
	}

	for (int i = 0; i < good_matches.size(); i++)
	{
		L_Points.push_back(L_KeyPts[good_matches[i].queryIdx].pt);
		R_Points.push_back(R_KeyPts[good_matches[i].trainIdx].pt);
	}
}


//void Stiching::OptimizeSeam(cv::Mat& img1, cv::Mat& trans, cv::Mat& dst)
//{
//	int start = MIN(corners.leftUp.x, corners.leftDown.x);//开始位置，即重叠区域的左边界  
//	
//	double processWidth = img1.cols - start;//重叠区域的宽度  
//	int rows = dst.rows;
//	int cols = img1.cols; //注意，是列数*通道数
//	double alpha = 1;//img1中像素的权重  
//	for (int i = 0; i < rows; i++)
//	{
//		if (i < img1.rows) {
//			uchar* p = img1.ptr<uchar>(i);  //获取第i行的首地址
//			uchar* t = trans.ptr<uchar>(i);
//			uchar* d = dst.ptr<uchar>(i);
//			for (int j = start; j < cols; j++)
//			{
//				//如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
//				if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
//				{
//					alpha = 1;
//				}
//				else
//				{
//					//img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好  
//					alpha = (processWidth - (j - start)) / processWidth;
//				}
//
//				d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
//				d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
//				d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);
//
//			}
//		}
//	}
//}

void Stiching::OptimizeSeam(cv::Mat& img1, cv::Mat& trans, cv::Mat& dst)
{
	int start = MIN(corners.leftUp.x, corners.leftDown.x);//开始位置，即重叠区域的左边界  

	double processWidth = img1.cols - start;//重叠区域的宽度  
	int rows = dst.rows;
	int cols = img1.cols; //注意，是列数*通道数
	double alpha = 1;//img1中像素的权重  
	for (int i = 0; i < rows; i++)
	{
		uchar* p = img1.ptr<uchar>(i);  //获取第i行的首地址
		uchar* t = trans.ptr<uchar>(i);
		uchar* d = dst.ptr<uchar>(i);
		for (int j = start; j < cols; j++)
		{
			if (p[j * 3] == 0 && p[j * 3 + 1] == 0 && p[j * 3 + 2] == 0) {
				d[j * 3] = 0;
				d[j * 3 + 1] = 0;
				d[j * 3 + 2] = 0;
			}
			else {
				//如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
				if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
				{
					alpha = 1;
				}
				else
				{
					//img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好  
					alpha = (processWidth - (j - start)) / processWidth;
				}

				d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
				d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
				d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);
			}


		}
	}

}