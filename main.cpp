#include <iostream>
#include "ADCensusStereo.h"
#include <chrono>
#include "utils.h"
using namespace std::chrono;	// 时间库

// opencv library
#include <opencv2/opencv.hpp>

/**
* \brief
* \param argv 3
* \param argc argc[1]:左影像路径 argc[2]: 右影像路径 argc[3]: 最小视差[可选，默认0] argc[4]: 最大视差[可选，默认64]
* \param eg. ..\Data\cone\im2.png ..\Data\cone\im6.png 0 64
* \param eg. ..\Data\Cloth3\view1.png ..\Data\Cloth3\view5.png 0 128
* \return
*/
int main(int argv, char** argc)
{
	if (argv < 3) {
		std::cout << "参数过少，请至少指定左右影像路径！" << std::endl;
		return -1;
	}

	printf("Image Loading...");
	//···············································································//
	// 读取影像
	std::string path_left = argc[1];
	std::string path_right = argc[2];

	cv::Mat img_left = cv::imread(path_left, cv::IMREAD_COLOR);
	cv::Mat img_right = cv::imread(path_right, cv::IMREAD_COLOR);

	if (img_left.data == nullptr || img_right.data == nullptr) {
		std::cout << "读取影像失败！" << std::endl;
		return -1;
	}
	if (img_left.rows != img_right.rows || img_left.cols != img_right.cols) {
		std::cout << "左右影像尺寸不一致！" << std::endl;
		return -1;
	}


	//···············································································//
	const sint32 width = static_cast<uint32>(img_left.cols);
	const sint32 height = static_cast<uint32>(img_right.rows);

	// 左右影像的彩色数据,将输入的彩色图像转换为一维数组
	auto bytes_left = new uint8[width * height * 3];
	auto bytes_right = new uint8[width * height * 3];
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			bytes_left[i * 3 * width + 3 * j] = img_left.at<cv::Vec3b>(i, j)[0];
			bytes_left[i * 3 * width + 3 * j + 1] = img_left.at<cv::Vec3b>(i, j)[1];
			bytes_left[i * 3 * width + 3 * j + 2] = img_left.at<cv::Vec3b>(i, j)[2];
			bytes_right[i * 3 * width + 3 * j] = img_right.at<cv::Vec3b>(i, j)[0];
			bytes_right[i * 3 * width + 3 * j + 1] = img_right.at<cv::Vec3b>(i, j)[1];
			bytes_right[i * 3 * width + 3 * j + 2] = img_right.at<cv::Vec3b>(i, j)[2];
		}
	}
	printf("Done!\n");

	// AD-Census匹配参数设计
	ADCensusOption ad_option;
	// 候选视差范围
	ad_option.min_disparity = argv < 4 ? 0 : atoi(argc[3]);
	ad_option.max_disparity = argv < 5 ? 64 : atoi(argc[4]);
	// 一致性检查阈值
	ad_option.lrcheck_thres = 1.0f;

	// 是否执行一致性检查
	ad_option.do_lr_check = true;

	// 是否执行视差填充
	// 视差图填充的结果并不可靠，若工程，不建议填充，若科研，则可填充
	ad_option.do_filling = true;
	
	printf("w = %d, h = %d, d = [%d,%d]\n\n", width, height, ad_option.min_disparity, ad_option.max_disparity);

	// 定义AD-Census匹配类实例
	ADCensusStereo ad_census;

	printf("AD-Census Initializing...\n");
	auto start = steady_clock::now();
	//···············································································//
	// 初始化
	if (!ad_census.Initialize(width, height, ad_option)) {
		std::cout << "AD-Census初始化失败！" << std::endl;
		return -2;
	}
	auto end = steady_clock::now();
	auto tt = duration_cast<milliseconds>(end - start);
	printf("AD-Census Initializing Done! Timing :	%lf s\n\n", tt.count() / 1000.0);

	printf("AD-Census Matching...\n");
	// disparity数组保存子像素的视差结果
	auto disparity = new float32[uint32(width * height)]();

	start = steady_clock::now();
	//···············································································//
	// 匹配
	if (!ad_census.Match(bytes_left, bytes_right, disparity)) {
		std::cout << "AD-Census匹配失败！" << std::endl;
		return -2;
	}
	end = steady_clock::now();
	tt = duration_cast<milliseconds>(end - start);
	printf("\nAD-Census Matching...Done! Timing :	%lf s\n", tt.count() / 1000.0);

	//···············································································//
	// 显示视差图
	ShowDisparityMap(disparity, width, height, "disp-left");
	// 保存视差图
	SaveDisparityMap(disparity, width, height, path_left);

	cv::waitKey(0);

	//···············································································//
	// 释放内存
	delete[] disparity;
	disparity = nullptr;
	delete[] bytes_left;
	bytes_left = nullptr;
	delete[] bytes_right;
	bytes_right = nullptr;
    return 0;
}
