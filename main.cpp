#include <iostream>
#include "ADCensusStereo.h"
#include <chrono>
#include "utils.h"
using namespace std::chrono;	// ʱ���

// opencv library
#include <opencv2/opencv.hpp>

/**
* \brief
* \param argv 3
* \param argc argc[1]:��Ӱ��·�� argc[2]: ��Ӱ��·�� argc[3]: ��С�Ӳ�[��ѡ��Ĭ��0] argc[4]: ����Ӳ�[��ѡ��Ĭ��64]
* \param eg. ..\Data\cone\im2.png ..\Data\cone\im6.png 0 64
* \param eg. ..\Data\Cloth3\view1.png ..\Data\Cloth3\view5.png 0 128
* \return
*/
int main(int argv, char** argc)
{
	if (argv < 3) {
		std::cout << "�������٣�������ָ������Ӱ��·����" << std::endl;
		return -1;
	}

	printf("Image Loading...");
	//��������������������������������������������������������������������������������������������������������������������������������������������������������������//
	// ��ȡӰ��
	std::string path_left = argc[1];
	std::string path_right = argc[2];

	cv::Mat img_left = cv::imread(path_left, cv::IMREAD_COLOR);
	cv::Mat img_right = cv::imread(path_right, cv::IMREAD_COLOR);

	if (img_left.data == nullptr || img_right.data == nullptr) {
		std::cout << "��ȡӰ��ʧ�ܣ�" << std::endl;
		return -1;
	}
	if (img_left.rows != img_right.rows || img_left.cols != img_right.cols) {
		std::cout << "����Ӱ��ߴ粻һ�£�" << std::endl;
		return -1;
	}


	//��������������������������������������������������������������������������������������������������������������������������������������������������������������//
	const sint32 width = static_cast<uint32>(img_left.cols);
	const sint32 height = static_cast<uint32>(img_right.rows);

	// ����Ӱ��Ĳ�ɫ����,������Ĳ�ɫͼ��ת��Ϊһά����
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

	// AD-Censusƥ��������
	ADCensusOption ad_option;
	// ��ѡ�ӲΧ
	ad_option.min_disparity = argv < 4 ? 0 : atoi(argc[3]);
	ad_option.max_disparity = argv < 5 ? 64 : atoi(argc[4]);
	// һ���Լ����ֵ
	ad_option.lrcheck_thres = 1.0f;

	// �Ƿ�ִ��һ���Լ��
	ad_option.do_lr_check = true;

	// �Ƿ�ִ���Ӳ����
	// �Ӳ�ͼ���Ľ�������ɿ��������̣���������䣬�����У�������
	ad_option.do_filling = true;
	
	printf("w = %d, h = %d, d = [%d,%d]\n\n", width, height, ad_option.min_disparity, ad_option.max_disparity);

	// ����AD-Censusƥ����ʵ��
	ADCensusStereo ad_census;

	printf("AD-Census Initializing...\n");
	auto start = steady_clock::now();
	//��������������������������������������������������������������������������������������������������������������������������������������������������������������//
	// ��ʼ��
	if (!ad_census.Initialize(width, height, ad_option)) {
		std::cout << "AD-Census��ʼ��ʧ�ܣ�" << std::endl;
		return -2;
	}
	auto end = steady_clock::now();
	auto tt = duration_cast<milliseconds>(end - start);
	printf("AD-Census Initializing Done! Timing :	%lf s\n\n", tt.count() / 1000.0);

	printf("AD-Census Matching...\n");
	// disparity���鱣�������ص��Ӳ���
	auto disparity = new float32[uint32(width * height)]();

	start = steady_clock::now();
	//��������������������������������������������������������������������������������������������������������������������������������������������������������������//
	// ƥ��
	if (!ad_census.Match(bytes_left, bytes_right, disparity)) {
		std::cout << "AD-Censusƥ��ʧ�ܣ�" << std::endl;
		return -2;
	}
	end = steady_clock::now();
	tt = duration_cast<milliseconds>(end - start);
	printf("\nAD-Census Matching...Done! Timing :	%lf s\n", tt.count() / 1000.0);

	//��������������������������������������������������������������������������������������������������������������������������������������������������������������//
	// ��ʾ�Ӳ�ͼ
	ShowDisparityMap(disparity, width, height, "disp-left");
	// �����Ӳ�ͼ
	SaveDisparityMap(disparity, width, height, path_left);

	cv::waitKey(0);

	//��������������������������������������������������������������������������������������������������������������������������������������������������������������//
	// �ͷ��ڴ�
	delete[] disparity;
	disparity = nullptr;
	delete[] bytes_left;
	bytes_left = nullptr;
	delete[] bytes_right;
	bytes_right = nullptr;
    return 0;
}
