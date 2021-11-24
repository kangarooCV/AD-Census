//
// Created by Zhi-E on 2021/11/23.
//

#ifndef CLION_UTILS_H
#define CLION_UTILS_H
#include "adcensus_types.h"
#include <opencv2/opencv.hpp>

/*显示视差图*/
void ShowDisparityMap(const float32* disp_map, const sint32& width, const sint32& height, const std::string& name);
/*保存视差图*/
void SaveDisparityMap(const float32* disp_map, const sint32& width, const sint32& height, const std::string& path);
/*保存视差点云*/
void SaveDisparityCloud(const uint8* img_bytes, const float32* disp_map, const sint32& width, const sint32& height, const std::string& path);


#endif //CLION_UTILS_H
