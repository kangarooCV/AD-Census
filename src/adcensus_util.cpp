/* -*-c++-*- AD-Census - Copyright (C) 2020.
* Author	: Yingsong Li(Ethan Li) <ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/AD-Census
* Describe	: implement of adcensus_util
*/

#include "adcensus_util.h"
#include <cassert>

void adcensus_util::census_transform_9x7(const uint8* source, vector<uint64>& census, const sint32& width, const sint32& height)
{
	if (source == nullptr || census.empty() || width <= 9 || height <= 7) {
		return;
	}

	// 逐像素计算census值
	for (sint32 i = 4; i < height - 4; i++) {
		for (sint32 j = 3; j < width - 3; j++) {

			// 中心像素值
			const uint8 gray_center = source[i * width + j];
			// 遍历大小为9x7的窗口内邻域像素，逐一比较像素值与中心像素值的的大小，计算census值
            // 用一个64位的数字来存储一个窗口的向量，每一位表示邻域和中心像素的大小
			uint64 census_val = 0u;
			for (sint32 r = -4; r <= 4; r++) {
				for (sint32 c = -3; c <= 3; c++) {
                    // 每次都左移一位，也就是将最低位来存储这个邻域像素的结果
					census_val <<= 1;
					const uint8 gray = source[(i + r) * width + j + c];
					if (gray < gray_center) {
                        // 如果中心像素大于邻域像素，将最低位置为1
						census_val += 1;
					}
				}
			}
			// 中心像素的census值
			census[i * width + j] = census_val;
		}
	}
}


uint8 adcensus_util::Hamming64(const uint64& x, const uint64& y)
{
    // 按位亦或，不同的位置为1
	uint64 dist = 0, val = x ^ y;

	// 统计有多少个位为1
	while (val) {
		++dist;
        // 每次都将最末尾的1置为0
		val &= val - 1;
	}

	return static_cast<uint8>(dist);
}

// 中值滤波
void adcensus_util::MedianFilter(const float32* in, float32* out, const sint32& width, const sint32& height, const sint32 wnd_size)
{
	const sint32 radius = wnd_size / 2;
	const sint32 size = wnd_size * wnd_size;
	
	std::vector<float32> wnd_data;
	wnd_data.reserve(size);

	for (sint32 y = 0; y < height; y++) {
		for (sint32 x = 0; x < width; x++) {
			wnd_data.clear();
			for (sint32 r = -radius; r <= radius; r++) {
				for (sint32 c = -radius; c <= radius; c++) {
					const sint32 row = y + r;
					const sint32 col = x + c;
					if (row >= 0 && row < height && col >= 0 && col < width) {	// 判断坐标的有效性
						wnd_data.push_back(in[row * width + col]);
					}
				}
			}
			// 取出中值
			std::sort(wnd_data.begin(), wnd_data.end());
			if (!wnd_data.empty()) {
				out[y * width + x] = wnd_data[wnd_data.size() / 2];
			}
		}
	}
}