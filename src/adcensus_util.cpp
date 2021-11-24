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

	// �����ؼ���censusֵ
	for (sint32 i = 4; i < height - 4; i++) {
		for (sint32 j = 3; j < width - 3; j++) {

			// ��������ֵ
			const uint8 gray_center = source[i * width + j];
			// ������СΪ9x7�Ĵ������������أ���һ�Ƚ�����ֵ����������ֵ�ĵĴ�С������censusֵ
            // ��һ��64λ���������洢һ�����ڵ�������ÿһλ��ʾ������������صĴ�С
			uint64 census_val = 0u;
			for (sint32 r = -4; r <= 4; r++) {
				for (sint32 c = -3; c <= 3; c++) {
                    // ÿ�ζ�����һλ��Ҳ���ǽ����λ���洢����������صĽ��
					census_val <<= 1;
					const uint8 gray = source[(i + r) * width + j + c];
					if (gray < gray_center) {
                        // ����������ش����������أ������λ��Ϊ1
						census_val += 1;
					}
				}
			}
			// �������ص�censusֵ
			census[i * width + j] = census_val;
		}
	}
}


uint8 adcensus_util::Hamming64(const uint64& x, const uint64& y)
{
    // ��λ��򣬲�ͬ��λ��Ϊ1
	uint64 dist = 0, val = x ^ y;

	// ͳ���ж��ٸ�λΪ1
	while (val) {
		++dist;
        // ÿ�ζ�����ĩβ��1��Ϊ0
		val &= val - 1;
	}

	return static_cast<uint8>(dist);
}

// ��ֵ�˲�
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
					if (row >= 0 && row < height && col >= 0 && col < width) {	// �ж��������Ч��
						wnd_data.push_back(in[row * width + col]);
					}
				}
			}
			// ȡ����ֵ
			std::sort(wnd_data.begin(), wnd_data.end());
			if (!wnd_data.empty()) {
				out[y * width + x] = wnd_data[wnd_data.size() / 2];
			}
		}
	}
}