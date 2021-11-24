/* -*-c++-*- AD-Census - Copyright (C) 2020.
* Author	: Yingsong Li(Ethan Li) <ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/AD-Census
* Describe	: implement of class CrossAggregator
*/

#include "cross_aggregator.h"
#include <cstring>

CrossAggregator::CrossAggregator(): width_(0), height_(0), img_left_(nullptr), img_right_(nullptr),
                                    cost_init_(nullptr),
                                    cross_L1_(0), cross_L2_(0), cross_t1_(0), cross_t2_(0),
                                    min_disparity_(0), max_disparity_(0), is_initialized_(false) { }

CrossAggregator::~CrossAggregator()
{
	
}

bool CrossAggregator::Initialize(const sint32& width, const sint32& height, const sint32& min_disparity, const sint32& max_disparity)
{
	width_ = width;
	height_ = height;
	min_disparity_ = min_disparity;
	max_disparity_ = max_disparity;
	
	const sint32 img_size = width_ * height_;
	const sint32 disp_range = max_disparity_ - min_disparity_;
	if (img_size <= 0 || disp_range <= 0) {
		is_initialized_ = false;
		return is_initialized_;
	}

	// Ϊ����ʮ�ֱ���������ڴ�
	vec_cross_arms_.clear();
	vec_cross_arms_.resize(img_size);

	// Ϊ��ʱ������������ڴ�
	vec_cost_tmp_[0].clear();
	vec_cost_tmp_[0].resize(img_size);
	vec_cost_tmp_[1].clear();
	vec_cost_tmp_[1].resize(img_size);

	// Ϊ�洢ÿ������֧����������������������ڴ�
	vec_sup_count_[0].clear();
	vec_sup_count_[0].resize(img_size);
	vec_sup_count_[1].clear();
	vec_sup_count_[1].resize(img_size);
	vec_sup_count_tmp_.clear();
	vec_sup_count_tmp_.resize(img_size);

	// Ϊ�ۺϴ�����������ڴ�
	cost_aggr_.resize(img_size * disp_range);

	is_initialized_ = !vec_cross_arms_.empty() && !vec_cost_tmp_[0].empty() && !vec_cost_tmp_[1].empty() 
					&& !vec_sup_count_[0].empty() && !vec_sup_count_[1].empty() 
					&& !vec_sup_count_tmp_.empty() && !cost_aggr_.empty();
	return is_initialized_;
}

void CrossAggregator::SetData(const uint8* img_left, const uint8* img_right, const float32* cost_init)
{
	img_left_ = img_left;
	img_right_ = img_right;
	cost_init_ = cost_init;
}

void CrossAggregator::SetParams(const sint32& cross_L1, const sint32& cross_L2, const sint32& cross_t1,
	const sint32& cross_t2)
{
	cross_L1_ = cross_L1;
	cross_L2_ = cross_L2;
	cross_t1_ = cross_t1;
	cross_t2_ = cross_t2;
}

void CrossAggregator::BuildArms() 
{
	// �����ؼ���ʮ�ֽ����
	for (sint32 y = 0; y < height_; y++) {
		for (sint32 x = 0; x < width_; x++) {
			CrossArm& arm = vec_cross_arms_[y * width_ + x];
			FindHorizontalArm(x, y, arm.left, arm.right);
			FindVerticalArm(x, y, arm.top, arm.bottom);
		}
	}
}


void CrossAggregator::Aggregate(const sint32& num_iters)
{
	if (!is_initialized_) {
		return;
	}

	const sint32 disp_range = max_disparity_ - min_disparity_;

	// �������ص�ʮ�ֽ����
	BuildArms();

	// ���۾ۺ�

	// �������־ۺϷ���ĸ�����֧������������
	ComputeSupPixelCount();

	// �Ƚ��ۺϴ��۳�ʼ��Ϊ��ʼ����
	memcpy(&cost_aggr_[0], cost_init_, width_*height_*disp_range*sizeof(float32));

	// ������ۺ�
    // horizontal_first ������ˮƽ����ۺ�
    bool horizontal_first = true;
	for (sint32 k = 0; k < num_iters; k++) {
        // ��ÿ��dά���϶�����֧������۾ۺ�
		for (sint32 d = min_disparity_; d < max_disparity_; d++) {
			AggregateInArms(d, horizontal_first);
		}
		// ��һ�ε���������˳��
		horizontal_first = !horizontal_first;
	}
}

CrossArm* CrossAggregator::get_arms_ptr()
{
	return &vec_cross_arms_[0];
}

float32* CrossAggregator::get_cost_ptr()
{
	if (!cost_aggr_.empty()) {
		return &cost_aggr_[0];
	}
	else {
		return nullptr;
	}
}

void CrossAggregator::FindHorizontalArm(const sint32& x, const sint32& y, uint8& left, uint8& right) const
{
	// �������ݵ�ַ
	const auto img0 = img_left_ + y * width_ * 3 + 3 * x;
	// ������ɫֵ
	const ADColor color0(img0[0], img0[1], img0[2]);
	
	left = right = 0;
	//�������ұ�,����ۺ��ұ�
	sint32 dir;
	for (sint32 k = 0; k < 2; k++) {
        k==0? dir=-1 : dir=1;
		// �����ֱ������������
		// �۳����ó���cross_L1
		auto img = img0 + dir * 3;
		auto color_last = color0;
		sint32 xn = x + dir;
		for (sint32 n = 0; n < std::min(cross_L1_, MAX_ARM_LENGTH); n++) {

			// �߽紦��
			if ((k==0&&xn<0)||(k==1&&xn==width_)) break;

			// ��ȡ��ɫֵ
			const ADColor color(img[0], img[1], img[2]);

			// ��ɫ����1���������غͼ������ص���ɫ���룩
			const sint32 color_dist1 = ColorDist(color, color0);
			if (color_dist1 >= cross_t1_) break;


			// ��ɫ����2���������غ�ǰһ�����ص���ɫ���룩
			if (n > 0) {
				const sint32 color_dist2 = ColorDist(color, color_last);
				if (color_dist2 >= cross_t1_) {
					break;
				}
			}

			// �۳�����L2����ɫ������ֵ��СΪt2
			if (n + 1 > cross_L2_) {
				if (color_dist1 >= cross_t2_) {
					break;
				}
			}

            k==0 ? left++ : right++;
            // ��¼��һ�������ɫ
			color_last = color;
            // �����ж���һ����
			xn += dir;
            // ͼ��ָ���Ƶ���һ�����صĵ�ַ
			img += dir * 3;
		}
	}
}

void CrossAggregator::FindVerticalArm(const sint32& x, const sint32& y, uint8& top, uint8& bottom) const
{
	// �������ݵ�ַ
	const auto img0 = img_left_ + y * width_ * 3 + 3 * x;
	// ������ɫֵ
	const ADColor color0(img0[0], img0[1], img0[2]);

	top = bottom = 0;
	//�������±�,���ϱۺ��±�
	sint32 dir = -1;
	for (sint32 k = 0; k < 2; k++) {
		// �����ֱ������������
		// �۳����ó���cross_L1
		auto img = img0 + dir * width_ * 3;
		auto color_last = color0;
		sint32 yn = y + dir;
		for (sint32 n = 0; n < std::min(cross_L1_, MAX_ARM_LENGTH); n++) {

			// �߽紦��
			if (k == 0) {
				if (yn < 0) {
					break;
				}
			}
			else {
				if (yn == height_) {
					break;
				}
			}

			// ��ȡ��ɫֵ
			const ADColor color(img[0], img[1], img[2]);

			// ��ɫ����1���������غͼ������ص���ɫ���룩
			const sint32 color_dist1 = ColorDist(color, color0);
			if (color_dist1 >= cross_t1_) {
				break;
			}

			// ��ɫ����2���������غ�ǰһ�����ص���ɫ���룩
			if (n > 0) {
				const sint32 color_dist2 = ColorDist(color, color_last);
				if (color_dist2 >= cross_t1_) {
					break;
				}
			}

			// �۳�����L2����ɫ������ֵ��СΪt2
			if (n + 1 > cross_L2_) {
				if (color_dist1 >= cross_t2_) {
					break;
				}
			}

			if (k == 0) {
				top++;
			}
			else {
				bottom++;
			}
			color_last = color;
			yn += dir;
			img += dir * width_ * 3;
		}
		dir = -dir;
	}
}

// �����Ⱥ�������ȼ���ȫͼ���صĺ�۳��ȴ浽��ʱ�����У�Ȼ��������ĵ���������꣬����ʱ������ȡ��ÿ����ĺ�۳�����ӵõ��ܵ����
void CrossAggregator::ComputeSupPixelCount()
{

	// ����ÿ�����ص�֧������������
	// ע�⣺���ֲ�ͬ�ľۺϷ������ص�֧���������ǲ�ͬ�ģ���Ҫ�ֿ�����

	for (sint32 n = 0; n < 2; n++) {
		// n=0 : horizontal_first; n=1 : vertical_first
        bool horizontal_first = (n == 0);
		const sint32 id = horizontal_first ? 0 : 1;
		for (sint32 k = 0; k < 2; k++) {
			// k=0 : pass1; k=1 : pass2
			for (sint32 y = 0; y < height_; y++) {
				for (sint32 x = 0; x < width_; x++) {
					// ��ȡarm��ֵ
					auto& arm = vec_cross_arms_[y*width_ + x];
					sint32 count = 0;
					if (horizontal_first) {
						if (k == 0) {
							// horizontal
                            // �ȼ���ȫͼ�����صĵ�һ������ĺ�۳���
                            count  = arm.left + arm.right +1;
                            // ����һ������ı۳��浽��ʱ������
                            vec_sup_count_tmp_[y*width_ + x] = count;
						}
						else {
							// vertical
                            sint32 y_min = std::max(0, y-arm.top);
                            sint32 y_max = std::min(height_, y + arm.bottom);
							for (sint32 y1 = y_min; y1 <= y_max; y1++) {
								count += vec_sup_count_tmp_[y1*width_ + x];
							}
                            // �����ڶ��������õ�һ�������������ı۳����ӾͿ��Եõ��ܵ����
                            vec_sup_count_[id][y*width_ + x] = count;
						}
					}
					else {
						if (k == 0) {
							// vertical
                            count  = arm.top + arm.bottom +1;
                            // ����һ������ı۳��浽��ʱ������
                            vec_sup_count_tmp_[y*width_ + x] = count;
						}
						else {
							// horizontal
                            sint32 x_min = std::max(0, x-arm.left);
                            sint32 x_max = std::min(width_, x+arm.right);
							for (sint32 x1 = x_min; x1 <= x_max; x1++) {
								count += vec_sup_count_tmp_[y*width_ + x1];
							}
                            // �����ڶ��������õ�һ�������������ı۳����ӾͿ��Եõ��ܵ����
                            vec_sup_count_[id][y*width_ + x] = count;
						}
					}
				}
			}
		}
	}
}


// ���۾ۺϣ�����֧������������۾ۺ�ֵ��Ȼ��ֵ����������
void CrossAggregator::AggregateInArms(const sint32& disparity, const bool& horizontal_first)
{
	// �˺����ۺ��������ص��Ӳ�Ϊdisparityʱ�Ĵ���
	if (disparity < min_disparity_ || disparity >= max_disparity_) {
		return;
	}

	const auto disp = disparity - min_disparity_;
	const sint32 disp_range = max_disparity_ - min_disparity_;
	if (disp_range <= 0) return;

	// ��disp��Ĵ��۴�����ʱ����vec_cost_tmp_[0]
	// �������Ա������ķ��ʸ����cost_aggr_,��߷���Ч��
	for (sint32 y = 0; y < height_; y++) {
		for (sint32 x = 0; x < width_; x++) {
			vec_cost_tmp_[0][y * width_ + x] = cost_aggr_[y * width_ * disp_range + x * disp_range + disp];
		}
	}

	// �����ؾۺ�
	const sint32 ct_id = horizontal_first ? 0 : 1;
	for (sint32 k = 0; k < 2; k++) {
		// k==0: ����1
		// k==1: ����2
		for (sint32 y = 0; y < height_; y++) {
			for (sint32 x = 0; x < width_; x++) {
				// ��ȡarm��ֵ
				auto& arm = vec_cross_arms_[y*width_ + x];

                sint32 x_min = std::max(0, x-arm.left);
                sint32 x_max = std::min(width_, x+arm.right);
                sint32 y_min = std::max(0, y-arm.top);
                sint32 y_max = std::min(height_, y + arm.bottom);

				// �ۺ�
				float32 cost = 0.0f;
                // ��ˮƽ�ٴ�ֱ
				if (horizontal_first) {
					if (k == 0) {
						// horizontal
                        // ��ˮƽ����ֱ
                        // ��һ�׶ξ���ˮƽ���̣�����ȫͼ�ĺ�۴���ֵ֮�ͣ��浽��ʱ������
						for (sint32 x1 =x_min; x1 <= x_max; x1++) {
							cost += vec_cost_tmp_[0][y * width_ + x1];
						}
                        vec_cost_tmp_[1][y*width_ + x] = cost;  // vec_cost_tmp_[1] �����洢ȫͼ��һ���μ���ı۴���ֵ
					} else if(k==1){
						// vertical
                        // �ڶ��׶��Ǳ������ĵ�����ۣ���ÿ����ĺ�۴���ֵ֮�����ۼӣ�������֧�������ظ���
						for (sint32 y1 = y_min; y1 <= y_max; y1++) {
							cost += vec_cost_tmp_[1][y1*width_ + x];
						}
                        cost_aggr_[y*width_*disp_range + x*disp_range + disp] = cost / vec_sup_count_[ct_id][y*width_ + x];
					}
				}
				else {  //����ֱ��ˮƽ
					if (k == 0) {   // ��һ�׶�
						// vertical
						for (sint32 y1 = y_min; y1 <= y_max; y1++) {
							cost += vec_cost_tmp_[0][y1 * width_ + x];
						}
                        //�ڵ�һ�׶μ���ȫͼ�ĵ��۴���֮�ͣ��ŵ���ʱ������
                        vec_cost_tmp_[1][y*width_ + x] = cost;
					} else if(k==1) {   // �ڶ��׶�
						// horizontal
						for (sint32 x1 = x_min; x1 <= x_max; x1++) {
							cost += vec_cost_tmp_[1][y*width_ + x1];
						}
                        // �ڶ��׶Σ�������һ������������ۼӺ󣬴浽���յĽ����
                        cost_aggr_[y*width_*disp_range + x*disp_range + disp] = cost / vec_sup_count_[ct_id][y*width_ + x];
					}
				}
			}
		}
	}
}