// 保留的原版室外巡线，仅由 lane_backend=outdoor 分支调用。
// 流程为裁剪缩放、亮度处理、灰度边缘、霍夫筛线、左右拟合和逐行加权误差。
// 本文件的原有表达式与标定保持原样；新版室外算法在 lane_outdoor_v2.hpp。
#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
namespace original_lane {
using namespace std;using namespace cv;
constexpr double gamma=1.;
Mat cropped_image,lane_mask;int lane_left_count=0,lane_right_count=0;
#define LINE_DISTANCE_MIN 50
#define LINE_ANGLE_MIN 200

// Unchanged active original lane image-processing path.
// 根据灰度均值计算局部 gamma，并用 256 项查找表处理输入的三个颜色通道。
// 函数内的 gamma 是本次计算值，与命名空间中的同名常量作用域不同。
void gammaCorrection(const Mat& input, Mat& output) {
    CV_Assert(gamma > 0);
    cv::Mat gray_image;
    cv::cvtColor(input, gray_image, cv::COLOR_BGR2GRAY);
    double mean_intensity = cv::mean(gray_image)[0];
    double gamma = 0.5 + mean_intensity / 80.0;
    if (gamma > 2) gamma = 2;
    else if (gamma < 0.5) gamma = 0.5;
    Mat lookupTable(1, 256, CV_8U);
    for (int i = 0; i < 256; i++) {
        lookupTable.at<uchar>(i) = saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }
    LUT(input, lookupTable, output);
}
// 把 y=slope*x+b 延伸到图像矩形边缘，返回落在边界范围内的交点。
// 后续用交点在 320×96 掩码上画出延长线或拟合边界。
vector<cv::Point> extendLine(const cv::Point pt, double slope, double b, const cv::Size imageSize) {
    double x_intersect, y_intersect;
    vector<cv::Point> pts;
    x_intersect = (0 - b) / slope;
    if (x_intersect >= 0 && x_intersect < imageSize.width) {
        pts.push_back( cv::Point(x_intersect, 0) );
    }
    x_intersect = (imageSize.height - b) / slope;
    if (x_intersect >= 0 && x_intersect < imageSize.width) {
        pts.push_back( cv::Point(x_intersect, imageSize.height) );
    }
    y_intersect = slope * 0 + b;
    if (y_intersect >= 0 && y_intersect < imageSize.height) {
        pts.push_back( cv::Point(0, y_intersect) );
    }
    y_intersect = slope * imageSize.width + b;
    if (y_intersect >= 0 && y_intersect < imageSize.height) {
        pts.push_back( cv::Point(imageSize.width, y_intersect) );
    }
    if (pts.size()) return pts;
    return { };
}
// 96 行的固定权重表：中间区域权重大、上下两端权重小，供误差加权平均使用。
int Weight_th[96] = {
    1, 1, 1, 1, 1, 1,
    2, 2, 2, 2, 2, 2,
    3, 3, 3, 3, 3, 3,
    4, 4, 4, 4, 4, 4,
    5, 5, 5, 5, 5, 5,
    6, 6, 6, 6, 6, 6,
    7, 7, 7, 7, 7, 7,
    8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8,
    7, 7, 7, 7, 7, 7,
    6, 6, 6, 6, 6, 6,
    5, 5, 5, 5, 5, 5,
    4, 4, 4, 4, 4, 4,
    3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 2, 2,
    1, 1, 1, 1, 1, 1  };
int mid_line = 160, left_up_line = 0, right_up_line = 0, up_line_count = 0;
// 从上沿左右边界确定横向搜索起点，然后逐行向两边找白色边界。
// 找不到左/右边界时分别使用 x=0 和最后一列，不估计上一帧车道宽度。
double image_handle2(const Mat data)
{
    if (!right_up_line) right_up_line = 319;
    mid_line = left_up_line + right_up_line >> 1;
    int weight_count = 0;
    double error_in = 0, error_out = 0;
    int co = data.cols, ro = data.rows;
	std::vector<int> left(ro, -1);
	std::vector<int> right(ro, -1);
	std::vector<int> mid(ro, -1);
	int mid_sum = 0;
	int l_lost = 0;
	int r_lost = 0;
	int t = 0;
#ifdef DRAW_MID
    Mat data_cpy = data.clone();
#endif
	// 跳过上下各约 10 行，每隔 2 行采样；left/right 数组记录本次扫描得到的边界。
	for (int i = ro - 1 - 10; i >= 10 ; i -= 2) {
		l_lost = 0;
		r_lost = 0;
		for (int j = mid_line; j >= 0; j --) {
			if (data.at<uchar>(i, j) == 255) {
				left[t] = j;
				l_lost = 1;
				break;
			}
		}
		if (l_lost == 0) left[t] = 0;
		for (int j = mid_line; j < co ; j ++) {
			if (data.at<uchar>(i, j) == 255 ) {
				right[t] = j;
				r_lost = 1;
				break;
			}
		}
		if (r_lost == 0) right[t] = co - 1;
#ifdef DRAW_MID
		Point p((left[t] + right[t]) / 2, i);
        circle(data_cpy, p, 2, Scalar(255, 255, 255));
#endif
		mid[t] = (left[t] + right[t]) / 2;
        // 原表达式以 180 为参考加权累积，限幅后除以 4；外层 TUxiang_Init3 还会加 5。
        // 因此理解最终转向误差时需要连同外层偏置一起看，不能只把这里的 180 当作最终中心。
        error_in += (mid[t] - 180) * Weight_th[i];
        weight_count += Weight_th[i];
		t ++;
    }
    error_out = error_in / weight_count;
    if (error_out < -160) error_out = -160;
    if (error_out > 160) error_out = 160;
    error_out /= 4;
#ifdef DRAW_MID
    imshow("find mid", data_cpy);
#endif
    return error_out;
}
// 640×480 输入裁取从 y=210 开始、高 192 的区域，再缩小为 320×96。
// 后续所有霍夫距离、拟合与逐行搜索都在这个缩小后的坐标系中进行。
double TUxiang_Init3(cv::Mat& data)
{
    cv::Rect roi_rect(0, (data.rows / 2 - 90 + 60), data.cols, (data.rows / 2.5));
    cropped_image = data(roi_rect);
    cv::resize(cropped_image, cropped_image, cv::Size(), 0.5, 0.5);
    gammaCorrection(cropped_image, cropped_image);
    // 保留原来的饱和度处理分支；真正送入下方霍夫变换的是灰度分支产生的 ca。
    // CLAHE 对象虽然设置了参数，但本函数没有调用 apply，因此不能把它算作生效步骤。
    cv::Mat hsv_image;
    cv::cvtColor(cropped_image, hsv_image, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv_image, hsv_channels);
    cv::Mat saturation = hsv_channels[1];
    Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(1.5);
    clahe->setTilesGridSize(Size(8, 8));
    cv::Mat blur1;
    cv::bilateralFilter(saturation, blur1, 7, 60, 60);
    cv::Mat gaussian_blur1;
    cv::GaussianBlur(blur1, gaussian_blur1, cv::Size(5, 5), 30);
    cv::Mat edges;
    cv::Canny(gaussian_blur1, edges, 50, 90);
    cv::Mat ca;
    if (1) {
        cv::Mat gray_image;
        cv::cvtColor(cropped_image, gray_image, cv::COLOR_BGR2GRAY);
        cv::Mat blur;
        cv::bilateralFilter(gray_image, blur, 7, 60, 60);
        cv::Mat gaussian_blur;
        cv::GaussianBlur(blur, gaussian_blur, cv::Size(5, 5), 30);
        cv::Canny(gaussian_blur, ca, 30, 50);
#ifdef LINE_DEBUG
        imshow("gray_image", gray_image);
#endif
    }
    // 灰度 Canny 边缘膨胀后提取线段；左右候选先按负/正角度区间分开。
    // LINE_ANGLE_MIN 保留为 200 度，实际筛选效果需按此数值理解。
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::Mat dilated_ca;
    cv::dilate(ca, dilated_ca, kernel, cv::Point(-1, -1), 2);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(dilated_ca, lines, 1, CV_PI / 180, 50, 25, 10);
    cv::Mat line_image = cv::Mat::zeros(dilated_ca.size(), CV_8UC1);
    cv::Mat line_image2 = cv::Mat::zeros(dilated_ca.size(), CV_8UC1);
    cv::Mat dilated_ca2 = cv::Mat::zeros(dilated_ca.size(), CV_8UC1), dilated_caca = cv::Mat::zeros(dilated_ca.size(), CV_8UC1);
    std::vector<cv::Vec4i> left_lines, right_lines, left_lines_mid, right_lines_mid;
    double left_min_angle = -90;
    double left_max_angle = -18;
    double right_min_angle = 18;
    double right_max_angle = 90;
    double left_most = 1000, right_most = 0;
    double left_angle = -1000, right_angle = 0;
    for (size_t i = 0; i < lines.size(); i ++)
    {
        cv::Vec4i line = lines[i];
        double angle_rad = atan2(line[3] - line[1], line[2] - line[0]);
        double angle_deg = angle_rad * 180 / CV_PI;
        if (angle_deg >= left_min_angle && angle_deg <= left_max_angle) {
            left_angle = max(left_angle, angle_deg);
        }
        if (angle_deg >= right_min_angle && angle_deg <= right_max_angle) {
            right_angle = max(right_angle, angle_deg);
        }
    }
    for (size_t i = 0; i < lines.size(); i ++)
    {
        cv::Vec4i line = lines[i];
        double angle_rad = atan2(line[3] - line[1], line[2] - line[0]);
        double angle_deg = angle_rad * 180 / CV_PI;
        if (angle_deg >= left_min_angle && angle_deg <= left_max_angle) {
            if (abs(angle_deg - left_angle) < LINE_ANGLE_MIN) {
                left_lines_mid.push_back(line);
                left_most = cv::min(left_most, (line[0] + line[2]) / 2.0);
            }
        }
    }
    // 左侧进一步按最左候选的横向距离选段，保存端点用于 Huber 拟合。
    // right_up_line/left_up_line 记录拟合线与掩码上沿交点，供后面的搜索中线使用。
    vector<cv::Point> fits_points_l, fits_points_r;
    left_up_line = 0, right_up_line = 0, up_line_count = 0;
    for (size_t i = 0; i < left_lines_mid.size(); i ++)
    {
        cv::Vec4i line = left_lines_mid[i];
        double angle_rad = atan2(line[3] - line[1], line[2] - line[0]);
        double angle_deg = angle_rad * 180 / CV_PI;
        if (abs(left_most - (line[0] + line[2]) / 2) < LINE_DISTANCE_MIN)  {
            double slope = tan(angle_rad);
            double b = line[1] - slope * line[0];
            vector<cv::Point> pts = extendLine(cv::Point(line[0], line[1]), slope, b, cv::Size(320, 96));
            if (pts.size() >= 2) {
                if (pts[0].x + pts[1].x >> 1 >= 270) continue;
                left_lines.push_back(cv::Vec4i(pts[0].x, pts[0].y, pts[1].x, pts[1].y));
            }
            fits_points_l.emplace_back(line[0], line[1]);
            fits_points_l.emplace_back(line[2], line[3]);
        }
    }
    for (size_t i = 0; i < left_lines.size(); i++)
    {
        const cv::Vec4i& line = left_lines[i];
        const cv::Point pt1(line[0], line[1]);
        const cv::Point pt2(line[2], line[3]);
        cv::line(line_image, pt1, pt2, cv::Scalar(255), 2, cv::LINE_AA);
    }
    // 左侧有候选时拟合成一条延长边界，并写入 line_image2；右侧稍后采用对应流程。
    if (left_lines.size()) {
        cv::Vec4f line_params;
        cv::fitLine(fits_points_l, line_params, cv::DIST_HUBER, 10, 10, 1);
        double vx = line_params[0];
        double vy = line_params[1];
        double x0 = line_params[2];
        double y0 = line_params[3];
        double m = vy / vx;
        double b2 = y0 - m * x0;
        vector<cv::Point> pts2 = extendLine(cv::Point(x0, y0), m, b2, cv::Size(320, 96));
        if (pts2[0].y == 0) left_up_line = pts2[0].x;
        else if (pts2[1].y == 0) left_up_line = pts2[1].x;
            cv::line(line_image2, pts2[0], pts2[1], cv::Scalar(255), 2, cv::LINE_AA);
    };
    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i line = lines[i];
        double angle_rad = atan2(line[3] - line[1], line[2] - line[0]);
        double angle_deg = angle_rad * 180 / CV_PI;
        if (angle_deg >= right_min_angle && angle_deg <= right_max_angle) {
            if (abs(angle_deg - right_angle) < LINE_ANGLE_MIN) {
                // 右侧候选按最右位置聚集；与左侧对称，但保留原有角度及位置判断表达式。
                right_lines_mid.push_back(line);
                right_most = cv::max(right_most, (line[0] + line[2]) / 2.0);
            }
        }
    }
    for (size_t i = 0; i < right_lines_mid.size(); i++)
    {
        cv::Vec4i line = right_lines_mid[i];
        double angle_rad = atan2(line[3] - line[1], line[2] - line[0]);
        double angle_deg = angle_rad * 180 / CV_PI;
        if (abs(right_most - (line[0] + line[2]) / 2) < LINE_DISTANCE_MIN) {
            double slope = tan(angle_rad);
            double b = line[1] - slope * line[0];
            vector<cv::Point> pts = extendLine(cv::Point(line[0], line[1]), slope, b, cv::Size(320, 96));
            if (pts.size() >= 2) {
                if (pts[0].x + pts[1].x >> 1 <= 90) continue;
                right_lines.push_back(cv::Vec4i(pts[0].x, pts[0].y, pts[1].x, pts[1].y));
            }
            fits_points_r.emplace_back(line[0], line[1]);
            fits_points_r.emplace_back(line[2], line[3]);
        }
    }
    for (size_t i = 0; i < right_lines.size(); i++)
    {
        const cv::Vec4i& line = right_lines[i];
        cv::Point pt1(line[0], line[1]);
        cv::Point pt2(line[2], line[3]);
        cv::line(line_image, pt1, pt2, cv::Scalar(255), 2, cv::LINE_AA);
    }
    if (right_lines.size()) {
        cv::Vec4f line_params;
        cv::fitLine(fits_points_r, line_params, cv::DIST_HUBER, 10, 10, 1);
        double vx = line_params[0];
        double vy = line_params[1];
        double x0 = line_params[2];
        double y0 = line_params[3];
        double m = vy / vx;
        double b2 = y0 - m * x0;
        vector<cv::Point> pts2 = extendLine(cv::Point(x0, y0), m, b2, cv::Size(320, 96));
        if (pts2[0].y == 0) right_up_line = pts2[0].x;
        else if (pts2[1].y == 0) right_up_line = pts2[1].x;
            cv::line(line_image2, pts2[0], pts2[1], cv::Scalar(255), 2, cv::LINE_AA);
    }
    // 最终控制使用拟合边界的膨胀图 dilated_caca；lane_mask 是它的副本。
    // 左右计数记录通过筛选的候选段数，供集成层判断真实边界是否存在。
    cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::dilate(line_image, dilated_ca2, kernel2, cv::Point(-1, -1), 1);
    cv::dilate(line_image2, dilated_caca, kernel2, cv::Point(-1, -1), 1);
    lane_left_count=left_lines.size();lane_right_count=right_lines.size();
    lane_mask=dilated_caca.clone();
    // 保留原版误差末端的 +5 标定偏置；不在注释工作中调整转向中心或计算公式。
    return image_handle2(dilated_caca)+5;
}

}
