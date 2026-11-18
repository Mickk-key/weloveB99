#ifndef TRAFFIC_LIGHT_DETECTOR_H
#define TRAFFIC_LIGHT_DETECTOR_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

enum class TrafficLightState { Unknown, Red, Green };

struct TrafficLightConfig {
    int confirmFrames = 2;
    int brightValue = 230;
    double roiBottom = 0.55;

    void validate() const {
        if (confirmFrames < 1 || confirmFrames > 100 || brightValue < 1 || brightValue > 255 ||
            !std::isfinite(roiBottom) || roiBottom < 0.2 || roiBottom > 1.0)
            throw std::runtime_error("Invalid traffic light frame count, brightness or search region");
    }
};

struct TrafficLightResult {
    TrafficLightState state = TrafficLightState::Unknown;
    bool stateConfirmed = false;
    cv::Point2f center;
    float lampRadius = 0;
    double ledFraction = 0;
    double colorFraction = 0;
    double backgroundContrast = 0;
    double backgroundDarkFraction = 0;
    double redCoverage = 0, greenCoverage = 0;
    const char* lightEvidence = "none";
    int candidateDisks = 0, candidatePanels = 0;
    int matchedLamps = 0;
    bool panelFound = false;
    int confirmationHits = 0;
    std::array<cv::Point2f, 3> lamps{}; // 左红、中黄、右绿；坐标与输入图像一致。
    std::array<float, 3> lampRadii{};
    std::array<cv::Point2f, 3> matchedLampCenters{};
    std::array<float, 3> matchedLampRadii{}; // 0 表示该灯位未匹配轮廓；不要求三灯齐全。
    std::array<cv::Point2f, 4> panelCorners{};
    std::array<std::array<cv::Point2f, 4>, 2> sideCorners{};
    std::array<double, 2> sideLightFractions{}, sideContrasts{};
    std::array<const char*, 2> sideReasons{{"not_checked", "not_checked"}};
    const char* reason = "no_panel_rectangle";
};

// 固定横排三灯道具：矩形外边界定位，灯盘轮廓辅助排除背景；灯牌内左/右灯位决定红/绿。
// 定位后只检测左右矩形区域的发光证据，不再要求亮灯本身有圆形轮廓；中部黄灯忽略。
class TrafficLightDetector {
public:
    void reset() {
        lastState = TrafficLightState::Unknown;
        stateHits = 0;
        lastRadius = 0;
    }

    TrafficLightResult detect(const cv::Mat& input, const TrafficLightConfig& config) {
        TrafficLightResult result;
        if (input.empty() || input.type() != CV_8UC3 || input.cols < 32 || input.rows < 32) {
            reset(); result.reason = "invalid_frame"; return result;
        }
        const double scale = 320.0 / input.cols;
        const int roiHeight = std::min(input.rows, std::max(1, cvRound(input.rows * config.roiBottom)));
        cv::Mat small, hsv;
        cv::resize(input(cv::Rect(0, 0, input.cols, roiHeight)), small,
                   cv::Size(320, std::max(1, cvRound(roiHeight * scale))), 0, 0, cv::INTER_AREA);
        cv::cvtColor(small, hsv, cv::COLOR_BGR2HSV);
        const auto disks = findDisks(small, hsv);
        const auto panels = findPanels(small, disks);
        result.candidateDisks = int(disks.size()); result.candidatePanels = int(panels.size());
        for (const auto& panel : panels) {
            const std::array<Side, 2> sides{{measureSide(small, panel, 0, config),
                                            measureSide(small, panel, 1, config)}};
            if (!sides[0].valid || !sides[1].valid) continue;
            const char* redEvidence = sides[0].evidence;
            const char* greenEvidence = sides[1].evidence;
            const auto state = redEvidence ? TrafficLightState::Red : greenEvidence ? TrafficLightState::Green : TrafficLightState::Unknown;
            result.state = state;
            const Side& active = sides[state == TrafficLightState::Green ? 1 : 0];
            setGeometry(result, panel, sides, scale);
            if (state == TrafficLightState::Unknown) result.reason = "no_led_or_low_contrast";
            else {
                result.colorFraction = redEvidence ? active.redFraction : active.greenFraction;
                result.lightEvidence = redEvidence ? redEvidence : greenEvidence;
            }
            // 固定单块比赛灯牌：先按外形置信度选目标，再判断亮灭。
            // 不能因真实灯牌熄灭，就改选背景中较弱的矩形和反光点。
            break;
        }
        if (result.state == TrafficLightState::Unknown) {
            reset();
        } else {
            const double movement = cv::norm(result.center - lastCenter);
            const bool sameTarget = result.state == lastState && lastRadius > 0 &&
                movement <= 1.5 * std::max(result.lampRadius, lastRadius) &&
                result.lampRadius >= 0.5 * lastRadius && result.lampRadius <= 2.0 * lastRadius;
            stateHits = sameTarget ? std::min(stateHits + 1, config.confirmFrames) : 1;
            lastState = result.state; lastCenter = result.center; lastRadius = result.lampRadius;
            result.confirmationHits = stateHits; result.stateConfirmed = stateHits >= config.confirmFrames;
            result.reason = result.stateConfirmed ? "confirmed" : "confirming";
        }
        return result;
    }

    static const char* stateName(TrafficLightState state) {
        switch (state) {
            case TrafficLightState::Red: return "RED";
            case TrafficLightState::Green: return "GREEN";
            default: return "UNKNOWN";
        }
    }

private:
    struct Disk { cv::Point2f center; float radius; double quality; };
    struct Side {
        bool valid = false;
        std::array<cv::Point2f, 4> corners{};
        double redFraction = 0, greenFraction = 0, whiteFraction = 0;
        double lightFraction = 0, contrast = 0, darkFraction = 0;
        const char* evidence = nullptr;
        const char* reason = "invalid_side";
    };
    TrafficLightState lastState = TrafficLightState::Unknown;
    int stateHits = 0;
    cv::Point2f lastCenter;
    float lastRadius = 0;

    static std::vector<Disk> findDisks(const cv::Mat& bgr, const cv::Mat& hsv) {
        cv::Mat gray, value;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        cv::extractChannel(hsv, value, 2);
        const auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        std::vector<Disk> all;
        // 扫描亮度层级，圆形轮廓可以来自亮芯、彩色灯罩或暗灯盘边缘。
        // RETR_LIST 同时保留暗孔边界；整个天空/路面的大轮廓不会作为灯盘。
        for (const auto& channel : {gray, value}) for (int threshold = 8; threshold <= 240; threshold += 8) {
            cv::Mat mask;
            cv::threshold(channel, mask, threshold, 255, cv::THRESH_BINARY);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
            for (const auto& contour : contours) {
                const cv::Rect box = cv::boundingRect(contour);
                const double aspect = double(box.width) / box.height;
                if (std::min(box.width, box.height) < 5 || std::max(box.width, box.height) > 60 ||
                    aspect < .50 || aspect > 2.0 || contour.size() < 5 || box.x <= 0 || box.y <= 0 ||
                    box.br().x >= bgr.cols || box.br().y >= bgr.rows) continue;
                std::vector<cv::Point> hull;
                cv::convexHull(contour, hull);
                const double area = cv::contourArea(hull), perimeter = cv::arcLength(hull, true);
                if (area < 12 || area < .5 * box.area() || perimeter <= 0) continue;
                const double circularity = 4 * CV_PI * area / (perimeter * perimeter);
                if (circularity < .60) continue;
                // 方块的圆度也可达到约0.79，不能只靠圆度把它当灯罩。
                // 允许椭圆/轻微变形，但明显三角、四边形和偏离椭圆的杂物不参与定位。
                std::vector<cv::Point> polygon;
                cv::approxPolyDP(hull, polygon, .025 * perimeter, true);
                if (polygon.size() <= 4) continue;
                const auto ellipse = cv::fitEllipse(contour);
                const double a = .5 * ellipse.size.width, b = .5 * ellipse.size.height;
                if (std::min(a, b) < 1.5 || std::max(a, b) > 2.0 * std::min(a, b)) continue;
                const double angle = ellipse.angle * CV_PI / 180.;
                const double cosine = std::cos(angle), sine = std::sin(angle);
                double fitError = 0;
                for (const auto& point : contour) {
                    const double dx = point.x - ellipse.center.x, dy = point.y - ellipse.center.y;
                    const double u = (dx * cosine + dy * sine) / a;
                    const double v = (-dx * sine + dy * cosine) / b;
                    fitError += std::abs(std::hypot(u, v) - 1.);
                }
                fitError /= contour.size();
                if (fitError > .22) continue;
                Disk disk;
                cv::minEnclosingCircle(hull, disk.center, disk.radius);
                if (disk.radius > 25) continue;
                disk.quality = circularity * std::min(1.0, cv::contourArea(contour) / area) * (1. - fitError);
                if (disk.quality < .55) continue;
                all.push_back(disk);
            }
        }
        std::sort(all.begin(), all.end(), [](const Disk& a, const Disk& b) { return a.quality > b.quality; });
        std::vector<Disk> disks;
        for (const auto& disk : all) {
            bool duplicate = false;
            for (const auto& kept : disks) {
                const double size = std::max(disk.radius, kept.radius);
                if (cv::norm(disk.center - kept.center) < .6 * size && std::abs(disk.radius - kept.radius) < .45 * size) {
                    duplicate = true; break;
                }
            }
            if (!duplicate) disks.push_back(disk);
            if (disks.size() == 48) break; // 留出暗灯罩候选名额；位置假设配对仍限前32个。
        }
        return disks;
    }

    struct Panel {
        cv::Point2f center, axis, normal;
        float halfWidth, halfHeight;
        double quality = 0;
        int matchedLamps = 0;
        std::array<cv::Point2f, 3> lampCenters{};
        std::array<float, 3> lampRadii{};
    };

    static bool matchPanelLamps(Panel& panel, const std::vector<Disk>& disks) {
        const double radius = .17 * panel.halfWidth;
        std::array<int, 3> selected{{-1, -1, -1}};
        std::array<double, 3> scores{};
        for (int k = 0; k < 3; ++k) {
            const auto predicted = panel.center + float(k - 1) * .64f * panel.halfWidth * panel.axis;
            for (size_t i = 0; i < disks.size(); ++i) {
                const auto& disk = disks[i];
                const auto delta = disk.center - predicted;
                const double along = std::abs(delta.dot(panel.axis));
                const double across = std::abs(delta.dot(panel.normal));
                if (along > .90 * radius || across > .85 * radius ||
                    disk.radius < .55 * radius || disk.radius > 1.70 * radius ||
                    across + .60 * disk.radius > .95 * panel.halfHeight) continue;
                const double score = disk.quality + .5 * (1. - cv::norm(delta) / (1.3 * radius)) -
                                     .2 * std::abs(std::log(disk.radius / radius));
                if (selected[k] < 0 || score > scores[k]) { selected[k] = int(i); scores[k] = score; }
            }
        }
        // 两个不同灯罩须处于同一排，尺寸相近，间距对应相邻或隔一个灯位。
        // 圆形本身可以宽松，几何关系共同排除背景里恰好出现的两个圆斑。
        const auto compatible = [&](int i, int j) {
            if (selected[i] < 0 || selected[j] < 0 || selected[i] == selected[j]) return false;
            const auto& a = disks[selected[i]];
            const auto& b = disks[selected[j]];
            const auto delta = b.center - a.center;
            const double meanRadius = .5 * (a.radius + b.radius);
            const double spacing = delta.dot(panel.axis) / (j - i);
            return std::min(a.radius, b.radius) >= .55 * std::max(a.radius, b.radius) &&
                   std::abs(delta.dot(panel.normal)) <= .85 * meanRadius &&
                   spacing >= 2.3 * meanRadius && spacing <= 5.8 * meanRadius;
        };
        int mask = 0;
        double support = -1;
        for (int i = 0; i < 3; ++i) for (int j = i + 1; j < 3; ++j) {
            if (compatible(i, j) && scores[i] + scores[j] > support) {
                mask = (1 << i) | (1 << j); support = scores[i] + scores[j];
            }
        }
        if (!mask) return false;
        if (compatible(0, 1) && compatible(0, 2) && compatible(1, 2)) {
            mask = 7; support = scores[0] + scores[1] + scores[2];
        }
        for (int k = 0; k < 3; ++k) if (mask & (1 << k)) {
            ++panel.matchedLamps;
            panel.lampCenters[k] = disks[selected[k]].center;
            panel.lampRadii[k] = disks[selected[k]].radius;
        }
        // 同样两灯时，优先位置、尺寸更贴合的灯牌，而非只累计圆的数量。
        panel.quality += 1.2 * panel.matchedLamps + support / panel.matchedLamps;
        return true;
    }

    static int grayAt(const cv::Mat& gray, cv::Point2f p) {
        return gray.at<unsigned char>(cvRound(p.y), cvRound(p.x));
    }

    // 验证四条外边界，而非仅找“画面里较黑的一块”。灯牌内部应比边界外暗。
    // 允许一侧接近树木/阴影，但上、下边及至少一个侧边必须有连续边界。
    static double panelBoundary(const cv::Mat& gray, const Panel& panel) {
        const float margin = std::max(1.f, .025f * panel.halfWidth);
        for (int x : {-1, 1}) for (int y : {-1, 1}) {
            const auto p = panel.center + float(x) * (panel.halfWidth + margin + 1) * panel.axis +
                           float(y) * (panel.halfHeight + margin + 1) * panel.normal;
            if (p.x < 1 || p.y < 1 || p.x >= gray.cols - 1 || p.y >= gray.rows - 1) return -1;
        }
        std::array<double, 4> support{}, strength{};
        for (int edge = 0; edge < 4; ++edge) {
            const bool horizontal = edge < 2;
            const float sign = edge % 2 == 0 ? -1.f : 1.f;
            const cv::Point2f outward = sign * (horizontal ? panel.normal : panel.axis);
            const float half = horizontal ? panel.halfHeight : panel.halfWidth;
            const cv::Point2f along = horizontal ? panel.axis : panel.normal;
            const float length = horizontal ? panel.halfWidth : panel.halfHeight;
            for (int k = 0; k < 20; ++k) {
                const auto p = panel.center + half * outward + (float(k) / 19.f * 1.8f - .9f) * length * along;
                const int inside = grayAt(gray, p - margin * outward);
                const int outside = grayAt(gray, p + margin * outward);
                const int difference = outside - inside;
                support[edge] += difference >= std::max(8., .08 * inside);
                strength[edge] += std::max(0, std::min(100, difference));
            }
            support[edge] /= 20.; strength[edge] /= 2000.;
        }
        if (support[0] < .55 || support[1] < .50 || std::max(support[2], support[3]) < .50) return -1;
        return support[0] + support[1] + .5 * (support[2] + support[3]) +
               .4 * (strength[0] + strength[1] + strength[2] + strength[3]);
    }

    static std::vector<Panel> findPanels(const cv::Mat& bgr, const std::vector<Disk>& disks) {
        cv::Mat gray;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        std::vector<Panel> candidates;
        const auto add = [&](Panel panel) {
            const double aspect = panel.halfWidth / panel.halfHeight;
            if (panel.halfWidth < 15 || panel.halfWidth > 150 || panel.halfHeight < 5 ||
                aspect < 2.2 || aspect > 5.2) return;
            panel.quality = panelBoundary(gray, panel);
            if (panel.quality < 0) return;
            // 至少两个灯罩即可，不要求正在发光的那一只也有可辨识圆形轮廓。
            if (!matchPanelLamps(panel, disks)) return;
            candidates.push_back(panel);
        };
        // 路径一：直接搜索深色长方形，跨多个灰度层级，适应曝光变化。
        // 多种开运算尺寸用于断开灯牌与电线/支架之间的细小连接。
        for (int kernel : {3, 5}) for (int level = 32; level <= 192; level += 16) {
            cv::Mat mask;
            cv::threshold(gray, mask, level, 255, cv::THRESH_BINARY_INV);
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel, kernel)));
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            for (const auto& contour : contours) {
                if (contour.size() < 4) continue;
                auto rect = cv::minAreaRect(contour);
                double angle = rect.angle * CV_PI / 180.;
                if (rect.size.width < rect.size.height) { std::swap(rect.size.width, rect.size.height); angle += CV_PI / 2.; }
                if (angle > CV_PI / 2.) angle -= CV_PI;
                if (std::abs(angle) > .40 || rect.size.area() <= 0 || cv::contourArea(contour) < .70 * rect.size.area()) continue;
                Panel panel;
                panel.center = rect.center;
                panel.axis = cv::Point2f(float(std::cos(angle)), float(std::sin(angle)));
                panel.normal = cv::Point2f(-panel.axis.y, panel.axis.x);
                panel.halfWidth = .5f * rect.size.width; panel.halfHeight = .5f * rect.size.height;
                add(panel);
            }
        }
        // 路径二：外轮廓与背景粘连时，用任意两个圆盘提出灯牌位置假设。
        // 假设仍必须通过相同的矩形外边界验证；圆盘配对本身不能判定灯色。
        const size_t pairCount = std::min(size_t(32), disks.size());
        for (size_t i = 0; i < pairCount; ++i) for (size_t j = i + 1; j < pairCount; ++j) {
            const Disk& a = disks[i].center.x < disks[j].center.x ? disks[i] : disks[j];
            const Disk& b = disks[i].center.x < disks[j].center.x ? disks[j] : disks[i];
            const auto delta = b.center - a.center;
            const double ratio = a.radius / b.radius;
            if (delta.x < 8 || std::abs(delta.y) > .40 * delta.x || ratio < .55 || ratio > 1.82) continue;
            for (int role = 0; role < 3; ++role) {
                const auto step = delta * (role == 2 ? .5f : 1.f);
                const double spacing = cv::norm(step), meanRadius = .5 * (a.radius + b.radius);
                if (spacing < 2.3 * meanRadius || spacing > 5.8 * meanRadius) continue;
                const double radius = std::max(1.08 * std::min(a.radius, b.radius), spacing / 4.0);
                Panel panel;
                panel.axis = step / float(spacing); panel.normal = cv::Point2f(-panel.axis.y, panel.axis.x);
                const auto middle = role == 0 ? b.center : role == 1 ? a.center : .5f * (a.center + b.center);
                for (double side : {1.7, 2.0, 2.3}) for (double height : {1.4, 1.7, 2.0, 2.3})
                    for (double shift : {-.2, 0., .2}) {
                        panel.center = middle + float(shift * radius) * panel.normal;
                        panel.halfWidth = float(spacing + side * radius);
                        panel.halfHeight = float(height * radius);
                        add(panel);
                    }
            }
        }
        std::sort(candidates.begin(), candidates.end(), [](const Panel& a, const Panel& b) { return a.quality > b.quality; });
        std::vector<Panel> panels;
        for (const auto& panel : candidates) {
            bool duplicate = false;
            for (const auto& kept : panels) {
                const cv::RotatedRect a(panel.center, cv::Size2f(2 * panel.halfWidth, 2 * panel.halfHeight),
                    float(std::atan2(panel.axis.y, panel.axis.x) * 180 / CV_PI));
                const cv::RotatedRect b(kept.center, cv::Size2f(2 * kept.halfWidth, 2 * kept.halfHeight),
                    float(std::atan2(kept.axis.y, kept.axis.x) * 180 / CV_PI));
                std::vector<cv::Point2f> intersection;
                cv::rotatedRectangleIntersection(a, b, intersection);
                if (intersection.size() >= 3 && cv::contourArea(intersection) > .5 * std::min(a.size.area(), b.size.area())) {
                    duplicate = true; break;
                }
            }
            if (!duplicate) panels.push_back(panel);
            if (panels.size() == 12) break;
        }
        return panels;
    }

    static int percentile(const std::array<int, 256>& histogram, int count, double fraction) {
        const int target = std::max(1, int(std::ceil(count * fraction)));
        int accumulated = 0;
        for (int value = 0; value < 256; ++value) {
            accumulated += histogram[value];
            if (accumulated >= target) return value;
        }
        return 255;
    }

    static cv::Point2f panelPoint(const Panel& panel, float x, float y) {
        return panel.center + (2 * x - 1) * panel.halfWidth * panel.axis +
               (2 * y - 1) * panel.halfHeight * panel.normal;
    }

    // 只检查亮区面积和二维分布，不限制外形，也不要求白芯拆成多个连通分量。
    static bool enoughLightArea(const cv::Mat& mask, double fraction) {
        std::vector<cv::Point> points;
        cv::findNonZero(mask, points);
        if (points.size() < size_t(std::max(3, cvCeil(fraction * mask.total())))) return false;
        const auto bounds = cv::boundingRect(points);
        return bounds.width >= std::max(2, cvCeil(.25 * mask.cols)) &&
               bounds.height >= std::max(2, cvCeil(.30 * mask.rows));
    }

    static Side measureSide(const cv::Mat& bgr, const Panel& panel, int side,
                            const TrafficLightConfig& config) {
        Side result;
        // 沿灯牌方向取左右区域。两端留4%，中间39%~61%及上下12%忽略，
        // 避免黄灯罩、支架和灯牌边沿反光进入亮灯判断。
        const float left = side == 0 ? .04f : .61f;
        const float right = side == 0 ? .39f : .96f;
        result.corners = {{panelPoint(panel, left, .12f), panelPoint(panel, right, .12f),
                           panelPoint(panel, right, .88f), panelPoint(panel, left, .88f)}};
        for (const auto& p : result.corners)
            if (p.x < 0 || p.y < 0 || p.x >= bgr.cols - 1 || p.y >= bgr.rows - 1) return result;
        const int width = std::max(3, cvRound(2 * panel.halfWidth * (right - left)));
        const int height = std::max(3, cvRound(2 * panel.halfHeight * .76));
        const auto dx = (result.corners[1] - result.corners[0]) / float(width - 1);
        const auto dy = (result.corners[3] - result.corners[0]) / float(height - 1);
        const cv::Matx23f mapping(dx.x, dy.x, result.corners[0].x,
                                  dx.y, dy.y, result.corners[0].y);
        cv::Mat patch, hsv;
        cv::warpAffine(bgr, patch, mapping, cv::Size(width, height),
                       cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_REPLICATE);
        cv::cvtColor(patch, hsv, cv::COLOR_BGR2HSV);
        std::array<int, 256> neutralHistogram{}, valueHistogram{};
        int reds = 0, greens = 0;
        for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
            const auto& rgb = patch.at<cv::Vec3b>(y, x);
            const auto& pixel = hsv.at<cv::Vec3b>(y, x);
            ++neutralHistogram[std::min(rgb[0], std::min(rgb[1], rgb[2]))];
            ++valueHistogram[pixel[2]];
            reds += (pixel[0] <= 12 || pixel[0] >= 160) && pixel[1] >= 45 && pixel[2] >= 50;
            greens += pixel[0] >= 40 && pixel[0] <= 95 && pixel[1] >= 45 && pixel[2] >= 25;
        }
        const int total = width * height;
        const int neutralBackground = percentile(neutralHistogram, total, .30);
        const int valueBackground = percentile(valueHistogram, total, .30);
        const int neutralHigh = percentile(neutralHistogram, total, .95);
        // 白芯三个通道都会升高，红/绿光晕通常只有部分通道升高。
        // 用 min(B,G,R) 比较同一区域暗部，不再以 HSV V 的光晕亮度否决白芯。
        const int neutralThreshold = std::max(80, neutralBackground + 50);
        const int whiteValue = std::max(100, std::min(config.brightValue - 50, neutralBackground + 80));
        const int relativeColorValue = std::max(140, std::max(valueBackground + 65,
            percentile(valueHistogram, total, .75) + 30));
        cv::Mat white = cv::Mat::zeros(height, width, CV_8U);
        cv::Mat colored = cv::Mat::zeros(height, width, CV_8U);
        int whites = 0, neutralSum = 0, dark = 0;
        for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
            const auto& rgb = patch.at<cv::Vec3b>(y, x);
            const auto& pixel = hsv.at<cv::Vec3b>(y, x);
            const int neutral = std::min(rgb[0], std::min(rgb[1], rgb[2]));
            dark += neutral < neutralHigh - 45;
            if (neutral >= neutralThreshold && pixel[1] <= 120 && pixel[2] >= whiteValue) {
                white.at<unsigned char>(y, x) = 255;
                ++whites; neutralSum += neutral;
            }
        }
        result.valid = true;
        result.redFraction = double(reds) / total;
        result.greenFraction = double(greens) / total;
        result.whiteFraction = double(whites) / total;
        result.lightFraction = result.whiteFraction;
        result.contrast = whites ? double(neutralSum) / whites - neutralBackground : neutralHigh - neutralBackground;
        result.darkFraction = double(dark) / total;
        if (enoughLightArea(white, .04)) {
            result.evidence = result.reason = "white_region";
            return result;
        }
        // 彩色LED仍需多个分散亮点；整块晒亮的红/绿灯罩不是发光证据。
        // 这里仅检查亮点分布，不使用色相决定红绿，也不要求光源轮廓为圆形。
        bool coloredArea = false;
        // 先找强亮点，再补低曝光；直接使用低阈值会把红色灯罩连成整块。
        for (int threshold : {config.brightValue, relativeColorValue}) {
            if (threshold > 255) continue;
            for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
                const auto& pixel = hsv.at<cv::Vec3b>(y, x);
                colored.at<unsigned char>(y, x) = pixel[1] >= 45 && pixel[2] >= threshold ? 255 : 0;
            }
            if (!enoughLightArea(colored, .03)) continue;
            coloredArea = true;
            cv::Mat labels, stats, centers;
            const int count = cv::connectedComponentsWithStats(colored, labels, stats, centers, 8);
            const int minimumArea = std::max(2, cvCeil(.003 * total));
            int significant = 0, largest = 0;
            const int coloredPixels = cv::countNonZero(colored);
            for (int i = 1; i < count; ++i) {
                const int area = stats.at<int>(i, cv::CC_STAT_AREA);
                significant += area >= minimumArea;
                largest = std::max(largest, area);
            }
            if (significant >= 3 && largest <= .70 * coloredPixels) {
                result.evidence = result.reason = "colored_region_array";
                result.lightFraction = double(coloredPixels) / total;
                result.contrast = percentile(valueHistogram, total, .95) - valueBackground;
                return result;
            }
        }
        result.reason = coloredArea ? "colored_region_not_array" :
                        whites ? "white_area_or_extent_low" : "no_bright_region";
        return result;
    }

    static cv::Point2f inputPoint(cv::Point2f point, double scale) {
        return cv::Point2f(float((point.x + 0.5) / scale - 0.5), float((point.y + 0.5) / scale - 0.5));
    }

    static void setGeometry(TrafficLightResult& result, const Panel& panel,
                            const std::array<Side, 2>& sides, double scale) {
        result.panelFound = true;
        result.matchedLamps = panel.matchedLamps;
        for (int i = 0; i < 4; ++i) {
            const float x = (i == 0 || i == 3) ? 0.f : 1.f, y = i < 2 ? 0.f : 1.f;
            result.panelCorners[i] = inputPoint(panelPoint(panel, x, y), scale);
        }
        // 名义灯位仅供目标连续帧关联/兼容调试，不作为圆形检测窗口。
        for (int k = 0; k < 3; ++k) {
            result.lamps[k] = inputPoint(panelPoint(panel, .5f + float(k - 1) * .32f, .5f), scale);
            result.lampRadii[k] = float(std::max(3, cvRound(.17 * panel.halfWidth)) / scale);
            if (panel.lampRadii[k] > 0) {
                result.matchedLampCenters[k] = inputPoint(panel.lampCenters[k], scale);
                result.matchedLampRadii[k] = float(panel.lampRadii[k] / scale);
            }
        }
        for (int side = 0; side < 2; ++side) {
            for (int i = 0; i < 4; ++i) result.sideCorners[side][i] = inputPoint(sides[side].corners[i], scale);
            result.sideLightFractions[side] = sides[side].lightFraction;
            result.sideContrasts[side] = sides[side].contrast;
            result.sideReasons[side] = sides[side].reason;
        }
        const bool green = result.state == TrafficLightState::Green;
        const Side& active = sides[green ? 1 : 0];
        result.center = result.lamps[green ? 2 : 0];
        result.lampRadius = result.lampRadii[green ? 2 : 0];
        result.ledFraction = active.whiteFraction;
        result.backgroundContrast = active.contrast;
        result.backgroundDarkFraction = active.darkFraction;
        result.redCoverage = sides[0].redFraction;
        result.greenCoverage = sides[1].greenFraction;
    }
};

#endif
