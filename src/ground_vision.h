// 基于相机标定的平地投影辅助模块，主要供 depth 停车后端使用。
// 地面坐标 x 向前、y 向左，长度单位米；与普通巡线的图像误差单位不同。
#ifndef GROUND_VISION_H
#define GROUND_VISION_H

// Lightweight, camera-only perception. Coordinates: x forward, y left, metres.
// No course positions, simulator state, GPS truth or object IDs are consumed.
// Flat-ground projection MUST be calibrated with the installed camera.
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

// 相机高度、前伸量、俯仰角和水平视场角共同决定投影；laneWidth 为假定车道宽。
// 这里是配置模型，改变摄像头实际姿态后需要相应标定，算法不会自动测出安装姿态。
struct GroundConfig {
    double cameraHeight=.14, cameraForward=.17, pitchDegrees=15, horizontalFov=120;
    double laneWidth=1.22;
};
// 各 Distance 为负数表示未检测；laneConfidence 为本算法给出的离散可信程度。
// cones 保存投影到地面的二维位置，不含物体 ID 或仿真真值。
struct GroundObservation {
    double laneError=0, laneHeading=0, laneConfidence=0;
    double zebraDistance=-1, yellowDistance=-1;
    double boardDistance=-1, boardLateral=0;
    std::vector<cv::Point2d> cones;
};

class GroundVision {
    static const int W=320, H=240;
    const double nearX=.25, farX=4.0, extent=1.6;
    cv::Mat homography;
    cv::Size lastSize;
    GroundConfig cached;
    static std::vector<std::vector<cv::Point>> contours(const cv::Mat& m) {
        std::vector<std::vector<cv::Point>> c;
        cv::findContours(m,c,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        return c;
    }
    // 鸟瞰图大小 320×240，覆盖前方 0.25..4.0 米、左右各 1.6 米的平地区域。
    // 鸟瞰图行号越大离车越近，列号越小越靠车体左侧。
    cv::Point2d birdGround(double u,double v) const {
        return {farX-v/(H-1)*(farX-nearX),extent*(1-2*u/(W-1))};
    }
    // 按针孔相机和平地假设，把给定地面点投影回原图，用于建立鸟瞰变换。
    cv::Point2f project(double x,double y,cv::Size s,const GroundConfig& c) const {
        double p=c.pitchDegrees*CV_PI/180, f=s.width/(2*std::tan(c.horizontalFov*CV_PI/360));
        double dx=x-c.cameraForward, depth=dx*std::cos(p)+c.cameraHeight*std::sin(p);
        double up=dx*std::sin(p)-c.cameraHeight*std::cos(p);
        return {float(s.width*.5-f*y/depth),float(s.height*.5-f*up/depth)};
    }
public:
    cv::Mat bird;
    // 从像素反求视线与地面的交点；视线平行地面或向上时没有有效交点，返回 false。
    bool unproject(double u,double v,cv::Size s,const GroundConfig& c,cv::Point2d& out) const {
        double p=c.pitchDegrees*CV_PI/180, f=s.width/(2*std::tan(c.horizontalFov*CV_PI/360));
        double up=-(v-s.height*.5)/f, left=-(u-s.width*.5)/f;
        double rx=std::cos(p)+up*std::sin(p), rz=-std::sin(p)+up*std::cos(p);
        if(rz>=-1e-5) return false;
        double scale=-c.cameraHeight/rz;
        out={c.cameraForward+scale*rx,scale*left};
        return true;
    }
    // 缓存相机参数和图像大小；参数变化时重算单应矩阵，否则复用。
    // needZebra/needCones/needParking 控制附加检测；车道几何始终计算。
    GroundObservation process(const cv::Mat& image,const GroundConfig& c,bool blueCones,
                              bool needZebra,bool needCones,bool needParking) {
        GroundObservation o;
        if(image.empty())return o;
        if(homography.empty() || image.size()!=lastSize || c.pitchDegrees!=cached.pitchDegrees ||
           c.cameraHeight!=cached.cameraHeight || c.cameraForward!=cached.cameraForward ||
           c.horizontalFov!=cached.horizontalFov) {
            std::vector<cv::Point2f> src{project(farX,extent,image.size(),c),project(farX,-extent,image.size(),c),
                project(nearX,-extent,image.size(),c),project(nearX,extent,image.size(),c)};
            std::vector<cv::Point2f> dst{{0,0},{W-1,0},{W-1,H-1},{0,H-1}};
            homography=cv::getPerspectiveTransform(src,dst); lastSize=image.size(); cached=c;
        }
        // 先变换到鸟瞰视角，再按白色 HSV 阈值和竖直开运算提取纵向车道边缘。
        cv::warpPerspective(image,bird,homography,cv::Size(W,H),cv::INTER_LINEAR);
        cv::Mat hsv,white,vertical;
        cv::cvtColor(bird,hsv,cv::COLOR_BGR2HSV);
        cv::inRange(hsv,cv::Scalar(0,0,145),cv::Scalar(179,65,255),white);
        cv::morphologyEx(white,vertical,cv::MORPH_OPEN,cv::getStructuringElement(cv::MORPH_RECT,{3,9}));
        struct Edge {double y,slope,score;};
        std::vector<Edge> left,right;
        // 对候选白线像素做地面坐标的一元直线拟合，在前方 0.9 米处比较其横向位置。
        // 优先选择接近假定半车道宽的左右边缘，并用宽度一致性检查双侧结果。
        for(const auto& contour:contours(vertical)) {
            cv::Rect r=cv::boundingRect(contour);
            if(r.height<18 || r.width>150)continue;
            double sx=0,sy=0,sxx=0,sxy=0; int n=0;
            for(int v=r.y;v<r.y+r.height;v+=2) {
                double x=birdGround(0,v).x;
                if(x<.35 || x>2.6)continue;
                const uchar* row=vertical.ptr<uchar>(v);
                for(int u=r.x;u<r.x+r.width;++u)if(row[u]) {
                    double y=birdGround(u,v).y;
                    sx+=x;sy+=y;sxx+=x*x;sxy+=x*y;++n;
                }
            }
            double den=n*sxx-sx*sx;
            if(n<18 || std::abs(den)<1e-6)continue;
            double slope=(n*sxy-sx*sy)/den, intercept=(sy-slope*sx)/n;
            double y=intercept+.9*slope;
            Edge e{y,slope,std::abs(std::abs(y)-c.laneWidth/2)};
            if(y>.12)left.push_back(e); else if(y<-.12)right.push_back(e);
        }
        auto cmp=[](const Edge&a,const Edge&b){return a.score<b.score;};
        std::sort(left.begin(),left.end(),cmp);std::sort(right.begin(),right.end(),cmp);
        // 双侧可信时取两边中点；仅有一侧时按配置 laneWidth 推算中心，可信度降为 0.55。
        // 这仅是地面投影辅助观测，不是主巡线 LaneV2 的画面边缘补位规则。
        if(!left.empty() && !right.empty() && left[0].y-right[0].y>.55*c.laneWidth &&
           left[0].y-right[0].y<1.6*c.laneWidth) {
            o.laneError=(left[0].y+right[0].y)/2;
            o.laneHeading=std::atan((left[0].slope+right[0].slope)/2);o.laneConfidence=.95;
        } else if(!left.empty() || !right.empty()) {
            Edge e=left.empty()?right[0]:left[0];
            o.laneError=e.y+(left.empty()?c.laneWidth/2:-c.laneWidth/2);
            o.laneHeading=std::atan(e.slope);o.laneConfidence=.55;
        }
        // 可选的鸟瞰斑马线粗检测：统计多次白色跳变及填充率，返回最近支持行的投影距离。
        // 集成主斑马线流程使用 zebra_original.hpp，深度停车调用时关闭本分支。
        if(needZebra) {
            int margin=int(W*(.5-.45*c.laneWidth/(2*extent))), rows=0,nearest=0;
            for(int v=0;v<H;v+=2) {
                int runs=0,filled=0;bool prior=false;
                const uchar* row=white.ptr<uchar>(v);
                for(int u=margin;u<W-margin;++u) {
                    bool on=row[u]!=0;if(on && !prior)++runs;if(on)++filled;prior=on;
                }
                double fill=double(filled)/(W-2*margin);
                if(runs>=3 && fill>.22 && fill<.85){++rows;nearest=v;}
            }
            if(rows>=3)o.zebraDistance=birdGround(W/2,nearest).x;
        }
        // 黄色横向开运算提取长横条，返回最近候选距离；停车专用入口还会在 ParkingVision 重筛。
        if(needParking) {
            cv::Mat yellow;
            cv::inRange(hsv,cv::Scalar(17,100,100),cv::Scalar(38,255,255),yellow);
            cv::morphologyEx(yellow,yellow,cv::MORPH_OPEN,cv::getStructuringElement(cv::MORPH_RECT,{41,1}));
            for(const auto& contour:contours(yellow)) {
                cv::Rect r=cv::boundingRect(contour);
                if(r.width<50 || r.height<2 || r.width<1.5*r.height)continue;
                double d=birdGround(r.x+r.width/2.,r.y+r.height).x;
                if(d>.1 && (o.yellowDistance<0 || d<o.yellowDistance))o.yellowDistance=d;
            }
        }
        // 立体物体在原相机图中分色，用候选框底部接地点做反投影，再估算尺寸区分板和桶。
        // 此处只提供通用候选，深度停车的蓝板识别还会应用额外尺寸和饱和度条件。
        if(needCones || needParking) {
            cv::Mat hsvImage,blue,coneColor;
            cv::cvtColor(image,hsvImage,cv::COLOR_BGR2HSV);
            cv::inRange(hsvImage,cv::Scalar(90,70,35),cv::Scalar(135,255,255),blue);
            if(blueCones)coneColor=blue;
            else {
                cv::Mat a,b;
                cv::inRange(hsvImage,cv::Scalar(0,90,60),cv::Scalar(10,255,255),a);
                cv::inRange(hsvImage,cv::Scalar(170,90,60),cv::Scalar(179,255,255),b);
                cv::bitwise_or(a,b,coneColor);
            }
            auto objects=[&](const cv::Mat& mask,bool boards,bool cones) {
                double f=image.cols/(2*std::tan(c.horizontalFov*CV_PI/360));
                for(const auto& contour:contours(mask)) {
                    double area=cv::contourArea(contour);if(area<8)continue;
                    cv::Rect r=cv::boundingRect(contour);cv::Point2d ground;
                    if(!unproject(r.x+r.width/2.,r.y+r.height,image.size(),c,ground) || ground.x<.38 || ground.x>8)continue;
                    double height=r.height*std::max(.1,ground.x-c.cameraForward)/f;
                    if(boards && height>.21 && area>35 && (o.boardDistance<0 || ground.x<o.boardDistance)) {
                        o.boardDistance=ground.x;o.boardLateral=ground.y;
                    } else if(cones && height>.018 && height<.18 && r.width<image.cols*.15 &&
                              r.width/double(std::max(1,r.height))<2.5)o.cones.push_back(ground);
                }
            };
            objects(blue,needParking,needCones && blueCones);
            if(needCones && !blueCones)objects(coneColor,false,true);
        }
        return o;
    }
};
#endif
