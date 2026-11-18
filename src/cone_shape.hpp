#pragma once
#include "cone_config.hpp"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>

// Shape proposals only: no HSV interval, hue class or required body colour.
// Independent channel contours retain isoluminant objects without joining the
// cone to texture edges from other channels. Only the debug mask is OR-combined.
// All proposals pass the same upright, widening silhouette tests.
class ConeShapeDetector {
    // Hulls alone turn foliage and joined background edges into triangles.
    // Check the actual silhouette for two approximately straight sides that
    // widen in opposite directions around a stable centre (any body colour).
    static bool taperedSides(const cv::Mat& silhouette){
        std::vector<double> ys,left,right;
        for(int y=int(silhouette.rows*.15);y<int(silhouette.rows*.85);++y){
            const uchar* p=silhouette.ptr<uchar>(y);int a=0,b=silhouette.cols-1;
            while(a<silhouette.cols&&!p[a])++a;while(b>=a&&!p[b])--b;
            if(b>=a){ys.push_back(y);left.push_back(a);right.push_back(b);}
        }
        if(ys.size()<6)return false;
        double sy=0,syy=0;for(double y:ys){sy+=y;syy+=y*y;}
        double n=ys.size(),den=n*syy-sy*sy;
        auto fit=[&](const std::vector<double>& xs,double& slope){
            double sx=0,sxy=0;for(size_t i=0;i<ys.size();++i){sx+=xs[i];sxy+=xs[i]*ys[i];}
            slope=(n*sxy-sx*sy)/std::max(1.,den);double intercept=(sx-slope*sy)/n,error=0;
            for(size_t i=0;i<ys.size();++i)error+=std::abs(xs[i]-slope*ys[i]-intercept);
            return error/n;
        };
        double ls,rs,le=fit(left,ls),re=fit(right,rs);
        double h=silhouette.rows,w=silhouette.cols;
        return ls*h<-.12*w&&rs*h>.12*w&&std::abs((ls+rs)*h)<.25*w&&
               le<=std::max(.6,.055*w)&&re<=std::max(.6,.055*w);
    }
    static double bandWidth(const cv::Mat& mask,double low,double high,double& centre){
        double width=0,sumCentre=0;int rows=0;
        for(int y=std::max(0,int(mask.rows*low));y<std::min(mask.rows,int(mask.rows*high));++y){
            const uchar* p=mask.ptr<uchar>(y);int a=0,b=mask.cols-1;
            while(a<mask.cols&&!p[a])++a;while(b>=a&&!p[b])--b;
            if(b>=a){width+=b-a+1;sumCentre+=(a+b)*.5;++rows;}
        }
        centre=rows?sumCentre/rows:mask.cols*.5;return rows?width/rows:0;
    }
public:
    std::vector<cv::Rect> detect(const cv::Mat& frame,const ConeConfig& c,cv::Mat& mask)const{
        int top=std::max(0,c.cone_roi_top-100);
        cv::Rect roi(0,top,frame.cols,frame.rows-top);cv::Mat crop=frame(roi),gray;
        cv::cvtColor(crop,gray,cv::COLOR_BGR2GRAY);
        std::vector<cv::Mat> channels;cv::split(crop,channels);channels.push_back(gray);
        auto contrast=cv::createCLAHE(2.,{8,8});cv::Mat edges=cv::Mat::zeros(crop.size(),CV_8UC1);
        std::vector<cv::Rect> proposals;
        for(const auto& channel:channels){
            cv::Mat local,blurred,e;contrast->apply(channel,local);
            cv::GaussianBlur(local,blurred,{3,3},0);cv::Canny(blurred,e,30,90);
            cv::morphologyEx(e,e,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,{3,3}));
            cv::bitwise_or(edges,e,edges);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(e,contours,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
            for(const auto& contour:contours){
                if(contour.size()<3)continue;
                std::vector<cv::Point> hull,poly;cv::convexHull(contour,hull);
                cv::Rect r=cv::boundingRect(hull);double ratio=double(r.height)/std::max(1,r.width);
                if(r.width<c.cone_shape_min_width||r.height<c.cone_shape_min_height||r.width>c.cone_max_width||
                   ratio<c.cone_shape_aspect_min||ratio>3.5||r.br().y+top<c.cone_roi_top+c.cone_min_height)continue;
                double area=cv::contourArea(hull),fill=area/std::max(1,r.area());
                // A cone leaving the bottom has no visible base edge, so its Canny
                // contour can be an open outline with near-zero area. Its hull must
                // still satisfy the same upright widening geometry below.
                bool bottomClipped=r.br().y>=crop.rows-1;
                if(fill<.32||fill>.88||(!bottomClipped&&std::abs(cv::contourArea(contour))<area*.60))continue;
                cv::approxPolyDP(hull,poly,.035*cv::arcLength(hull,true),true);
                if(poly.size()<3||poly.size()>6)continue;
                std::vector<cv::Point> local;for(auto p:hull)local.push_back(p-r.tl());
                cv::Mat silhouette=cv::Mat::zeros(r.size(),CV_8UC1);cv::fillConvexPoly(silhouette,local,255);
                double tc,mc,bc;
                double tw=bandWidth(silhouette,.12,.27,tc),mw=bandWidth(silhouette,.42,.58,mc),bw=bandWidth(silhouette,.73,.90,bc);
                if(bw<5||tw/bw>c.cone_shape_top_ratio||mw<tw*.95||bw<mw*1.10||
                   std::abs(tc-bc)>r.width*.28||std::abs(mc-bc)>r.width*.22)continue;
                if(!bottomClipped){
                    cv::Mat actual=cv::Mat::zeros(r.size(),CV_8UC1);
                    std::vector<cv::Point> outline;for(auto p:contour)outline.push_back(p-r.tl());
                    cv::drawContours(actual,std::vector<std::vector<cv::Point>>{outline},0,255,cv::FILLED);
                    if(!taperedSides(actual))continue;
                }
                r.y+=top;proposals.push_back(r);
            }
        }
        mask=cv::Mat::zeros(frame.size(),CV_8UC1);edges.copyTo(mask(roi));
        // Inner/outer edge loops and painted bands are one physical proposal.
        std::sort(proposals.begin(),proposals.end(),[](const cv::Rect& a,const cv::Rect& b){return a.area()>b.area();});
        std::vector<cv::Rect> result;
        for(auto r:proposals){
            bool duplicate=false;
            for(auto kept:result){
                double common=(r&kept).area();
                if(common/std::max(1,std::min(r.area(),kept.area()))>.72){duplicate=true;break;}
            }
            if(!duplicate)result.push_back(r);
        }
        return result;
    }
};
