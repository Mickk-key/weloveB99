// 通过 Linux V4L2 ioctl 查询和设置相机成像参数；不控制电机或摄像头支架舵机。
// 设备句柄在构造时打开、析构时关闭，和 VideoCapture 的取帧职责分开。
#pragma once
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstring>
class CameraControls {
    int fd=-1;
public:
    explicit CameraControls(const std::string& device){fd=::open(device.c_str(),O_RDWR|O_NONBLOCK);if(fd<0)throw std::runtime_error("Cannot open camera controls: "+device+": "+std::strerror(errno));}
    ~CameraControls(){if(fd>=0)::close(fd);}
    // query 查询范围/步长/支持状态，get 读取当前值，menu 检查枚举菜单项是否可用。
    v4l2_queryctrl query(unsigned id){v4l2_queryctrl q{};q.id=id;if(ioctl(fd,VIDIOC_QUERYCTRL,&q)<0||(q.flags&V4L2_CTRL_FLAG_DISABLED))throw std::runtime_error("Camera does not support control "+std::to_string(id));return q;}
    int get(unsigned id){v4l2_control c{};c.id=id;if(ioctl(fd,VIDIOC_G_CTRL,&c)<0)throw std::runtime_error("Camera control read failed");return c.value;}
    bool menu(unsigned id,int value){v4l2_querymenu q{};q.id=id;q.index=value;return ioctl(fd,VIDIOC_QUERYMENU,&q)==0;}
    // 写入前检查设备报告的最小值、最大值、步长和菜单选项；写入后读回并打印实际值。
    void set(unsigned id,int value){
        auto q=query(id);if(value<q.minimum||value>q.maximum||(q.step>0&&(value-q.minimum)%q.step))throw std::runtime_error(std::string((char*)q.name)+" value outside supported range/step");
        if(q.type==V4L2_CTRL_TYPE_MENU&&!menu(id,value))throw std::runtime_error("Unsupported camera menu value");
        v4l2_control c{};c.id=id;c.value=value;if(ioctl(fd,VIDIOC_S_CTRL,&c)<0)throw std::runtime_error(std::string((char*)q.name)+" set failed: "+std::strerror(errno));
        std::cout<<"[CAMERA] "<<q.name<<" requested="<<value<<" actual="<<get(id)<<std::endl;
    }
    // 自动曝光优先选择光圈优先模式，不支持时回退到自动模式；手动时才写绝对曝光值。
    // 因此自动模式下配置的 set_exposure 不会作为固定曝光值写入。
    void exposure(bool automatic,int value){
        int mode=V4L2_EXPOSURE_MANUAL;
        if(automatic){mode=menu(V4L2_CID_EXPOSURE_AUTO,V4L2_EXPOSURE_APERTURE_PRIORITY)?V4L2_EXPOSURE_APERTURE_PRIORITY:V4L2_EXPOSURE_AUTO;}
        set(V4L2_CID_EXPOSURE_AUTO,mode);
        if(!automatic)set(V4L2_CID_EXPOSURE_ABSOLUTE,value);
    }
    // 列出常见控制项的范围、默认值、当前值及菜单项；单项不支持时报告后继续查询。
    void info(){
        for(unsigned id:{V4L2_CID_EXPOSURE_AUTO,V4L2_CID_EXPOSURE_ABSOLUTE,V4L2_CID_BRIGHTNESS,V4L2_CID_GAIN,V4L2_CID_AUTO_WHITE_BALANCE,V4L2_CID_WHITE_BALANCE_TEMPERATURE,V4L2_CID_EXPOSURE_AUTO_PRIORITY}){
            try{auto q=query(id);std::cout<<q.name<<" min="<<q.minimum<<" max="<<q.maximum<<" step="<<q.step<<" default="<<q.default_value<<" current="<<get(id)<<'\n';
                if(q.type==V4L2_CTRL_TYPE_MENU)for(int v=q.minimum;v<=q.maximum;++v){v4l2_querymenu m{};m.id=id;m.index=v;if(ioctl(fd,VIDIOC_QUERYMENU,&m)==0)std::cout<<"  "<<v<<"="<<m.name<<'\n';}
            }catch(const std::exception& e){std::cout<<"unsupported id="<<id<<" "<<e.what()<<'\n';}
        }
    }
    // 按当前曝光约 15% 或设备最小步长调整，再限幅并对齐步长；调用方负责实际写入。
    int adjust(int value,int direction){auto q=query(V4L2_CID_EXPOSURE_ABSOLUTE);int step=std::max(q.step,int(std::round(value*.15)));int n=std::max(q.minimum,std::min(q.maximum,value+direction*step));if(q.step>0)n=q.minimum+(n-q.minimum)/q.step*q.step;return n;}
};
