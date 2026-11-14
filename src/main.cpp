// 程序入口：解析命令行、选择图像来源、调用 Suite，再把最终命令交给 Output。
// 默认 env=outdoor_v2、modules=lane；其他功能需通过 --modules 显式启用。
// 图像统一为 640×480 BGR；检测模块只计算结果，硬件输出集中在 runtime.hpp。
#include "suite.hpp"
#include "camera_controls.hpp"
#include "sim_transport.hpp"
#include <memory>

// 将一帧的状态、PWM、巡线误差及各模块结果组织成 JSONL。
// 同一份消息用于终端、退出时的日志和仿真回传；PWM 是控制量，不是实际车速。
std::string json(const Command& c,double t){
    std::ostringstream s;s<<"{\"t\":"<<t<<",\"state\":\""<<c.state<<"\",\"motor_pwm\":"<<c.motor<<",\"servo_pwm\":"<<c.steer<<",\"target_pwm\":"<<c.target<<",\"error\":"<<c.error<<",\"lane_error\":"<<c.baseError<<",\"left\":"<<c.left<<",\"right\":"<<c.right<<",\"cone\":\""<<c.cone<<"\",\"offset\":"<<c.offset<<",\"passed\":"<<c.passed<<",\"light\":"<<c.light<<",\"red_pixels\":"<<c.red<<",\"green_pixels\":"<<c.green<<",\"zebra\":\""<<c.zebra<<"\",\"zebra_y\":"<<c.zebraY<<",\"parking\":\""<<c.parking<<"\",\"parking_armed\":"<<c.parkingArmed<<",\"reason\":\""<<c.reason<<"\",\"fixed_remaining_s\":"<<c.fixedRemaining<<",\"frame_dt_s\":"<<c.frameDt<<",\"processing_ms\":"<<c.processingMs<<",\"parking_side\":"<<c.parkingSide<<",\"edge_heading\":"<<c.edgeHeading<<",\"qr_hits\":"<<c.qrHits<<",\"qr_age_s\":"<<c.qrAge<<"}\n";return s.str();
}
int main(int argc,char** argv){try{
    std::string env="outdoor_v2",config,mod="lane",image,video,out="runs/latest",exposure,backend,parkingMode,device;
    bool motion=false,show=false,masks=false,check=false,rawOnly=false,info=false;int frames=0,port=0,exposureValue=-1;
    // 无值开关直接设置布尔量，其余选项消费后一个参数；非法名称或缺值会报错。
    // --masks 同时开启窗口，--camera-only 只查看原图；默认 motion=false。
    for(int i=1;i<argc;++i){std::string a=argv[i];
        if(a=="--enable-motion")motion=true;else if(a=="--show")show=true;else if(a=="--masks")masks=show=true;
        else if(a=="--check-config")check=true;else if(a=="--camera-only")rawOnly=show=true;else if(a=="--camera-info")info=true;
        else if(a=="--help"){
            std::cout<<"Race suite: --env indoor|outdoor|outdoor_v2|sim --modules lane,light,cone,zebra,parking (or all)\n"
                     <<"--show --masks --enable-motion --parking-mode qr|depth --lane-backend indoor|outdoor|outdoor_v2\n"
                     <<"--camera-only (raw window, NO GPIO) --camera-info --exposure auto|manual --exposure-value N --device PATH\n"
                     <<"--config YAML --check-config --image FILE --replay VIDEO --frames N --out RELATIVE_PREFIX --sim-port PORT\n";return 0;
        }else{
            if(i+1==argc)throw std::runtime_error("Missing option value: "+a);std::string v=argv[++i];
            if(a=="--env")env=v;else if(a=="--config")config=v;else if(a=="--modules")mod=v;else if(a=="--image")image=v;else if(a=="--replay")video=v;
            else if(a=="--out")out=v;else if(a=="--exposure")exposure=v;else if(a=="--lane-backend")backend=v;else if(a=="--parking-mode")parkingMode=v;else if(a=="--device")device=v;
            else if(a=="--frames"||a=="--sim-port"||a=="--exposure-value"){size_t used=0;int n=std::stoi(v,&used);if(used!=v.size()||n<1)throw std::runtime_error("Invalid number: "+v);if(a=="--frames")frames=n;else if(a=="--sim-port")port=n;else exposureValue=n;}
            else throw std::runtime_error("Unknown option: "+a);
        }
    }
    if(env!="indoor"&&env!="outdoor"&&env!="outdoor_v2"&&env!="sim")throw std::runtime_error("Unknown environment");
    if(!exposure.empty()&&exposure!="auto"&&exposure!="manual")throw std::runtime_error("Exposure must be auto or manual");
    // 配置优先级：先按环境选择 YAML，再由命令行覆盖后端、停车模式和相机选项。
    // --config 指定了文件时，不再自动使用 config/<env>.yaml。
    if(config.empty())config="config/"+env+".yaml";
    SuiteConfig c;c.load(config);if(!backend.empty())c.backend=backend;if(!parkingMode.empty())c.parkingMode=parkingMode;
    if((c.backend!="indoor"&&c.backend!="outdoor"&&c.backend!="outdoor_v2")||(c.parkingMode!="qr"&&c.parkingMode!="depth"))throw std::runtime_error("Invalid backend/parking mode");
    if(!device.empty())c.drive.camera_device=device;
    if(exposureValue>0)c.drive.set_exposure=exposureValue;
    if(!exposure.empty())c.drive.set_auto_exposure=exposure=="manual"?1:3;
    Modules modules(mod);
    std::cout<<"CONFIG env="<<env<<" lane="<<c.backend<<" modules="<<modules.str()<<" parking_mode="<<c.parkingMode<<" watchdog="<<(c.drive.control_watchdog_enabled?"ON":"OFF")<<" file="<<config<<std::endl;
    // 配置检查在打开相机、创建 Output 之前返回，只读取和校验参数。
    if(check)return 0;
    // 运动模式只允许树莓派编译版本使用实时相机；图片、回放、仿真及相机工具拒绝运动。
    // 这里仅检查运行组合；是否真正初始化 GPIO 仍由 Output 的 motion 参数决定。
    if((motion&&(rawOnly||info||port||env=="sim"||!image.empty()||!video.empty())))throw std::runtime_error("Motion is only allowed with live camera driving in indoor/outdoor");
#ifndef RACE_PI
    if(motion)throw std::runtime_error("Desktop build has NO GPIO; use build_pi.sh on the car");
#endif
    if((!image.empty())+(!video.empty())+(port>0)>1)throw std::runtime_error("Choose one input source");
    if((rawOnly||info)&&(port||!image.empty()||!video.empty()))throw std::runtime_error("Camera tools require a real camera");
    if(env=="sim"&&!port&&image.empty()&&video.empty())throw std::runtime_error("Use sh sim.sh for a simulation, or supply --image/--replay");
    if(show&&!getenv("DISPLAY"))throw std::runtime_error("Window requires local desktop or SSH X11 forwarding (DISPLAY)");
    signal(SIGINT,on_signal);signal(SIGTERM,on_signal);signal(SIGHUP,on_signal);cv::setNumThreads(2);
    // 输出前缀必须是项目内的相对路径。终端打印由独立线程处理，避免慢终端拖住控制。
    // 输入来源互斥：仿真 TCP、单张图片、视频文件或实时 V4L2 相机。
    parent_dirs(out);AsyncConsole console;SimTransport sim;cv::VideoCapture cap;cv::Mat frame;
    std::unique_ptr<CameraControls> controls;
    if(port)sim.open(port);
    else if(!image.empty()){frame=cv::imread(image);if(frame.empty())throw std::runtime_error("Image unreadable");cv::resize(frame,frame,{640,480});}
    else if(!video.empty()){if(!cap.open(video))throw std::runtime_error("Replay unavailable");}
    else{
        // 实时相机先请求分辨率和帧率，再设置曝光等控制；-1 的可选参数表示不写入。
        // 这些是成像控制，摄像头支架舵机是否动作另由 Output 和 camera_home_enabled 决定。
        controls.reset(new CameraControls(c.drive.camera_device));if(info){controls->info();return 0;}
        if(!cap.open(c.drive.camera_device,cv::CAP_V4L2))throw std::runtime_error("Camera unavailable");
        cap.set(cv::CAP_PROP_FRAME_WIDTH,640);cap.set(cv::CAP_PROP_FRAME_HEIGHT,480);cap.set(cv::CAP_PROP_FPS,c.cameraFps);cap.set(cv::CAP_PROP_BUFFERSIZE,1);
        controls->exposure(c.drive.set_auto_exposure!=1,c.drive.set_exposure);
        if(c.brightness>=0)controls->set(V4L2_CID_BRIGHTNESS,c.brightness);if(c.gain>=0)controls->set(V4L2_CID_GAIN,c.gain);if(c.whiteBalanceAuto>=0)controls->set(V4L2_CID_AUTO_WHITE_BALANCE,c.whiteBalanceAuto);
        std::cout<<"[CAMERA] requested 640x480 fps="<<c.cameraFps<<" actual="<<cap.get(cv::CAP_PROP_FRAME_WIDTH)<<'x'<<cap.get(cv::CAP_PROP_FRAME_HEIGHT)<<" fps="<<cap.get(cv::CAP_PROP_FPS)<<"; camera servos unchanged"<<std::endl;
    }
    // Hardware is initialized only after the input and camera settings succeed.
    // 输入初始化成功后才创建硬件输出对象；未指定 --enable-motion 时不发送 GPIO。
    // Suite 保存跨帧识别和任务状态，records 只保留最近 4096 条状态记录。
    // Output后构造先析构：异常退出也先停止输出，再等待后台QR线程收尾。
    Suite suite(c,modules,image.empty()&&video.empty()&&!port);Output output(c.drive,motion,false);auto start=Clock::now();int n=0;Command command;std::deque<std::string> records;std::string previousState;
    // 主循环顺序：取帧 → 算法与任务仲裁 → 输出命令 → 诊断显示。
    // phase 标记当前处理阶段，供控制超时时定位卡顿位置。
    while(!interrupted&&!output.aborted){
        output.phase=1;if(image.empty()&&!(port?sim.read(frame):cap.read(frame)))break;if(frame.empty())break;
        if(frame.size()!=cv::Size(640,480)){if(!motion&&!video.empty())cv::resize(frame,frame,{640,480});else throw std::runtime_error("Expected 640x480 camera frame");}
        // 任务时间单位统一为秒：仿真使用包内累计时间，回放按视频 FPS，单图按 30 FPS。
        // 实车使用 steady_clock 的已运行时长；这些时间用于制动、等灯、等待和绕桶状态。
        double now=port?sim.time:!video.empty()?n/std::max(1.,cap.get(cv::CAP_PROP_FPS)):!image.empty()?n/30.:std::chrono::duration<double>(Clock::now()-start).count();
        // 只有非纯相机模式才运行 Suite；send 接收已经完成优先级仲裁的一条电机/舵机命令。
        // 日志每 10 帧或状态变化时提交终端，完整的有界记录在结束后落盘。
        if(!rawOnly){output.phase=2;auto processingStart=Clock::now();
            try{command=suite.step(frame,now);}catch(...){output.finish();throw;}
            command.processingMs=std::chrono::duration<double,std::milli>(Clock::now()-processingStart).count();output.phase=4;if(!output.send(command.motor,command.steer,command.state,command.brakeRemaining))break;
            auto message=json(command,now);if(port)sim.send(message);if(records.size()>=4096)records.pop_front();records.push_back(message);if(n%10==0||command.state!=previousState)console.publish(message);previousState=command.state;
        }
        // 显示原图、识别叠加图及可选巡线掩码；Esc/q 结束。
        // 纯相机模式下 a/m 切换自动/手动曝光，+/- 调整曝光；这组按键不参与驾驶控制。
        if(show){output.phase=5;cv::imshow("camera raw",frame);if(!rawOnly){cv::imshow("race detection",suite.draw(frame,command));if(masks)cv::imshow("lane mask",suite.laneMask);}
            int key=cv::waitKey(1)&255;if(key==27||key=='q')break;
            if(rawOnly&&controls){try{if(key=='a'){controls->exposure(true,c.drive.set_exposure);c.drive.set_auto_exposure=3;}else if(key=='m'){controls->exposure(false,c.drive.set_exposure);c.drive.set_auto_exposure=1;}else if(key=='+'||key=='='||key=='-'){c.drive.set_exposure=controls->adjust(c.drive.set_exposure,key=='-'?-1:1);controls->exposure(false,c.drive.set_exposure);c.drive.set_auto_exposure=1;}}catch(const std::exception& e){std::cerr<<"[CAMERA] "<<e.what()<<std::endl;}}
        }
        ++n;if((frames&&n>=frames)||(!image.empty()&&n>=std::max(1,frames)))break;
    }
    // 结束时先将输出收回中位并停止硬件线程，再等待终端线程退出，最后写日志和图片。
    // 控制看门狗触发返回 2；异常也由最外层捕获并返回 2。
    output.finish();console.finish();
    if(!rawOnly){std::ofstream f(out+".jsonl");for(const auto& row:records)f<<row;if(!frame.empty())cv::imwrite(out+".jpg",suite.draw(frame,command));}
    std::cout<<"FINAL neutral="<<c.drive.neutral_pwm<<" frames="<<n<<" watchdog_abort="<<output.aborted<<std::endl;
    return output.aborted==2?2:0;
}catch(const std::exception& e){std::cerr<<"ERROR "<<e.what()<<std::endl;return 2;}}
