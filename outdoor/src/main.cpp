// 程序入口：解析命令行、选择图像来源、调用 Suite，再把最终命令交给 Output。
// 本目录由 lane_backend.hpp 固定巡线环境；默认modules=lane，完整比赛使用--modules all。
// 图像统一为 640×480 BGR；检测模块只计算结果，硬件输出集中在 runtime.hpp。
#include "suite.hpp"
#include "camera_controls.hpp"
#include <memory>
#include <unistd.h>
#include "parking_photo.hpp"

// 将一帧的状态、PWM、巡线误差及各模块结果组织成 JSONL。
// 同一份消息用于终端、退出时的日志；PWM 是控制量，不是实际车速。
std::string json(const Command& c,double t,bool motion){
    std::ostringstream s;s<<"{\"t\":"<<t<<",\"state\":\""<<c.state<<"\",\"motor_pwm\":"<<c.motor<<",\"motion_enabled\":"<<(motion?1:0)<<",\"servo_pwm\":"<<c.steer<<",\"target_pwm\":"<<c.target<<",\"error\":"<<c.error<<",\"lane_error\":"<<c.baseError<<",\"left\":"<<c.left<<",\"right\":"<<c.right<<",\"cone\":\""<<c.cone<<"\",\"offset\":"<<c.offset<<",\"passed\":"<<c.passed<<",\"cone_target_id\":"<<c.coneTarget<<",\"cone_next_id\":"<<c.coneNext<<",\"cone_tracks\":"<<c.coneTracks<<",\"cone_side\":"<<c.coneSide<<",\"cone_handoff\":"<<c.coneHandoff<<",\"light\":"<<c.light.observation.state<<",\"light_mode\":\""<<c.light.mode<<"\",\"light_phase\":\""<<c.light.phase<<"\",\"light_confirmed\":"<<c.light.observation.confirmed<<",\"light_evidence\":\""<<c.light.observation.evidence<<"\",\"light_sequence\":\""<<c.light.sequence<<"\",\"light_sequence_remaining_s\":"<<c.light.sequenceRemaining<<",\"light_red_wait_remaining_s\":"<<c.light.redWaitRemaining<<",\"light_release_reason\":\""<<c.light.releaseReason<<"\""<<",\"light_detection_ran\":"<<c.light.detectionRan<<",\"light_detection_calls\":"<<c.light.detectionCalls<<",\"red_pixels\":"<<c.light.observation.red<<",\"green_pixels\":"<<c.light.observation.green<<",\"zebra\":\""<<c.zebra<<"\",\"zebra_y\":"<<c.zebraY<<",\"parking\":\""<<c.parking<<"\",\"parking_armed\":"<<c.parkingArmed<<",\"frame_dt_s\":"<<c.frameDt<<",\"processing_ms\":"<<c.processingMs<<",\"parking_side\":"<<c.parkingSide<<",\"parking_panel_checked\":"<<c.panelChecked<<",\"parking_panel_found\":"<<c.panelFound<<",\"parking_panel_hits\":"<<c.panelHits<<",\"parking_side_name\":\""<<(c.parkingSide>0?"RIGHT":c.parkingSide<0?"LEFT":"UNSELECTED")<<"\",\"parking_path\":\""<<c.parkingPath<<"\""<<",\"cone_cruise_remaining_s\":"<<c.coneCruiseRemaining<<",\"parking_cruise_remaining_s\":"<<c.parkingCruiseRemaining<<",\"race_stage\":\""<<c.raceStage<<"\",\"start_board_confirmed\":"<<c.startBoardConfirmed<<",\"start_clear_hits\":"<<c.startClearHits<<"}\n";return s.str();
}
int main(int argc,char** argv){try{
    std::string env=LaneBackend::name(),config,mod="lane",image,video,out="runs/latest",exposure,device;
    bool motion=false,show=false,masks=false,check=false,rawOnly=false,info=false,checkHardware=false,lightNight=false;int frames=0,exposureValue=-1;double runSeconds=0;
    // 单一程序build/race；夜间开关只改变light模块的识别器。
    // 无值开关直接设置布尔量，其余选项消费后一个参数；非法名称或缺值会报错。
    // --masks 同时开启窗口，--camera-only 只查看原图；默认 motion=false。
    for(int i=1;i<argc;++i){std::string a=argv[i];
        if(a=="--enable-motion")motion=true;else if(a=="--show")show=true;else if(a=="--masks")masks=show=true;
        else if(a=="--light-night")lightNight=true;
        else if(a=="--check-hardware")checkHardware=true;else if(a=="--check-config")check=true;else if(a=="--camera-only")rawOnly=show=true;else if(a=="--camera-info")info=true;
        else if(a=="--help"){
            std::cout<<"Race (environment fixed by this folder): --modules lane,light,cone,zebra,parking3,parking4 (all = lane,zebra,light,cone,parking4)\n"
                     <<"Full course: zebra -> timed lane/stop -> lane cone_after_light_seconds -> cone -> lane parking_after_cones_seconds -> parking4; no light detection.\n"
                     <<"--light-night (night detector for visual light mode; ignored by all's default fixed timer)\n"
                     <<"--show --masks --enable-motion \n"
                     <<"--camera-only (raw window, NO GPIO) --camera-info --exposure auto|manual --exposure-value N --device PATH\n"
                     <<"--config YAML --check-config --check-hardware --image FILE --replay VIDEO --frames N --out RELATIVE_PREFIX\n--run-seconds N (optional wall-clock limit; neutral is sent even if camera processing stalls)\n";return 0;
        }else{
            if(i+1==argc)throw std::runtime_error("Missing option value: "+a);std::string v=argv[++i];
            if(a=="--env")env=v;else if(a=="--config")config=v;else if(a=="--modules")mod=v;else if(a=="--image")image=v;else if(a=="--replay")video=v;
            else if(a=="--out")out=v;else if(a=="--exposure")exposure=v;else if(a=="--device")device=v;
            else if(a=="--run-seconds"){size_t used=0;runSeconds=std::stod(v,&used);if(used!=v.size()||!std::isfinite(runSeconds)||runSeconds<=0||runSeconds>600)throw std::runtime_error("--run-seconds must be in (0,600]");}
            else if(a=="--frames"||a=="--exposure-value"){size_t used=0;int n=std::stoi(v,&used);if(used!=v.size()||n<1)throw std::runtime_error("Invalid number: "+v);if(a=="--frames")frames=n;else exposureValue=n;}
            else throw std::runtime_error("Unknown option: "+a);
        }
    }
    if(env!=LaneBackend::name())throw std::runtime_error("Unknown environment");
    if(!exposure.empty()&&exposure!="auto"&&exposure!="manual")throw std::runtime_error("Exposure must be auto or manual");
    // 配置优先级：加载本目录的环境 YAML；停车只有parking3，相机选项仍可由命令行覆盖。
    // --config 指定了文件时，不再自动使用 config/<env>.yaml。
    if(config.empty())config="config/"+env+".yaml";
    SuiteConfig c;c.load(config);
    Modules modules(mod);
    const bool fixedLight=modules.fullCourse()&&c.fixedLight.enabled;
    c.lightNight=lightNight&&!fixedLight;if(c.lightNight)TrafficRecognizer::checkNightConfig();
    if(!device.empty())c.drive.camera_device=device;
    if(exposureValue>0)c.drive.set_exposure=exposureValue;
    if(!exposure.empty())c.drive.set_auto_exposure=exposure=="manual"?1:3;
    std::cout<<"CONFIG version=race env="<<env<<" lane="<<c.backend<<" modules="<<modules.str()<<" parking_mode="<<(modules.parking4?"parking4":modules.parking3?"parking3":"disabled")<<" cone_mode="<<"visual_shape"<<" watchdog="<<(c.drive.control_watchdog_enabled?"ON":"OFF")<<" file="<<config<<" race_sequence="<<(modules.fullCourse()?"ordered_wait_board":"independent");
    if(modules.light)std::cout<<" light_mode="<<(fixedLight?"fixed_timed":c.lightNight?"night_v2":"signal_color");
    if(fixedLight)std::cout<<" light_detection=OFF cruise_seconds="<<c.fixedLight.cruiseSeconds
        <<" cruise_pwm="<<c.fixedLight.cruisePwm<<" cruise_steering=lane"
        <<" brake_pwm="<<c.light.brakePwm<<" brake_seconds="<<c.light.brakeSeconds
        <<" stop_seconds="<<c.fixedLight.stopSeconds<<" cone_delay_seconds="<<c.coneAfterLightSeconds;
    if(modules.fullCourse())std::cout<<" parking_delay_seconds="<<c.parkingAfterConesSeconds;
    std::cout<<std::endl;
    // 配置检查在打开相机、创建 Output 之前返回，只读取和校验参数。
    if(check)return 0;
    if(checkHardware){
        if(motion)throw std::runtime_error("--check-hardware does not drive; omit --enable-motion");
#ifdef RACE_PI
        MotionGuard guard;std::cout<<"HARDWARE_CHECK_OK no active owner, no residual DMA5/PWM; no GPIO output sent"<<std::endl;return 0;
#else
        throw std::runtime_error("Hardware preflight requires the Raspberry Pi build");
#endif
    }
    // 运动模式只允许树莓派编译版本使用实时相机；图片、回放及相机工具拒绝运动。
    // 这里仅检查运行组合；是否真正初始化 GPIO 仍由 Output 的 motion 参数决定。
    if((motion&&(rawOnly||info||!image.empty()||!video.empty())))throw std::runtime_error("Motion is only allowed with live camera driving in indoor/outdoor");
#ifndef RACE_PI
    if(motion)throw std::runtime_error("Desktop build has NO GPIO; use build_pi.sh on the car");
#endif
    if((!image.empty())+(!video.empty())>1)throw std::runtime_error("Choose one input source");
    if((rawOnly||info)&&(!image.empty()||!video.empty()))throw std::runtime_error("Camera tools require a real camera");
    if(show&&!getenv("DISPLAY"))throw std::runtime_error("Window requires local desktop or SSH X11 forwarding (DISPLAY)");
    install_signal_handlers();cv::setNumThreads(2);
    // 输出前缀必须是项目内的相对路径。终端打印由独立线程处理，避免慢终端拖住控制。
    // 输入来源互斥：单张图片、视频文件或实时 V4L2 相机。
    parent_dirs(out);AsyncConsole console;cv::VideoCapture cap;cv::Mat frame;
    std::unique_ptr<CameraControls> controls;
    if(!image.empty()){frame=cv::imread(image);if(frame.empty())throw std::runtime_error("Image unreadable");cv::resize(frame,frame,{640,480});}
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
    // Output后构造先析构：异常退出也先停止输出，再结束其余对象。
    Suite suite(c,modules);Output output(c.drive,motion,false,runSeconds);auto start=Clock::now();int n=0;Command command;std::deque<std::string> records;std::string previousState,previousZebra;bool zebraVoicePlayed=false;
    ParkingPhoto parkingPhoto(c.parkingPhotoDelay);
    const std::string parkingPhotoPath=out+"_parked_"+std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count())+"_"+std::to_string(getpid())+".jpg";
    // 主循环顺序：取帧 → 算法与任务仲裁 → 输出命令 → 诊断显示。
    // phase 标记当前处理阶段，供控制超时时定位卡顿位置。
    while(!interrupted&&!output.aborted){
        output.phase=1;if(image.empty()&&!cap.read(frame))break;if(frame.empty())break;
        if(frame.size()!=cv::Size(640,480)){if(!motion&&!video.empty())cv::resize(frame,frame,{640,480});else throw std::runtime_error("Expected 640x480 camera frame");}
        // 任务时间单位统一为秒：回放按视频 FPS，单图按 30 FPS。
        // 实车使用 steady_clock 的已运行时长；这些时间用于制动、等灯、等待和绕桶状态。
        double now=!video.empty()?n/std::max(1.,cap.get(cv::CAP_PROP_FPS)):!image.empty()?n/30.:std::chrono::duration<double>(Clock::now()-start).count();
        // 只有非纯相机模式才运行 Suite；send 接收已经完成优先级仲裁的一条电机/舵机命令。
        // 日志每 10 帧或状态变化时提交终端，完整的有界记录在结束后落盘。
        if(!rawOnly){output.phase=2;auto processingStart=Clock::now();
            try{command=suite.step(frame,now);}catch(...){output.finish();throw;}
            command.processingMs=std::chrono::duration<double,std::milli>(Clock::now()-processingStart).count();output.phase=4;if(!output.send(command.motor,command.steer,command.state,command.brakeRemaining))break;
            try{
                if(parkingPhoto.update(frame,now,command.state,command.motor,c.drive.neutral_pwm,parkingPhotoPath))
                    std::cout<<"[PARK_PHOTO] saved="<<parkingPhotoPath<<" motion_enabled="<<motion<<std::endl;
            }catch(const std::exception& e){std::cerr<<"[PARK_PHOTO_ERROR] "<<e.what()<<std::endl;}
            if(modules.zebra&&!zebraVoicePlayed&&previousZebra!="HOLD"&&command.zebra=="HOLD"){
                zebraVoicePlayed=true;
                if(motion){pid_t child=fork();
                    if(child==0){execl("/usr/bin/aplay","aplay","-q","-D","plughw:1,0","audio/parking_announcement.wav",(char*)nullptr);_exit(127);}
                    std::cout<<"[ZEBRA_AUDIO] hold_started child="<<child<<std::endl;
                }else std::cout<<"[ZEBRA_AUDIO] hold_started preview_only"<<std::endl;
            }
            previousZebra=command.zebra;
            auto message=json(command,now,motion);if(records.size()>=4096)records.pop_front();records.push_back(message);if(n%10==0||command.state!=previousState)console.publish(message);previousState=command.state;
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
    const char* stopReason=interrupted?"signal":output.aborted==4?"run_limit":output.aborted==2?"watchdog":output.aborted==5?"io_error":"normal_end";
    std::cout<<"FINAL neutral="<<c.drive.neutral_pwm<<" frames="<<n<<" watchdog_abort="<<output.aborted<<" signal="<<interrupted.load()<<" io_error="<<output.io_error.load()<<" duration_limit="<<(output.aborted==4?1:0)<<" stop_reason="<<stopReason<<std::endl;
    return output.aborted==2||output.aborted==5||output.io_error?2:0;
}catch(const std::exception& e){std::cerr<<"ERROR "<<e.what()<<std::endl;return 2;}}
