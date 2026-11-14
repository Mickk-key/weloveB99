// 本机 TCP 仿真传输：接收压缩图像和帧时长，发送主循环生成的 JSON 控制结果。
// 只传视觉输入及时间，不读取仿真车辆位置；main.cpp 禁止此输入与运动模式同时启用。
#pragma once
#include <sys/socket.h>
#include <arpa/inet.h>
struct SimTransport {
    int server=-1,client=-1;double time=0;
    ~SimTransport(){if(client>=0)close(client);if(server>=0)close(server);}
    // 只监听回环地址 127.0.0.1，等待单个客户端；等待期间每秒检查退出信号。
    // 连接后设置 20 秒接收超时，套接字由析构关闭。
    void open(int port){
        if(port<1||port>65535)throw std::runtime_error("Invalid simulation port");
        server=socket(AF_INET,SOCK_STREAM,0);int yes=1;setsockopt(server,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));
        sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        if(bind(server,(sockaddr*)&a,sizeof(a))||listen(server,1))throw std::runtime_error("Simulation bind failed");
        std::cout<<"SIM_READY NO_GPIO"<<std::endl;
        while(!interrupted){fd_set fds;FD_ZERO(&fds);FD_SET(server,&fds);timeval wait{1,0};if(select(server+1,&fds,nullptr,nullptr,&wait)>0){client=accept(server,nullptr,nullptr);break;}}
        if(client<0)throw std::runtime_error("Simulation connection interrupted");timeval timeout{20,0};setsockopt(client,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
    }
    // TCP 可能分片到达，循环读取直到凑齐指定字节数；断开或读取失败则返回 false。
    bool receive(void* p,size_t size){char* b=(char*)p;while(size){auto n=recv(client,b,size,0);if(n<=0)return false;b+=n;size-=n;}return true;}
    // 一帧协议：网络字节序 uint32 图像字节数 + uint32 帧间隔毫秒 + 压缩图像数据。
    // 校验大小和时间范围后解码为 BGR，并将 dt/1000 累加为算法使用的秒级时间。
    bool read(cv::Mat& image){uint32_t n,dt;if(!receive(&n,4)||!receive(&dt,4))return false;n=ntohl(n);dt=ntohl(dt);if(n<20||n>4000000||dt<1||dt>1000)throw std::runtime_error("Invalid simulation packet");std::vector<uchar> bytes(n);if(!receive(bytes.data(),n))return false;image=cv::imdecode(bytes,cv::IMREAD_COLOR);time+=dt/1000.;return !image.empty();}
    // 循环发送完整 JSON 文本，处理部分发送；MSG_NOSIGNAL 避免断连时由 SIGPIPE 结束进程。
    void send(const std::string& data){size_t p=0;while(p<data.size()){auto n=::send(client,data.data()+p,data.size()-p,MSG_NOSIGNAL);if(n<=0)throw std::runtime_error("Simulation send failed");p+=n;}}
};
