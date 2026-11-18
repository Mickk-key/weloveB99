#pragma once
#ifdef RACE_PI
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/mman.h>
#include <unistd.h>

// No GPIO initialization or register writes are allowed during preflight.
class ReadOnlyRegisters {
    int fd=-1;
    void* page=MAP_FAILED;
public:
    ReadOnlyRegisters(const char* device, off_t offset) {
        fd=open(device,O_RDONLY|O_SYNC|O_CLOEXEC);
        if(fd<0)throw std::runtime_error(std::string("Cannot inspect ")+device+"; run with sudo");
        page=mmap(nullptr,4096,PROT_READ,MAP_SHARED,fd,offset);
        if(page==MAP_FAILED){close(fd);fd=-1;throw std::runtime_error("Cannot map GPIO preflight registers");}
    }
    uint32_t read(unsigned offset) const {
        // MMIO needs scalar volatile accesses, not memcpy / SIMD loads.
        return *reinterpret_cast<volatile uint32_t*>(static_cast<unsigned char*>(page)+offset);
    }
    ~ReadOnlyRegisters(){if(page!=MAP_FAILED)munmap(page,4096);if(fd>=0)close(fd);}
    ReadOnlyRegisters(const ReadOnlyRegisters&)=delete;
    ReadOnlyRegisters& operator=(const ReadOnlyRegisters&)=delete;
};

class MotionGuard {
    int lock_fd=-1;
    static void check_owners() {
        DIR* proc=opendir("/proc");
        if(!proc)throw std::runtime_error("Cannot inspect GPIO owners");
        std::string conflict;
        while(auto* entry=readdir(proc)) {
            int pid=std::atoi(entry->d_name);
            if(pid<=0 || pid==getpid())continue;
            std::ifstream maps(std::string("/proc/")+entry->d_name+"/maps");
            std::string line;
            while(std::getline(maps,line)) {
                if(line.find("/dev/mem")==std::string::npos && line.find("/dev/gpiomem")==std::string::npos)continue;
                std::istringstream fields(line);std::string address,permissions;
                fields>>address>>permissions;
                if(permissions.size()>1 && permissions[1]=='w') {
                    conflict="GPIO controller/memory owner is still active (PID "+std::to_string(pid)+"). Stop it before driving.";
                    break;
                }
            }
            if(!conflict.empty())break;
        }
        closedir(proc);
        if(!conflict.empty())throw std::runtime_error(conflict);
    }
    static void check_residual_output() {
        std::ifstream model_file("/proc/device-tree/model");
        std::string model((std::istreambuf_iterator<char>(model_file)),{});
        if(model.find("Raspberry Pi 4")==std::string::npos)
            throw std::runtime_error("GPIO conflict preflight is validated for this Raspberry Pi 4 only");
        // network-rc/rpio-pwm uses DMA5 on this board; pigpio uses DMA7/6.
        // An orphan DMA5 may be paused and resume when pigpio reconfigures PCM.
        ReadOnlyRegisters dma("/dev/mem",0xfe007000u);
        if((dma.read(0x500)&1u) || dma.read(0x504))
            throw std::runtime_error("Residual DMA5 output detected. No motor signal sent. Stop the old controller cleanly; if orphaned, power down the motor and restart the Pi.");
        ReadOnlyRegisters gpio("/dev/gpiomem",0);
        uint32_t previous=gpio.read(0x34)&0x3000u;
        auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(80);
        while(std::chrono::steady_clock::now()<end) {
            uint32_t current=gpio.read(0x34)&0x3000u;
            if(current!=previous)
                throw std::runtime_error("GPIO12/13 are already toggling. No motor signal sent; resolve the existing PWM source first.");
            previous=current;
        }
    }
public:
    MotionGuard() {
        lock_fd=open("build/motion.lock",O_CREAT|O_RDWR|O_CLOEXEC,0660);
        if(lock_fd<0)throw std::runtime_error("Cannot open build/motion.lock; run from this project directory");
        if(flock(lock_fd,LOCK_EX|LOCK_NB)<0){close(lock_fd);lock_fd=-1;throw std::runtime_error("Another clean-v3 motion process already holds the motor lock");}
        try{check_owners();check_residual_output();}
        catch(...){flock(lock_fd,LOCK_UN);close(lock_fd);lock_fd=-1;throw;}
    }
    ~MotionGuard(){if(lock_fd>=0){flock(lock_fd,LOCK_UN);close(lock_fd);}}
    MotionGuard(const MotionGuard&)=delete;
    MotionGuard& operator=(const MotionGuard&)=delete;
};
#endif
