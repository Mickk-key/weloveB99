// 异步终端日志：控制线程只提交最新状态，工作线程负责可能阻塞的实际输出。
// 只保留一条待发消息，新消息覆盖旧消息，防止打印积压占用控制循环时间。
#pragma once
#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

// Console output must never run on the control thread. At most one unsent
// message is retained; a newer status replaces it while the sink is busy.
// finish() can wait for an already-running sink. Neutral the motor BEFORE
// calling finish() or destroying this object. A sink must not call finish().
class AsyncConsole {
public:
    using Sink=std::function<void(const std::string&)>;
    std::atomic<unsigned long> failures{0};

    explicit AsyncConsole(Sink sink=Sink())
        : sink_(sink?std::move(sink):Sink([](const std::string& s){std::cout<<s<<std::endl;})),
          worker_([this]{consume();}) {}

    AsyncConsole(const AsyncConsole&)=delete;
    AsyncConsole& operator=(const AsyncConsole&)=delete;

    // 持锁只完成消息交换，释放后通知工作线程；停止过程中直接忽略新消息。
    void publish(std::string message) {
        {
            std::lock_guard<std::mutex> guard(mutex_);
            if(stopping_)return;
            pending_=std::move(message);
            hasPending_=true;
        }
        ready_.notify_one();
    }

    // Idempotent; callers may finish explicitly before the destructor.
    // Pending messages are discarded, rather than delaying shutdown to print.
    // 丢弃尚未开始输出的消息并等待工作线程结束；正在执行的 sink 仍可能阻塞。
    // 因此 main.cpp 先 output.finish() 归中位，再调用 console.finish()。
    void finish() {
        std::lock_guard<std::mutex> finalizer(finishMutex_);
        {
            std::lock_guard<std::mutex> guard(mutex_);
            stopping_=true;
            hasPending_=false;
            pending_.clear();
        }
        ready_.notify_one();
        if(worker_.joinable())worker_.join();
    }

    ~AsyncConsole(){finish();}

private:
    Sink sink_;
    std::mutex mutex_,finishMutex_;
    std::condition_variable ready_;
    bool stopping_=false,hasPending_=false;
    std::string pending_;
    std::thread worker_;

    // 条件变量等待新消息，取走消息后释放队列锁，再进行终端 I/O 或自定义回调。
    // 回调异常只增加 failures 计数，不抛回控制线程。
    void consume() {
        for(;;) {
            std::string message;
            {
                std::unique_lock<std::mutex> guard(mutex_);
                ready_.wait(guard,[this]{return stopping_||hasPending_;});
                if(stopping_)return;
                message=std::move(pending_);
                hasPending_=false;
            }
            // No queue lock is held across slow I/O or user-provided callbacks.
            try{sink_(message);}
            catch(...){failures.fetch_add(1,std::memory_order_relaxed);}
        }
    }
};
