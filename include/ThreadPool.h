#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <future>
#include <queue>
#include <type_traits>
#include <functional>

template<typename T>
class SafeQueue
{
public:
    SafeQueue() = default;

    ~SafeQueue() = default;

    SafeQueue(const SafeQueue& other) = delete;

    SafeQueue& operator = (const SafeQueue& other) = delete;

    SafeQueue(SafeQueue&& other) = delete;

    SafeQueue& operator = (const SafeQueue&& other) = delete;

    SafeQueue(const SafeQueue&& other) = delete;

    bool empty()
    {
        std::unique_lock<std::mutex> locker(m_Mutex);
        return m_Queue.empty();
    }

    int size()
    {
        std::unique_lock<std::mutex> locker(m_Mutex);
        return m_Queue.size();
    }

    void push(T& value)
    {
        std::unique_lock<std::mutex> locker(m_Mutex);
        m_Queue.emplace(value);
    }

    void push(T&& value)
    {
        std::unique_lock<std::mutex> locker(m_Mutex);
        m_Queue.emplace(std::move(value));
    }

    bool pop(T& value)
    {
        std::unique_lock<std::mutex> locker(m_Mutex);
        if(m_Queue.empty())
        {
            return false;
        }
        else
        {
            value = std::move(m_Queue.front());
            m_Queue.pop();
            return true;
        }
    }

private:
    std::queue<T> m_Queue;
    std::mutex m_Mutex;
};

//单任务队列 线程池
//提交的任务：普通函数，匿名函数，仿函数，类成员函数
//返回值不同，参数列表不同
class SimpleThreadPool {
    public:
        explicit SimpleThreadPool(size_t threads = std::thread::hardware_concurrency()) {
            for (size_t i = 0; i < threads; ++i)
                workers.emplace_back([this] { workerLoop(); });
        }
    
        template <typename Func, typename... Args>
        auto submitTask(Func&& func, Args&&... args) -> std::future<decltype(func(std::forward<Args>(args)...))> {
            using returnType = decltype(func(std::forward<Args>(args)...));
    
            auto task = std::make_shared<std::packaged_task<returnType()>>(
                [func = std::forward<Func>(func), args = std::make_tuple(std::forward<Args>(args)...)]() mutable {
                    return std::apply(func, std::move(args));
                }
            );
    
            std::future<returnType> result = task->get_future();
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                taskQueue.emplace([task]() { (*task)(); });
            }
            condition.notify_one();
            return result;
        }
    
        ~SimpleThreadPool() {
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                stop = true;
            }
            condition.notify_all();
            for (std::thread& worker : workers)
                worker.join();
        }
    
    private:
        std::vector<std::thread> workers;
        std::queue<std::function<void()>> taskQueue;
        std::mutex queueMutex;
        std::condition_variable condition;
        bool stop = false;
    
        void workerLoop() {
            while (true) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(queueMutex);
                    condition.wait(lock, [this] { return stop || !taskQueue.empty(); });
                    if (stop && taskQueue.empty()) return;
                    task = std::move(taskQueue.front());
                    taskQueue.pop();
                }
                task();
            }
        }
    };


#endif
