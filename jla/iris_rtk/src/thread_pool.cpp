//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-09-26
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <chrono>

#include "rtk/structs/concurrent_queue.h"
#include "rtk/thread_pool.h"

namespace rtk {

struct TaskQueue {
    using CQueue = ConcurrentQueue<std::function<void()>>;
    CQueue tasks;
};

} // namespace iris

void rtk::ThreadPool::init(size_t size)
{
    queue = new TaskQueue;
    for (size_t i = 0; i < size; ++i)
        workers.emplace_back(std::thread([this]{
            while (not stop.load(std::memory_order_relaxed)){
                if (not dequeue_task()){
                    ++sleep_count;
                    std::unique_lock<std::mutex> lock(queue_mutex);
                    queue_condition.wait_for(lock, std::chrono::milliseconds(32));
                    --sleep_count;
                }// end if
            }// end while
        }));
}

rtk::ThreadPool::~ThreadPool()
{
    stop = true;
    queue_condition.notify_all();

    for (auto& thread : workers)
        thread.join();

    delete queue;
}

void rtk::ThreadPool::enqueue(std::function<void()>&& function)
{
    ++tasks_to_complete;
    queue->tasks.enqueue(std::move(function));
    if (sleep_count > 0){
        queue_condition.notify_one();
    }
}

bool rtk::ThreadPool::dequeue_task()
{
    std::function<void()> task;
    if (queue->tasks.try_dequeue(task)){
        task();
        --tasks_to_complete;
        return true;
    }

    return false;
}

void rtk::ThreadPool::wait()
{
    while (tasks_to_complete)
        if (not dequeue_task())
            std::this_thread::yield();
}

