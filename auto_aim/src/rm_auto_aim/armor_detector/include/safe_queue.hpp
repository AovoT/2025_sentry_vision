#pragma once
#include <condition_variable>
#include <mutex>
#include <queue>

template <typename T>
class LatestQueue
{
public:
  // 写入时只保留最新帧
  void push(T value)
  {
    std::lock_guard<std::mutex> lock(mtx);
    if (!queue.empty()) queue.pop();  // 丢弃旧帧
    queue.push(std::move(value));
    cv.notify_one();
  }

  // 非阻塞取出当前帧（如无则返回 false）
  bool pop(T & value)
  {
    std::lock_guard<std::mutex> lock(mtx);
    if (queue.empty()) return false;
    value = std::move(queue.front());
    queue.pop();
    return true;
  }

  // 阻塞等待一定时间取出最新帧
  bool waitAndPop(T & value, int timeout_ms = 1000)
  {
    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] {
          return !queue.empty();
        })) {
      return false;
    }
    value = std::move(queue.front());
    queue.pop();
    return true;
  }

  // 获取是否有新帧（可选）
  bool hasData() const
  {
    std::lock_guard<std::mutex> lock(mtx);
    return !queue.empty();
  }

private:
  std::queue<T> queue;
  mutable std::mutex mtx;
  std::condition_variable cv;
};
