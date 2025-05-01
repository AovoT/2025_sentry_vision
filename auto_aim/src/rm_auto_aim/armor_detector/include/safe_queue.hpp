#pragma once
#include <condition_variable>
#include <mutex>
#include <queue>

template <typename T>
class SafeQueue
{
public:
  void push(T value)
  {
    std::lock_guard<std::mutex> lock(mtx);
    queue.push(std::move(value));
    cv.notify_one();
  }

  bool pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mtx);
    if (queue.empty()) return false;
    value = std::move(queue.front());
    queue.pop();
    return true;
  }

  bool waitAndPop(T & value, int timeout_ms = 1000)
  {
    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(
          lock, std::chrono::milliseconds(timeout_ms), [this] { return !queue.empty(); })) {
      return false;
    }
    value = std::move(queue.front());
    queue.pop();
    return true;
  }

private:
  std::queue<T> queue;
  mutable std::mutex mtx;
  std::condition_variable cv;
};
