#ifndef GPACO_CPU_EXECUTOR_H
#define GPACO_CPU_EXECUTOR_H
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
namespace gpaco {
// One submitting thread; tasks must not recursively submit to this executor.
// Reuse workers across all generation phases, with a barrier between phases.
class CPUExecutor {
  public:
    explicit CPUExecutor(int threads)
        : m_workers(threads > 0
                        ? static_cast<size_t>(threads)
                        : static_cast<size_t>(std::max(1u, std::thread::hardware_concurrency()))) {
        if (threads < 0)
            throw std::invalid_argument("CPU threads must be non-negative");
        if (m_workers == 1)
            return;
        try {
            for (size_t i = 0; i < m_workers; ++i)
                m_pool.emplace_back([this] { worker(); });
        } catch (...) {
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_stop = true;
            }
            m_ready.notify_all();
            for (auto &thread : m_pool)
                thread.join();
            throw;
        }
    }
    ~CPUExecutor() {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_stop = true;
        }
        m_ready.notify_all();
        for (auto &thread : m_pool)
            thread.join();
    }
    CPUExecutor(const CPUExecutor &) = delete;
    CPUExecutor &operator=(const CPUExecutor &) = delete;
    size_t workers() const { return m_workers; }
    void run(size_t count, const std::function<void(size_t)> &work, size_t grain = 1) {
        if (!grain)
            throw std::invalid_argument("CPU task grain must be positive");
        if (!count)
            return;
        if (m_workers == 1 || count == 1) {
            for (size_t i = 0; i < count; ++i)
                work(i);
            return;
        }
        std::unique_lock<std::mutex> lock(m_mutex);
        m_work = work;
        m_count = count;
        m_grain = grain;
        m_next = 0;
        m_failed = false;
        m_failure = nullptr;
        m_remaining = m_pool.size();
        ++m_epoch;
        m_ready.notify_all();
        m_done.wait(lock, [this] { return m_remaining == 0; });
        m_work = {};
        if (m_failure)
            std::rethrow_exception(m_failure);
    }

  private:
    void worker() {
        size_t seen = 0;
        for (;;) {
            {
                std::unique_lock<std::mutex> lock(m_mutex);
                m_ready.wait(lock, [&] { return m_stop || m_epoch != seen; });
                if (m_stop)
                    return;
                seen = m_epoch;
            }
            while (!m_failed.load()) {
                const size_t index = m_next.fetch_add(m_grain, std::memory_order_relaxed);
                if (index >= m_count)
                    break;
                try {
                    const size_t end = index + std::min(m_grain, m_count - index);
                    for (size_t job = index; job < end && !m_failed.load(std::memory_order_relaxed);
                         ++job)
                        m_work(job);
                } catch (...) {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    if (!m_failure)
                        m_failure = std::current_exception();
                    m_failed = true;
                }
            }
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                if (--m_remaining == 0)
                    m_done.notify_one();
            }
        }
    }
    size_t m_workers, m_count = 0, m_remaining = 0, m_epoch = 0, m_grain = 1;
    std::vector<std::thread> m_pool;
    std::mutex m_mutex;
    std::condition_variable m_ready, m_done;
    std::atomic<size_t> m_next{0};
    std::atomic<bool> m_failed{false};
    bool m_stop = false;
    std::exception_ptr m_failure;
    std::function<void(size_t)> m_work;
};
} // namespace gpaco
#endif
