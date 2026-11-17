#pragma once
#include "core/types.h"
#include "nlohmann/json.hpp"
#include "core/cpu_executor.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
namespace gpaco {
namespace fs = std::filesystem;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using Range = std::unordered_map<std::string, Real>;
using Pool = CPUExecutor;
inline void require(bool condition, const std::string &message) {
    if (!condition)
        throw std::runtime_error("GP-ACO: " + message);
}
inline void check(bool condition, const std::string &message) {
    require(condition, message);
}
inline double seconds(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
inline uint64_t mix(uint64_t x) {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}
inline uint64_t instanceSeed(uint64_t root, int domain, int sample, int iteration, int ant) {
    for (int value : {domain, sample, iteration, ant})
        root = mix(root ^ mix(uint64_t(value)));
    return root;
}
inline uint64_t key(uint64_t root, int domain, int sample, int environment, int generation,
                    int ant) {
    for (int value : {domain, sample, environment, generation, ant})
        root = mix(root ^ uint64_t(value));
    return root;
}
inline uint64_t hashText(const std::string &text) {
    uint64_t hash = 14695981039346656037ULL;
    for (unsigned char c : text)
        hash = (hash ^ c) * 1099511628211ULL;
    return hash;
}
inline uint64_t routeHash(const Sol &solution) {
    return hashText(Json(solution.variable().m_truck_routes).dump());
}
inline size_t actionCount(const Routes &routes) {
    size_t count = 0;
    for (const auto &r : routes)
        count += r.size();
    return count;
}
std::string readText(const fs::path &path);
inline Json readJson(const fs::path &path) {
    return Json::parse(readText(path));
}
void atomicWrite(const fs::path &path, const std::string &text);
uint64_t fileHash(const fs::path &path);
class FileLock {
  public:
    explicit FileLock(const fs::path &path);
    ~FileLock();
    FileLock(const FileLock &) = delete;
    FileLock &operator=(const FileLock &) = delete;

  private:
    intptr_t m_handle = -1;
};
} // namespace gpaco
