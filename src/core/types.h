#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>
namespace gpaco {
using Real = float;
using ParamMap = std::unordered_map<std::string, std::variant<int, bool, Real, std::string>>;
class NormalRandomGenerator {
  public:
    NormalRandomGenerator(unsigned seed = 42, Real mean = 0.0, Real stddev = 1.0)
        : seed_(seed), mean_(mean), stddev_(stddev) {
        reset();
    }

    Real operator()() { return dist_(engine_); }

    void reset() {
        engine_.seed(seed_);
        dist_.param(std::normal_distribution<Real>::param_type(mean_, stddev_));
    }

    void setSeed(unsigned new_seed) {
        seed_ = new_seed;
        reset();
    }

    void setParam(Real mean, Real stddev) {
        mean_ = mean;
        stddev_ = stddev;
        reset();
    }

    void setStd(Real stddev) {
        stddev_ = stddev;
        reset();
    }

  private:
    unsigned seed_;
    Real mean_, stddev_;
    std::mt19937 engine_;
    std::normal_distribution<Real> dist_;
};
struct TruckInf {
    int truck;
    int point;
    int shovelPoint;
    Real pointProgress;
    Real progress;
    Real arriveTime;
    Real workTime;
    Real leaveTime;
    Real ore;
    Real grade;
    Real kaolin;
    Real fines;
    Real hardness;
};
class TruckEncoding {
  public:
    std::vector<std::vector<int>> m_truck_routes;
    std::vector<std::vector<int>> m_truck_map;
    std::vector<std::vector<TruckInf>> m_node_truck_sequence;
    std::vector<std::vector<Real>> m_arrive_time;
    std::vector<std::vector<std::vector<Real>>> m_cur_load_sequence;
    std::vector<int> m_cur_shovel_flow_length;

    std::vector<std::vector<NormalRandomGenerator>> m_load_time_normal;
    std::vector<NormalRandomGenerator> m_unload_time_normal;

    int m_sample_id = -1;
    std::vector<int> m_truck_action;
};
class Solution {
    TruckEncoding state;
    Real value = std::numeric_limits<Real>::infinity();

  public:
    TruckEncoding &variable() { return state; }
    const TruckEncoding &variable() const { return state; }
    void setFitness(Real fitness) { value = fitness; }
    Real fitness() const { return value; }
};
using Sol = Solution;
using Ptr = std::shared_ptr<Sol>;
using Routes = std::vector<std::vector<int>>;
using Matrix = std::vector<std::vector<std::vector<Real>>>;
} // namespace gpaco
