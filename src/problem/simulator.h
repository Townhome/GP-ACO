#pragma once
#include "core/types.h"
namespace gpaco {
class TruckBase {
  public:
    using TruckIndType = Sol;
    explicit TruckBase(const ParamMap &params);
    void resetSolutionWithSeed(Sol *s, std::uint64_t seed);
    void resetSolutionImpl(Sol *s, std::mt19937_64 *deterministic_rng);
    int choseTruck(int sample, Sol *s);
    std::vector<int> updateMask(int sample, int truck, Sol *s);
    TruckInf getTruckInfo(int sample, int truck, int point, Sol *s);
    int searchGrade(int sample, int point, TruckInf &truck_info, Sol *s);
    void updateState(int sample, int truck, int point, Sol *s);
    void insertSequence(int sample, TruckInf &truck_info, Sol *s);
    Real evaluate(int sample, Sol *s);
    std::vector<std::vector<Real>>
    getHeuristicValue(int sample, int truck, const std::vector<int> &mask,
                      const std::unordered_map<std::string, Real> &data_range, Sol *s,
                      int feature_count = 16);
    Real acoHeuristicValue(int sample, int truck, int point, Sol *s);
    int getMaxRouteLength(int sample);
    int getNumTruck() const { return m_num_trucks; }
    int getNumShovel() const { return m_num_shovels; }
    int getNumUnload() const { return m_num_unloads; }
    int getNumNodes() const { return m_num_nodes; }
    int getNumSample() const { return m_num_samples; }
    Real getShiftTime() const { return m_shift_time; }
    int m_num_shovels, m_num_trucks, m_num_unloads, m_num_nodes, m_num_samples;
    Real m_max_quantity = 0;
    Real m_min_grade = 0;
    Real m_max_grade = 0;
    Real m_min_capacity = 0;
    Real m_max_capacity = 0;
    Real m_max_port = 0;
    Real m_max_time = 0; // max truck process time

    Real m_shift_time;
    Real m_target_grade;
    Real m_time_window;
    Real m_std_load_time;
    Real m_std_unload_time;
    std::vector<std::vector<int>> m_shovel_type;
    std::vector<std::vector<int>> m_shovel_radius;
    std::vector<std::vector<Real>> m_shovel_down_time;
    std::vector<std::vector<std::vector<std::vector<Real>>>> m_load_sequence;
    std::vector<std::vector<Real>> m_load_quantity;
    std::vector<std::vector<Real>> m_load_grade;
    std::vector<Real> m_load_quantity_total;
    std::vector<Real> m_load_grade_total;
    std::vector<std::vector<int>> m_shovel_flow_length;
    std::vector<std::vector<Real>> m_point_quantity;
    std::vector<std::vector<int>> m_unload_type;
    std::vector<std::vector<int>> m_unload_port;
    std::vector<std::vector<Real>> m_truck_capacity;
    std::vector<std::vector<int>> m_truck_type;
    std::vector<std::vector<Real>> m_truck_unload_time;
    std::vector<std::vector<int>> m_truck_port;
    std::vector<std::vector<Real>> m_truck_down_time;
    std::vector<std::vector<int>> m_driver_type;
    std::vector<std::vector<std::vector<std::vector<std::vector<std::vector<Real>>>>>>
        m_road_predict;
    std::vector<std::vector<std::vector<std::vector<std::vector<std::vector<Real>>>>>>
        m_road_predict_d;
    std::vector<std::vector<std::vector<Real>>> m_load_time;
    std::vector<std::vector<Real>> m_time_range;
    std::vector<Real> m_event_time;

    std::vector<std::vector<std::vector<NormalRandomGenerator>>> m_load_time_normal;
    std::vector<std::vector<NormalRandomGenerator>> m_unload_time_normal;
    std::unordered_map<std::string, Real> getDataRange() {
        return {
            {"max_quantity", m_max_quantity}, {"min_grade", m_min_grade},
            {"max_grade", m_max_grade},       {"min_capacity", m_min_capacity},
            {"max_capacity", m_max_capacity}, {"max_port", m_max_port},
            {"max_time", m_max_time},         {"max_shift", m_shift_time},
        };
    }
};
} // namespace gpaco
