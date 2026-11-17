#include "problem/simulator.h"
#include "gp/tree_nodes.h"
#include <set>
#include <fstream>
#include <sstream>
#include <cfloat>
#include <numeric>
#include <stdexcept>
namespace gpaco {
std::vector<std::string> split(const std::string &str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}

template <typename T>
std::vector<int> calculateIndices(const std::vector<T> &list1, const std::vector<T> &list2) {
    std::unordered_map<T, int> position_map;
    for (size_t index = 0; index < list2.size(); ++index) {
        position_map[list2[index]] = index;
    }

    std::vector<int> modified_list1;
    for (auto &item : list1) {
        if (position_map.find(item) != position_map.end()) {
            modified_list1.push_back(position_map[item]);
        } else {
            modified_list1.push_back(-1);
        }
    }

    return modified_list1;
}

template <typename T>
std::pair<std::vector<int>, std::vector<T>> calculateTypes(const std::vector<T> &type_list) {
    std::vector<T> list_copy = type_list;

    std::set<T> unique_set(type_list.begin(), type_list.end());
    std::vector<T> sorted_list(unique_set.begin(), unique_set.end());

    auto modified_positions = calculateIndices(list_copy, sorted_list);

    return {modified_positions, sorted_list};
}

TruckBase::TruckBase(const ParamMap &params) {

    m_num_shovels = std::get<int>(params.at("ShovelSize"));
    m_num_trucks = std::get<int>(params.at("TruckSize"));
    m_num_unloads = std::get<int>(params.at("UnloadSize"));

    m_num_samples = 0;

    m_min_grade = std::numeric_limits<Real>::max();
    m_min_capacity = std::numeric_limits<Real>::max();

    int sample = 0;
    auto &filename = std::get<std::string>(params.at("DataFile"));
    int offset = 0;

    std::ifstream file(filename);
    std::string line;
    int instance = 0, road = 0, road_d = 0;
    std::string data_type = "";

    std::vector<Real> shovels, trucks, drivers;
    std::vector<std::string> unloads;
    std::vector<std::vector<unsigned>> seeds;
    while (std::getline(file, line)) {
        if (line.find("=") != std::string::npos) {
            instance++;
            if (instance > offset)
                sample++;
            data_type = "";
            road = 0;
            road_d = 0;
            shovels.clear();
            trucks.clear();
            drivers.clear();
            unloads.clear();
            continue;
        }
        if (instance < offset)
            continue;

        if (line.find("-") != std::string::npos) {
            data_type = line;
            continue;
        } else {
            std::vector<std::string> data = split(line, ' ');

            if (data_type.find("parameters") != std::string::npos) {
                m_shift_time = std::stod(data[0]);
                m_target_grade = std::stod(data[1]);
                m_time_window = std::stod(data[2]);
                m_event_time.push_back(std::stod(data[3]));
                m_std_load_time = std::stod(data[4]);
                m_std_unload_time = std::stod(data[5]);

            } else if (data_type.find("seeds") != std::string::npos) {
                seeds.push_back(std::vector<unsigned>());
                for (auto &seed : data)
                    seeds.back().push_back(static_cast<unsigned>(std::stoul(seed)));
            } else if (data_type.find("shovels") != std::string::npos) {
                if (sample >= m_shovel_radius.size()) {
                    m_shovel_radius.push_back({});
                    m_shovel_down_time.push_back({});
                }
                m_shovel_radius.back().push_back(std::stod(data[3]));
                m_shovel_down_time.back().push_back(std::stod(data[4]));

                shovels.push_back(std::stod(data[1]));
                if (shovels.size() == m_num_shovels) {
                    std::pair<std::vector<int>, std::vector<Real>> result = calculateTypes(shovels);
                    shovels = result.second;
                    m_shovel_type.push_back(result.first);
                }
            } else if (data_type.find("load sequence") != std::string::npos) {
                if (sample >= m_load_sequence.size()) {
                    m_load_sequence.push_back({});
                    m_shovel_flow_length.push_back(std::vector<int>(m_num_shovels, 0));
                }
                int shovel_index = std::stoi(data[0]);

                if (shovel_index >= m_load_sequence.back().size()) {
                    m_load_sequence.back().push_back({});
                }

                Real quantity = std::stod(data[1]);
                Real grade = std::stod(data[2]);
                m_load_sequence.back()[shovel_index].push_back({quantity, grade});
                m_shovel_flow_length.back()[shovel_index]++;

                if (quantity > m_max_quantity)
                    m_max_quantity = quantity;
                if (grade > m_max_grade)
                    m_max_grade = grade;
                if (grade < m_min_grade)
                    m_min_grade = grade;
            } else if (data_type.find("unloads") != std::string::npos) {
                if (sample >= m_unload_type.size()) {
                    m_unload_type.push_back({});
                    m_unload_port.push_back({});
                }
                int port = std::stoi(data[3]);
                unloads.push_back(data[2]);
                m_unload_port.back().push_back(port);

                if (port > m_max_port)
                    m_max_port = port;

                if (unloads.size() == m_num_unloads) {
                    m_unload_type.back() = calculateIndices(unloads, {"ore", "rock", "soil"});
                    m_point_quantity.push_back({});

                    Real ore = 0, rock = 0;
                    for (size_t i = 0; i < m_num_shovels; i++) {
                        Real total = 0;
                        for (auto &point : m_load_sequence.back()[i]) {
                            if (point[1] > 0.3)
                                ore += point[0];
                            else
                                rock += point[0];
                            total += point[0];
                        }
                        m_point_quantity.back().push_back(total);
                    }

                    int ore_unload_num = 0, rock_unload_num = 0;
                    for (auto &unload : m_unload_type.back()) {
                        if (unload == 0)
                            ore_unload_num++;
                        else
                            rock_unload_num++;
                    }

                    for (auto &unload : m_unload_type.back()) {
                        if (unload == 0)
                            m_point_quantity.back().push_back(ore / ore_unload_num);
                        else
                            m_point_quantity.back().push_back(rock / rock_unload_num);
                    }
                }
            } else if (data_type.find("trucks") != std::string::npos) {
                if (sample >= m_truck_capacity.size()) {
                    m_truck_capacity.push_back({});
                    m_truck_type.push_back({});
                    m_truck_unload_time.push_back({});
                    m_truck_port.push_back({});
                    m_driver_type.push_back({});
                    m_truck_down_time.push_back({});
                }

                Real capacity = std::stod(data[1]);
                Real driver = std::stod(data[2]);
                Real unload_time = std::stod(data[3]);
                m_truck_capacity.back().push_back(capacity);
                m_truck_unload_time.back().push_back(unload_time);
                m_truck_port.back().push_back(std::stoi(data[4]));
                m_truck_down_time.back().push_back(std::stod(data[5]));
                trucks.push_back(capacity);
                drivers.push_back(driver);

                if (capacity > m_max_capacity)
                    m_max_capacity = capacity;
                if (capacity < m_min_capacity)
                    m_min_capacity = capacity;
                if (unload_time > m_max_time)
                    m_max_time = unload_time;

                if (trucks.size() == m_num_trucks) {
                    std::pair<std::vector<int>, std::vector<Real>> truck_result =
                        calculateTypes(trucks);
                    std::pair<std::vector<int>, std::vector<Real>> driver_result =
                        calculateTypes(drivers);
                    m_truck_type.back() = truck_result.first;
                    m_driver_type.back() = driver_result.first;
                    trucks = truck_result.second;
                    drivers = driver_result.second;
                }
            } else if (data_type.find("roads") != std::string::npos) {
                if (sample >= m_road_predict.size())
                    m_road_predict.push_back(
                        std::vector<std::vector<std::vector<std::vector<std::vector<Real>>>>>(
                            m_num_shovels,
                            std::vector<std::vector<std::vector<std::vector<Real>>>>(
                                m_num_unloads,
                                std::vector<std::vector<std::vector<Real>>>(
                                    trucks.size(), std::vector<std::vector<Real>>(
                                                       drivers.size(), std::vector<Real>(4, 0))))));

                int load_index = road / m_num_unloads;
                int unload_index = road % m_num_unloads;

                for (size_t i = 0; i < data.size(); i++) {
                    std::vector<std::string> times = split(data[i], ',');
                    int truck_index = i / drivers.size();
                    int driver_index = i % drivers.size();
                    Real heavy_time = std::stod(times[0]);
                    Real empty_time = std::stod(times[1]);
                    Real heavy_oil = std::stod(times[2]);
                    Real empty_oil = std::stod(times[3]);
                    m_road_predict.back()[load_index][unload_index][truck_index][driver_index][0] =
                        heavy_time;
                    m_road_predict.back()[load_index][unload_index][truck_index][driver_index][1] =
                        empty_time;
                    m_road_predict.back()[load_index][unload_index][truck_index][driver_index][2] =
                        heavy_oil;
                    m_road_predict.back()[load_index][unload_index][truck_index][driver_index][3] =
                        empty_oil;
                    if (heavy_time > m_max_time)
                        m_max_time = heavy_time;
                    if (empty_time > m_max_time)
                        m_max_time = empty_time;
                }

                road++;
            } else if (data_type.find("event") != std::string::npos) {
                if (sample >= m_road_predict_d.size())
                    m_road_predict_d.push_back(
                        std::vector<std::vector<std::vector<std::vector<std::vector<Real>>>>>(
                            m_num_shovels,
                            std::vector<std::vector<std::vector<std::vector<Real>>>>(
                                m_num_unloads,
                                std::vector<std::vector<std::vector<Real>>>(
                                    trucks.size(), std::vector<std::vector<Real>>(
                                                       drivers.size(), std::vector<Real>(4, 0))))));

                int load_index = road_d / m_num_unloads;
                int unload_index = road_d % m_num_unloads;

                for (size_t i = 0; i < data.size(); i++) {
                    std::vector<std::string> times = split(data[i], ',');
                    int truck_index = i / drivers.size();
                    int driver_index = i % drivers.size();
                    Real heavy_time = std::stod(times[0]);
                    Real empty_time = std::stod(times[1]);
                    Real heavy_oil = std::stod(times[2]);
                    Real empty_oil = std::stod(times[3]);
                    m_road_predict_d
                        .back()[load_index][unload_index][truck_index][driver_index][0] =
                        heavy_time;
                    m_road_predict_d
                        .back()[load_index][unload_index][truck_index][driver_index][1] =
                        empty_time;
                    m_road_predict_d
                        .back()[load_index][unload_index][truck_index][driver_index][2] = heavy_oil;
                    m_road_predict_d
                        .back()[load_index][unload_index][truck_index][driver_index][3] = empty_oil;
                    if (heavy_time > m_max_time)
                        m_max_time = heavy_time;
                    if (empty_time > m_max_time)
                        m_max_time = empty_time;
                }

                road_d++;
            } else if (data_type.find("load time") != std::string::npos) {
                if (sample >= m_load_time.size())
                    m_load_time.push_back(std::vector<std::vector<Real>>(
                        shovels.size(), std::vector<Real>(trucks.size(), 0)));

                for (size_t i = 0; i < data.size(); i++) {
                    int shovel_index = i / trucks.size();
                    int truck_index = i % trucks.size();
                    Real load_time = std::stod(data[i]);
                    m_load_time.back()[shovel_index][truck_index] = load_time;
                    if (load_time > m_max_time)
                        m_max_time = load_time;
                }
            }
        }
    }

    m_num_samples = sample;
    m_num_nodes = m_num_shovels + m_num_unloads;

    m_time_range = std::vector<std::vector<Real>>(m_num_samples, {0, m_shift_time});

    m_load_quantity =
        std::vector<std::vector<Real>>(m_num_samples, std::vector<Real>(m_num_shovels, 0));
    m_load_grade = m_load_quantity;
    m_load_quantity_total = std::vector<Real>(m_num_samples, 0);
    m_load_grade_total = m_load_quantity_total;
    for (size_t i = 0; i < m_num_samples; i++) {
        for (size_t j = 0; j < m_num_shovels; j++) {
            for (size_t k = 0; k < m_load_sequence[i][j].size(); k++) {
                Real total = m_load_sequence[i][j][k][0];
                Real ore = total * m_load_sequence[i][j][k][1];
                m_load_quantity[i][j] += total;
                m_load_grade[i][j] += ore;
                m_load_quantity_total[i] += total;
                m_load_grade_total[i] += ore;
            }
            m_load_grade[i][j] /= m_load_quantity[i][j];
        }
        m_load_grade_total[i] /= m_load_quantity_total[i];
    }

    m_load_time_normal.resize(m_num_samples);
    m_unload_time_normal.resize(m_num_samples);
    for (size_t i = 0; i < m_num_samples; i++) {
        int pos_count = 0;
        m_load_time_normal[i].resize(m_load_time[i].size());
        for (size_t j = 0; j < m_load_time[i].size(); j++) {
            m_load_time_normal[i][j].resize(m_load_time[i][j].size());
            for (size_t k = 0; k < m_load_time[i][j].size(); k++) {
                m_load_time_normal[i][j][k].setSeed(seeds[i][pos_count]);
                m_load_time_normal[i][j][k].setParam(m_load_time[i][j][k], m_std_load_time);
                pos_count++;
            }
        }

        m_unload_time_normal[i].resize(m_truck_unload_time[i].size());
        for (size_t j = 0; j < m_truck_unload_time[i].size(); j++) {
            m_unload_time_normal[i][j].setSeed(seeds[i][pos_count]);
            m_unload_time_normal[i][j].setParam(m_truck_unload_time[i][j], m_std_unload_time);
            pos_count++;
        }
    }
}

void TruckBase::resetSolutionWithSeed(Sol *s, std::uint64_t seed) {
    std::mt19937_64 rng(seed);
    resetSolutionImpl(s, &rng);
}

void TruckBase::resetSolutionImpl(Sol *s, std::mt19937_64 *deterministic_rng) {
    auto &x = (*s).variable();
    int sample = x.m_sample_id;
    x.m_cur_load_sequence = m_load_sequence[sample];
    x.m_cur_shovel_flow_length = m_shovel_flow_length[sample];
    x.m_load_time_normal = m_load_time_normal[sample];
    x.m_unload_time_normal = m_unload_time_normal[sample];

    for (auto &rands : x.m_load_time_normal)
        for (auto &rand : rands)
            rand.reset();

    for (auto &rand : x.m_unload_time_normal)
        rand.reset();

    x.m_truck_routes.clear();
    x.m_truck_map.clear();
    x.m_arrive_time.clear();
    x.m_node_truck_sequence.clear();

    x.m_truck_action.clear();

    x.m_truck_routes.resize(m_num_trucks);
    x.m_truck_map.resize(m_num_trucks);
    x.m_arrive_time.resize(m_num_trucks);
    x.m_node_truck_sequence.resize(m_num_nodes);

    // Initialize time window status

    while (1) {
        Real min_time = FLT_MAX;
        int truck = -1;

        for (int j = 0; j < m_num_trucks; j++) {
            Real time = x.m_arrive_time[j].size() == 0 ? 0 : x.m_arrive_time[j].back();
            if (time >= m_time_range[sample][0] || time >= m_truck_down_time[sample][j])
                continue;
            if (time < min_time) {
                min_time = time;
                truck = j;
            }
        }
        if (truck == -1)
            break;

        std::vector<int> mask = updateMask(sample, truck, s);
        std::vector<int> points;
        for (int j = 0; j < mask.size(); j++)
            if (mask[j] == 1)
                points.push_back(j);
        if (points.size() == 0)
            break;

        int rand_pos = std::uniform_int_distribution<int>(0, static_cast<int>(points.size()) -
                                                                 1)(*deterministic_rng);
        int point = points[rand_pos];
        updateState(sample, truck, point, s);
    }
}

int TruckBase::choseTruck(int sample, Sol *s) {
    auto &arrive_time = (*s).variable().m_arrive_time;
    Real end_time = m_time_range[sample][1];
    Real min_time = FLT_MAX;
    int indice = -1;
    for (size_t j = 0; j < m_num_trucks; j++) {
        Real time = arrive_time[j].size() == 0 ? 0 : arrive_time[j].back();
        if (time >= m_truck_down_time[sample][j] || time >= end_time)
            continue;
        if (time < min_time) {
            min_time = time;
            indice = j;
        }
    }

    return indice;
}

std::vector<int> TruckBase::updateMask(int sample, int truck, Sol *s) {
    auto &x = (*s).variable();
    auto &solution = x.m_truck_routes;
    auto &solution_map = x.m_truck_map;
    auto &node_truck_sequence = x.m_node_truck_sequence;
    auto &load_sequence = x.m_cur_load_sequence;
    auto &shovel_flow_length = x.m_cur_shovel_flow_length;

    std::vector<int> mask(m_num_nodes, 0);
    Real capacity = m_truck_capacity[sample][truck];
    bool is_load = solution[truck].size() % 2 == 0;

    // is_load == 1, going to loading point, mask unloading points and loading points with no ore

    if (is_load) {
        for (size_t j = 0; j < m_num_nodes; j++) {
            if (j >= m_num_shovels)
                mask[j] = 0;
            else {
                Real load_quantity = 0;
                int length = shovel_flow_length[j];
                for (size_t k = 0; k < length; k++)
                    load_quantity += load_sequence[j][k][0];
                if (load_quantity < capacity)
                    mask[j] = 0;
                else
                    mask[j] = 1;

                if (mask[j] == 1) {
                    Real arrive_time = getTruckInfo(sample, truck, j, s).arriveTime;
                    if (arrive_time >= m_shovel_down_time[sample][j])
                        mask[j] = 0;
                }
            }
        }
    }

    // is_load == 0, going to unloading point, mask loading points

    else {
        int shovel = solution[truck].back();
        int seq_indice = solution_map[truck].back();
        Real grade = node_truck_sequence[shovel][seq_indice].grade;

        for (size_t j = 0; j < m_num_nodes; j++) {
            if (j < m_num_shovels)
                mask[j] = 0;
            else {
                int type = m_unload_type[sample][j - m_num_shovels];
                int invalid = (grade >= 0.3 && type != 0) || (grade < 0.3 && type == 0);
                mask[j] = invalid ? 0 : 1;
            }
        }
    }

    return mask;
}

TruckInf TruckBase::getTruckInfo(int sample, int truck, int point, Sol *s) {
    auto &x = (*s).variable();
    auto &solution = x.m_truck_routes;
    auto &solution_map = x.m_truck_map;
    auto &node_truck_sequence = x.m_node_truck_sequence;

    Real capacity = m_truck_capacity[sample][truck];
    int truck_type = m_truck_type[sample][truck];
    int driver_type = m_driver_type[sample][truck];
    bool is_load = solution[truck].size() % 2 == 0;
    bool is_empty = solution[truck].size() == 0;

    // if the route of current truck is non-empty, arrive_time = last_leave_time + travel_time

    Real arrive_time = 0;

    if (!is_empty) {

        // caculate leave time in the last point of currrent truck

        int last_point = solution[truck].back();
        int seq_indice = solution_map[truck].back();
        Real leave_time = node_truck_sequence[last_point][seq_indice].leaveTime;

        // caculate travel time between last point and adding point

        int load_indice = is_load ? point : last_point;
        int unload_indice = is_load ? (last_point - m_num_shovels) : (point - m_num_shovels);
        Real travel_time = 0;
        if (leave_time < m_event_time[sample])
            travel_time =
                is_load
                    ? m_road_predict[sample][load_indice][unload_indice][truck_type][driver_type][1]
                    : m_road_predict[sample][load_indice][unload_indice][truck_type][driver_type]
                                    [0];
        else
            travel_time = is_load ? m_road_predict_d[sample][load_indice][unload_indice][truck_type]
                                                    [driver_type][1]
                                  : m_road_predict_d[sample][load_indice][unload_indice][truck_type]
                                                    [driver_type][0];

        arrive_time = leave_time + travel_time;
    }

    Real handle_time = is_load ? m_load_time[sample][m_shovel_type[sample][point]][truck_type]
                               : m_truck_unload_time[sample][truck];

    // update truck sequences

    TruckInf truck_info = {truck,
                           (int)solution[truck].size(),
                           0,
                           0,
                           0,
                           arrive_time,
                           arrive_time,
                           arrive_time + handle_time,
                           capacity,
                           0,
                           0,
                           0,
                           0};
    return truck_info;
}

int TruckBase::searchGrade(int sample, int point, TruckInf &truck_info, Sol *s) {
    auto &x = (*s).variable();
    auto &solution = x.m_truck_routes;
    auto &solution_map = x.m_truck_map;
    auto &node_truck_sequence = x.m_node_truck_sequence;

    int truck = truck_info.truck;
    Real point_quantity = m_point_quantity[sample][point];
    Real capacity = truck_info.ore;
    Real handle_time = truck_info.leaveTime - truck_info.workTime;

    // find the insert position of sequences based on arriving time

    Real arrive_time = truck_info.arriveTime;
    int length = node_truck_sequence[point].size();
    int insert_pos = 0;

    for (int i = length - 1; i >= 0; i--) {
        Real time = node_truck_sequence[point][i].arriveTime;
        if (i == length - 1) {
            if (time <= arrive_time) {
                insert_pos = length;
                break;
            }
        } else {
            Real last_time = node_truck_sequence[point][i + 1].arriveTime;
            if (time <= arrive_time && arrive_time <= last_time) {
                insert_pos = i + 1;
                break;
            }
        }
    }

    // calculate current progress of each task

    Real last_progress = insert_pos == 0 ? 0 : node_truck_sequence[point][insert_pos - 1].progress;
    truck_info.progress = last_progress + (capacity / point_quantity);

    // if go to loading point

    if (truck_info.point % 2 == 0) {
        int length = m_shovel_flow_length[sample][point];
        TruckInf load_info = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        if (insert_pos != 0)
            load_info = node_truck_sequence[point][insert_pos - 1];
        else
            load_info.shovelPoint = length - 1;

        int last_shovel_point = load_info.shovelPoint;
        Real last_point_progress = load_info.pointProgress;
        Real last_leave_time = load_info.leaveTime;

        // 1. calculate waitting time

        if (last_leave_time > arrive_time) {
            truck_info.workTime = last_leave_time;
            truck_info.leaveTime = last_leave_time + handle_time;
        }

        // 2. caculate the grade and progress of loading points the truck is going to

        int shovel_point = 0;
        Real point_progress = 0, grade = 0;
        Real ore = 0, sum = 0, remaining_capacity = capacity;

        for (int i = last_shovel_point; i >= 0; i--) {
            Real row_quantity = m_load_sequence[sample][point][i][0];
            Real quantity = row_quantity;
            Real temp_grade = m_load_sequence[sample][point][i][1];
            if (i == last_shovel_point)
                quantity *= (1 - last_point_progress);

            Real used_capacity = quantity > remaining_capacity ? remaining_capacity : quantity;
            ore += used_capacity * temp_grade;
            sum += used_capacity;
            remaining_capacity = capacity - sum;

            if (remaining_capacity == 0) {
                shovel_point = i;
                point_progress = 1 - (quantity - used_capacity) / row_quantity;
                grade = ore / capacity;
                break;
            }
        }
        truck_info.shovelPoint = shovel_point;
        truck_info.pointProgress = point_progress;
        truck_info.grade = grade;
    }

    // if go to unloading point

    else {
        // calculate grade of the truck

        int last_point = solution[truck][truck_info.point - 1];
        int seq_indice = solution_map[truck][truck_info.point - 1];
        truck_info.grade = node_truck_sequence[last_point][seq_indice].grade;
    }

    return insert_pos;
}

void TruckBase::updateState(int sample, int truck, int point, Sol *s) {
    auto &x = (*s).variable();
    auto &solution = x.m_truck_routes;
    auto &solution_map = x.m_truck_map;
    auto &times = x.m_arrive_time;
    auto &node_truck_sequence = x.m_node_truck_sequence;
    auto &load_sequence = x.m_cur_load_sequence;
    auto &shovel_flow_length = x.m_cur_shovel_flow_length;
    auto &load_time_normal = x.m_load_time_normal;
    auto &unload_time_normal = x.m_unload_time_normal;
    auto &action = x.m_truck_action;

    Real capacity = m_truck_capacity[sample][truck];
    int truck_type = m_truck_type[sample][truck];
    bool is_load = solution[truck].size() % 2 == 0;

    // 1. Calculate Information for the Current Step
    // ---------------------------------------------------------
    auto truck_info = getTruckInfo(sample, truck, point, s);

    Real handle_time = is_load ? load_time_normal[m_shovel_type[sample][point]][truck_type]()
                               : unload_time_normal[truck]();
    truck_info.leaveTime = truck_info.workTime + handle_time;
    Real arrive_time = truck_info.arriveTime;

    // 2. Commit Current Step to History
    // ---------------------------------------------------------
    solution[truck].push_back(point);
    solution_map[truck].push_back(0); // placeholder, updated by insertSequence
    times[truck].push_back(arrive_time);
    action.push_back(truck);

    // Insert into Node Sequence (Global Timeline)
    insertSequence(sample, truck_info, s);

    // 3. Recursive Re-simulation Strategy
    //    Identify all tasks in the system that are strictly "in the future"
    //    relative to this new event and might need rescheduling.
    // ---------------------------------------------------------

    struct RedoTask {
        int truck_idx;
        int point_idx;
        Real original_time;
    };
    std::vector<RedoTask> redo_list;

    // A. Collect and Remove "Future" Tasks from Node Sequences
    for (size_t i = 0; i < m_num_nodes; i++) {
        auto &seq = node_truck_sequence[i];
        // Iterate backwards to safely erase
        for (int k = seq.size() - 1; k >= 0; k--) {
            if (seq[k].arriveTime > arrive_time) {
                int point = solution[seq[k].truck][seq[k].point];
                redo_list.push_back({seq[k].truck, point, seq[k].arriveTime});
                seq.erase(seq.begin() + k); // Remove from timeline immediately
            }
        }
    }

    // Base case: No future conflicts, we are done.
    // if (redo_list.empty()) return;

    // B. Sort tasks to preserve original causal order
    std::sort(redo_list.begin(), redo_list.end(), [](const RedoTask &a, const RedoTask &b) {
        return a.original_time < b.original_time;
    });

    // C. Rollback Truck Route State (solution, times, etc.)
    // Count how many steps to pop back for each truck
    std::vector<int> trucks_to_pop(m_num_trucks, 0);
    for (const auto &task : redo_list) {
        trucks_to_pop[task.truck_idx]++;
    }

    for (size_t t = 0; t < m_num_trucks; t++) {
        auto &route = solution[t];
        auto &t_map = solution_map[t];
        auto &t_times = times[t];

        for (int k = 0; k < trucks_to_pop[t]; ++k) {
            if (!route.empty()) {
                route.pop_back();
                t_map.pop_back();
                t_times.pop_back();
                for (int a = action.size() - 1; a >= 0; --a) {
                    if (action[a] == t) {
                        action.erase(action.begin() + a);
                        break;
                    }
                }
            }
        }
    }

    for (size_t i = 0; i < m_num_shovels; i++) {
        Real shovel_point = 0, point_progress = 0;
        if (node_truck_sequence[i].size() == 0)
            shovel_point = m_shovel_flow_length[sample][i] - 1;
        else {
            shovel_point = node_truck_sequence[i].back().shovelPoint;
            point_progress = node_truck_sequence[i].back().pointProgress;
        }
        shovel_flow_length[i] = shovel_point + 1;
        for (size_t j = 0; j < load_sequence[i].size(); j++) {
            Real row_quantity = m_load_sequence[sample][i][j][0];
            if (j == shovel_point)
                load_sequence[i][j][0] = row_quantity * (1 - point_progress);
            else
                load_sequence[i][j][0] = row_quantity;
        }
    }

    // =========================================================

    // =========================================================

    for (const auto &task : redo_list) {
        // Recursively call updateState.
        // Since the environment (timelines & resources) has been reset,
        // this call will naturally calculate the CORRECT new times and resource consumptions.
        updateState(sample, task.truck_idx, task.point_idx, s);
    }
}

void TruckBase::insertSequence(int sample, TruckInf &truck_info, Sol *s) {
    auto &x = (*s).variable();
    auto &solution = x.m_truck_routes;
    auto &solution_map = x.m_truck_map;
    auto &node_truck_sequence = x.m_node_truck_sequence;

    int truck = truck_info.truck, point = solution[truck_info.truck][truck_info.point];

    Real point_quantity = m_point_quantity[sample][point];
    Real capacity = truck_info.ore;
    Real handle_time = truck_info.leaveTime - truck_info.workTime;
    Real arrive_time = truck_info.arriveTime;

    int insert_pos = searchGrade(sample, point, truck_info, s);

    // record insert position of seq

    solution_map[truck][truck_info.point] = insert_pos;

    // if go to unloading point

    if (truck_info.point % 2 != 0) {

        // 1. query the unloading truck in seq

        std::vector<int> unloading_queue;
        int unloading_port = 0;

        for (int i = insert_pos - 1; i >= 0; i--) {
            TruckInf &info = node_truck_sequence[point][i];
            if (info.leaveTime > arrive_time) {
                unloading_queue.push_back(i);
                unloading_port += m_truck_port[sample][info.truck];
            }
        }
        int self_port = m_truck_port[sample][truck];
        int unload_port = m_unload_port[sample][point - m_num_shovels];

        // 2. if unloading truck is non-empty, calculate waitting time

        if (unloading_port + self_port > unload_port) {

            std::sort(unloading_queue.begin(), unloading_queue.end(),
                      [&node_truck_sequence, point](const int &x1, const int &x2) {
                          return node_truck_sequence[point][x1].leaveTime <
                                 node_truck_sequence[point][x2].leaveTime;
                      });

            int empty_port = unload_port - unloading_port;
            Real leave_time = 0;

            for (size_t i = 0; i < unloading_queue.size(); i++) {
                TruckInf &info = node_truck_sequence[point][unloading_queue[i]];
                empty_port += m_truck_port[sample][info.truck];
                if (empty_port >= self_port) {
                    leave_time = info.leaveTime;
                    break;
                }
            }
            truck_info.workTime = leave_time;
            truck_info.leaveTime = leave_time + handle_time;
        }
    }

    // insert updated truck schedule info

    node_truck_sequence[point].insert(node_truck_sequence[point].begin() + insert_pos, truck_info);

    for (size_t i = insert_pos + 1; i < node_truck_sequence[point].size(); i++) {
        int seq_truck = node_truck_sequence[point][i].truck;
        int seq_point = node_truck_sequence[point][i].point;
        solution_map[seq_truck][seq_point] = i;
    }
}

Real TruckBase::evaluate(int sample, Sol *s) {
    auto &node_truck_sequence = (*s).variable().m_node_truck_sequence;

    Real start_time = m_time_range[sample][0];
    Real end_time = m_time_range[sample][1];

    // evaluate ore flow grade deviation

    Real flow_deviation = 0;
    int num_window = 0;

    for (size_t i = 0; i < m_num_unloads; i++) {
        if (m_unload_type[sample][i] != 0)
            continue;
        std::vector<TruckInf> &queue = node_truck_sequence[m_num_shovels + i];

        for (size_t j = 0; j < queue.size(); j++) {
            if (queue[j].arriveTime < start_time)
                continue;
            Real count_ore = 0;
            Real count_grade = 0;
            bool finish = false;
            Real window_grade = 0;
            for (size_t k = j; k < queue.size(); k++) {
                Real ore = m_truck_capacity[sample][queue[k].truck];
                Real grade = queue[k].grade;
                count_ore += ore;
                count_grade += ore * grade;

                if (count_ore >= m_time_window || queue[k].arriveTime >= end_time ||
                    k == queue.size() - 1) {
                    if (queue[k].arriveTime >= end_time || k == queue.size() - 1)
                        finish = true;
                    window_grade = count_grade / count_ore;
                    flow_deviation +=
                        (window_grade - m_target_grade) * (window_grade - m_target_grade);
                    num_window++;
                    break;
                }
            }
            if (finish)
                break;
        }
    }

    if (num_window == 0)
        flow_deviation = 0;
    else
        flow_deviation = std::sqrt(flow_deviation / num_window);

    return flow_deviation;
}

std::vector<std::vector<Real>>
TruckBase::getHeuristicValue(int sample, int truck, const std::vector<int> &mask,
                             const std::unordered_map<std::string, Real> &data_range, Sol *s,
                             int feature_count) {
    auto &x = (*s).variable();
    auto &solution = x.m_truck_routes;
    auto &solution_map = x.m_truck_map;
    auto &node_truck_sequence = x.m_node_truck_sequence;
    auto &load_sequence = x.m_cur_load_sequence;
    auto &shovel_flow_length = x.m_cur_shovel_flow_length;

    auto &max_shift = data_range.at("max_shift");
    auto &max_capacity = data_range.at("max_capacity");
    auto &min_grade = data_range.at("min_grade");
    auto &max_grade = data_range.at("max_grade");
    auto &max_time = data_range.at("max_time");
    auto &max_port = data_range.at("max_port");
    auto &max_quantity = data_range.at("max_quantity");
    Real max_deviation =
        std::max(std::abs(m_target_grade - min_grade), std::abs(m_target_grade - max_grade));

    // caculate rest grade and ore on each point

    std::vector<Real> rest_quantity(m_num_shovels, 0);
    std::vector<Real> rest_grade = rest_quantity;
    Real rest_quantity_total = 0;
    Real rest_grade_total = rest_quantity_total;

    for (size_t j = 0; j < m_num_shovels; j++) {
        int length = shovel_flow_length[j];
        for (size_t k = 0; k < length; k++) {
            Real total = load_sequence[j][k][0];
            Real ore = total * load_sequence[j][k][1];
            rest_quantity[j] += total;
            rest_grade[j] += ore;
            rest_quantity_total += total;
            rest_grade_total += ore;
        }
        rest_grade[j] /= rest_quantity[j];
    }
    rest_grade_total /= rest_quantity_total;

    if (feature_count < 0)
        feature_count = static_cast<int>(Terminals::TerminalNum);
    if (feature_count <= 0 || feature_count > static_cast<int>(Terminals::TerminalNum)) {
        throw std::invalid_argument("invalid GP terminal feature count");
    }
    std::vector<std::vector<Real>> values(m_num_nodes, std::vector<Real>(feature_count, 1));

    // 1. other truck's average grade

    Real truck_ore = 0, truck_grade = 0;
    for (size_t j = 0; j < m_num_trucks; j++) {
        if (j == truck || solution[j].size() == 0)
            continue;
        int last_point = solution[j].back();
        int seq_indice = solution_map[j].back();
        Real grade = node_truck_sequence[last_point][seq_indice].grade;
        truck_ore += m_truck_capacity[sample][j];
        truck_grade += m_truck_capacity[sample][j] * grade;
    }
    if (truck_ore != 0)
        truck_grade /= truck_ore;

    // 2. average load grade across all loading point

    Real load_ore = 0, load_grade = 0;
    for (size_t j = 0; j < m_num_shovels; j++) {
        int length = shovel_flow_length[j];
        load_ore += load_sequence[j][length - 1][0];
        load_grade += load_sequence[j][length - 1][0] * load_sequence[j][length - 1][1];
    }
    load_grade /= load_ore;

    // 3. window grade of the unloading point

    std::vector<Real> unload_grade(m_num_nodes, max_grade);
    Real unload_grade_total = 0, unload_ore_total = 0;
    for (size_t j = 0; j < m_num_unloads; j++) {
        if (m_unload_type[sample][j] != 0)
            continue;
        int point = m_num_shovels + j;

        Real count_ore = 0;
        Real count_grade = 0;
        Real window_grade = 0;
        std::vector<TruckInf> &queue = node_truck_sequence[point];
        for (int q = queue.size() - 1; q >= 0; q--) {
            Real ore = m_truck_capacity[sample][queue[q].truck];
            Real grade = queue[q].grade;
            count_ore += ore;
            count_grade += ore * grade;

            if (count_ore >= m_time_window || q == 0) {
                window_grade = count_grade / count_ore;
                break;
            }
        }
        unload_grade[point] = window_grade;
        unload_ore_total += count_ore;
        unload_grade_total += count_grade;
    }
    if (unload_ore_total != 0)
        unload_grade_total /= unload_ore_total;

    // 4. ore flow error of unloading point

    std::vector<Real> unload_error(m_num_nodes, max_deviation);

    Real start_time = m_time_range[sample][0];
    Real end_time = m_time_range[sample][1];

    // evaluate ore flow grade deviation

    Real flow_deviation = 0;
    int num_window = 0;

    for (size_t j = 0; j < m_num_unloads; j++) {
        if (m_unload_type[sample][j] != 0)
            continue;
        std::vector<TruckInf> &queue = node_truck_sequence[m_num_shovels + j];
        Real flow_deviation_unload = 0;
        int num_window_unload = 0;
        for (size_t k = 0; k < queue.size(); k++) {
            if (queue[k].arriveTime < start_time)
                continue;
            Real count_ore = 0;
            Real count_grade = 0;
            bool finish = false;
            Real window_grade = 0;
            for (size_t l = k; l < queue.size(); l++) {
                Real ore = m_truck_capacity[sample][queue[l].truck];
                Real grade = queue[l].grade;
                count_ore += ore;
                count_grade += ore * grade;

                if (count_ore >= m_time_window || queue[l].arriveTime >= end_time ||
                    l == queue.size() - 1) {
                    if (queue[l].arriveTime >= end_time || l == queue.size() - 1)
                        finish = true;
                    window_grade = count_grade / count_ore;
                    flow_deviation +=
                        (window_grade - m_target_grade) * (window_grade - m_target_grade);
                    flow_deviation_unload +=
                        (window_grade - m_target_grade) * (window_grade - m_target_grade);
                    num_window++;
                    num_window_unload++;
                    break;
                }
            }
            if (finish)
                break;
        }
        if (num_window_unload == 0)
            flow_deviation_unload = 1;
        else
            flow_deviation_unload = std::sqrt(flow_deviation_unload / num_window_unload);
        unload_error[j] = flow_deviation_unload;
    }
    if (num_window == 0)
        flow_deviation = 1;
    else
        flow_deviation = std::sqrt(flow_deviation / num_window);

    for (size_t j = 0; j < m_num_nodes; j++) {
        if (mask[j] == 0)
            continue;
        bool is_load = j < m_num_shovels;

        // Terminals::CTC

        values[j][0] = m_truck_capacity[sample][truck] / max_capacity;

        // Terminals::GradeTG

        values[j][1] = (m_target_grade - min_grade) / (max_grade - min_grade);

        // Terminals::GradeALP

        values[j][2] = (m_load_grade_total[sample] - min_grade) / (max_grade - min_grade);

        // Terminals::RGradeALP

        values[j][3] = (rest_grade_total - min_grade) / (max_grade - min_grade);

        // Terminals::PALP

        values[j][4] = rest_quantity_total / m_load_quantity_total[sample];

        // Terminals::GradeCLP

        if (is_load)
            values[j][5] = (m_load_grade[sample][j] - min_grade) / (max_grade - min_grade);
        else
            values[j][5] = 1;

        // Terminals::RGradeCLP

        if (is_load)
            values[j][6] = (rest_grade[j] - min_grade) / (max_grade - min_grade);
        else
            values[j][6] = 1;

        // Terminals::PCLP

        if (is_load)
            values[j][7] = rest_quantity[j] / m_load_quantity[sample][j];
        else
            values[j][7] = 1;

        // Terminals::CTGUP

        Real grade = 0;
        if (is_load) {
            auto truck_info = getTruckInfo(sample, truck, j, s);
            searchGrade(sample, j, truck_info, s);
            grade = truck_info.grade;
        } else {
            int last_point = solution[truck].back();
            int seq_indice = solution_map[truck].back();
            grade = node_truck_sequence[last_point][seq_indice].grade;
        }
        values[j][8] = (grade - min_grade) / (max_grade - min_grade);

        // Terminals::OTGUP

        values[j][9] = (truck_grade - min_grade) / (max_grade - min_grade);

        // Terminals::ALGALP

        values[j][10] = (load_grade - min_grade) / (max_grade - min_grade);

        // Terminals::WGUP

        values[j][11] = (unload_grade[j] - min_grade) / (max_grade - min_grade);

        // Terminals::AWGAUP

        values[j][12] = (unload_grade_total - min_grade) / (max_grade - min_grade);

        // Terminals::ErrUP

        values[j][13] = unload_error[j] / max_deviation;

        // Terminals::ErrAUP

        values[j][14] = flow_deviation / max_deviation;

        // Terminals::ACOH

        auto value = acoHeuristicValue(sample, truck, j, s);
        values[j][15] = value / max_deviation;
    }

    return values;
}

Real TruckBase::acoHeuristicValue(int sample, int truck, int point, Sol *s) {
    auto &x = (*s).variable();
    auto &solution = x.m_truck_routes;
    auto &solution_map = x.m_truck_map;
    auto &node_truck_sequence = x.m_node_truck_sequence;

    Real capacity = m_truck_capacity[sample][truck];
    bool is_load = solution[truck].size() % 2 == 0;
    int truck_type = m_truck_type[sample][truck];
    int driver_type = m_driver_type[sample][truck];

    Real value = 0;

    if (is_load) {
        auto truck_info = getTruckInfo(sample, truck, point, s);
        searchGrade(sample, point, truck_info, s);
        Real truck_grade = truck_info.grade;

        Real error = 0;
        int num_ore_crusher = 0;
        for (size_t k = 0; k < m_num_unloads; k++) {
            if (m_unload_type[sample][k] != 0)
                continue;
            num_ore_crusher++;
            std::vector<TruckInf> &queue = node_truck_sequence[m_num_shovels + k];

            Real count_ore = capacity;
            Real count_grade = capacity * truck_grade;
            Real window_grade = 0;
            for (int q = queue.size() - 1; q >= 0; q--) {
                Real ore = m_truck_capacity[sample][queue[q].truck];
                Real grade = queue[q].grade;
                count_ore += ore;
                count_grade += ore * grade;

                if (count_ore >= m_time_window || q == 0)
                    break;
            }
            window_grade = count_grade / count_ore;
            error += fabs(window_grade - m_target_grade);
        }
        error /= num_ore_crusher;
        value = error;
    } else {
        int shovel = solution[truck].back();
        int seq_indice = solution_map[truck].back();
        Real truck_grade = node_truck_sequence[shovel][seq_indice].grade;

        Real count_ore = capacity;
        Real count_grade = capacity * truck_grade;
        Real window_grade = 0;
        std::vector<TruckInf> &queue = node_truck_sequence[point];
        for (int q = queue.size() - 1; q >= 0; q--) {
            Real ore = m_truck_capacity[sample][queue[q].truck];
            Real grade = queue[q].grade;
            count_ore += ore;
            count_grade += ore * grade;

            if (count_ore >= m_time_window || q == 0) {
                window_grade = count_grade / count_ore;
                break;
            }
        }
        Real error = fabs(window_grade - m_target_grade);
        value = error;
    }

    return value;
}

int TruckBase::getMaxRouteLength(int sample) {
    Real minLoadingTime = FLT_MAX, minHeavyTime = FLT_MAX, minUnloadingTime = FLT_MAX,
         minEmptyTime = FLT_MAX;

    for (auto &times : m_load_time[sample])
        for (auto &time : times)
            if (time < minLoadingTime)
                minLoadingTime = time;

    for (auto &time : m_truck_unload_time[sample])
        if (time < minUnloadingTime)
            minUnloadingTime = time;

    for (size_t i = 0; i < m_road_predict[sample].size(); i++) {
        for (size_t j = 0; j < m_road_predict[sample][i].size(); j++) {
            for (size_t k = 0; k < m_road_predict[sample][i][j].size(); k++) {
                for (size_t l = 0; l < m_road_predict[sample][i][j][k].size(); l++) {
                    if (m_road_predict[sample][i][j][k][l][0] < minHeavyTime)
                        minHeavyTime = m_road_predict[sample][i][j][k][l][0];
                    if (m_road_predict[sample][i][j][k][l][1] < minEmptyTime)
                        minEmptyTime = m_road_predict[sample][i][j][k][l][1];
                    if (m_road_predict_d[sample][i][j][k][l][0] < minHeavyTime)
                        minHeavyTime = m_road_predict_d[sample][i][j][k][l][0];
                    if (m_road_predict_d[sample][i][j][k][l][1] < minEmptyTime)
                        minEmptyTime = m_road_predict_d[sample][i][j][k][l][1];
                }
            }
        }
    }
    Real minCycleTime = minLoadingTime + minHeavyTime + minUnloadingTime + minEmptyTime;

    return (m_shift_time / minCycleTime) * 3;
}
} // namespace gpaco
