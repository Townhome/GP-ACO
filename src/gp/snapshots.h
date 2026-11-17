#pragma once
#include "gp/model.h"
#include "problem/simulator.h"
#include "core/cpu_executor.h"
#include <algorithm>
#include <map>
#include <numeric>
#include <stdexcept>
#include <tuple>

namespace gpaco::snapshots {

using Range = std::unordered_map<std::string, Real>;

inline void require(bool condition, const std::string &message) {
    if (!condition)
        throw std::runtime_error("GP-ACO snapshots: " + message);
}
struct Input {
    int mine = 0;
    TruckBase *problem = nullptr;
    std::string split;
    std::vector<int> samples;
    std::vector<uint64_t> hashes;
};
struct Reference {
    std::vector<Snapshot> shots;
    Json sources = Json::array();
    std::vector<double> weights;
    Json summary;
};
struct EventInfo {
    bool normal = true;
    double first_time = 0;
    std::string family = "normal";
};
// Read operational fields, not ordinal E1--E5 blocks. Weather and road events
// share a family because the files do not carry an explicit event-type label.
inline EventInfo eventInfo(const TruckBase &pro, int sample) {
    EventInfo out;
    out.first_time = pro.m_shift_time;
    bool truck = false, shovel = false, road = pro.m_event_time.at(sample) < pro.m_shift_time;
    for (Real t : pro.m_truck_down_time.at(sample))
        if (t < pro.m_shift_time) {
            truck = true;
            out.first_time = std::min(out.first_time, double(t));
        }
    for (Real t : pro.m_shovel_down_time.at(sample))
        if (t < pro.m_shift_time) {
            shovel = true;
            out.first_time = std::min(out.first_time, double(t));
        }
    if (road)
        out.first_time = std::min(out.first_time, double(pro.m_event_time.at(sample)));
    int count = int(truck) + int(shovel) + int(road);
    out.normal = count == 0;
    out.family = count > 1 ? "multiple"
                 : truck   ? "truck_failure"
                 : shovel  ? "shovel_failure"
                 : road    ? "road_or_weather"
                           : "normal";
    return out;
}
inline Json json(const Reference &ref) {
    Json rows = Json::array();
    for (const auto &shot : ref.shots)
        rows.push_back({{"features", shot.action_features}, {"mask", shot.action_mask}});
    return {{"sources", ref.sources},
            {"snapshots", rows},
            {"weights", ref.weights},
            {"summary", ref.summary}};
}
inline Json summarize(const Json &sources) {
    Json mines = Json::object(), events = Json::object();
    size_t multiple_actions = 0, after_event = 0;
    for (const auto &row : sources) {
        auto mine = row.at("scenario").get<std::string>();
        auto event = row.at("event_family").get<std::string>();
        mines[mine] = mines.value(mine, size_t(0)) + 1;
        events[event] = events.value(event, size_t(0)) + 1;
        multiple_actions += row.at("legal_actions").get<int>() > 1;
        after_event += row.at("after_disturbance").get<bool>();
    }
    return {{"count", sources.size()},
            {"by_mine", mines},
            {"by_event_family", events},
            {"multiple_legal_actions", multiple_actions},
            {"after_disturbance", after_event}};
}
struct Position {
    uint64_t priority = 0;
    int input = 0, sample = 0, step = 0;
    double time = 0;
    int legal_actions = 0;
};
inline bool priorityLess(const Position &a, const Position &b) {
    return std::tie(a.priority, a.input, a.sample, a.step) <
           std::tie(b.priority, b.input, b.sample, b.step);
}
template <class Visit>
inline void trajectory(const Input &input, int sample, uint64_t root_seed, Visit visit) {
    auto &pro = *input.problem;
    Sol sol;
    sol.variable().m_sample_id = sample;
    pro.resetSolutionWithSeed(&sol, instanceSeed(root_seed, input.mine, sample, 11, 0));
    std::mt19937_64 actions(instanceSeed(root_seed, input.mine, sample, 12, 0));
    for (int step = 0;; ++step) {
        int truck = pro.choseTruck(sample, &sol);
        if (truck < 0)
            break;
        auto mask = pro.updateMask(sample, truck, &sol);
        std::vector<int> legal;
        for (size_t action = 0; action < mask.size(); ++action)
            if (mask[action])
                legal.push_back(int(action));
        if (legal.empty())
            break;
        const auto &arrival = sol.variable().m_arrive_time.at(truck);
        double time = arrival.empty() ? 0. : double(arrival.back());
        visit(step, truck, mask, int(legal.size()), time, sol);
        int action = legal[std::uniform_int_distribution<size_t>(0, legal.size() - 1)(actions)];
        pro.updateState(sample, truck, action, &sol);
    }
}
class Bank {
    struct Item {
        Snapshot shot;
        Json source;
    };
    std::vector<std::vector<Item>> all;

  public:
    Json audit;
    // Enumerate positions in parallel, then replay only selected instances to
    // materialize features. This avoids retaining features for every state.
    Bank(const std::vector<Input> &inputs, int all_capacity, const Range &range,
         uint64_t trajectory_seed, uint64_t sampling_seed, Pool &pool, int min_legal_actions = 1) {
        require(min_legal_actions > 0, "minimum legal action count must be positive");
        all.resize(inputs.size());
        std::vector<std::pair<int, int>> jobs;
        for (size_t i = 0; i < inputs.size(); ++i)
            for (int sample : inputs[i].samples)
                jobs.emplace_back(int(i), sample);
        std::vector<std::vector<Position>> positions(jobs.size());
        pool.run(jobs.size(), [&](size_t job) {
            auto [i, sample] = jobs[job];
            std::mt19937_64 priorities(instanceSeed(sampling_seed, inputs[i].mine, sample, 21, 0));
            trajectory(inputs[i], sample, trajectory_seed,
                       [&](int step, int, const auto &, int legal, double time, auto &) {
                           auto priority = priorities();
                           if (legal >= min_legal_actions)
                               positions[job].push_back({priority, i, sample, step, time, legal});
                       });
        });
        std::vector<std::vector<Position>> selected_all(inputs.size());
        Json candidate_sources = Json::array();
        for (size_t i = 0; i < inputs.size(); ++i) {
            std::vector<Position> merged;
            Json event_counts = Json::object();
            for (size_t j = 0; j < jobs.size(); ++j)
                if (jobs[j].first == int(i)) {
                    auto family = eventInfo(*inputs[i].problem, jobs[j].second).family;
                    event_counts[family] =
                        event_counts.value(family, size_t(0)) + positions[j].size();
                    merged.insert(merged.end(), positions[j].begin(), positions[j].end());
                }
            std::sort(merged.begin(), merged.end(), priorityLess);
            for (const auto &p : merged) {
                if (int(selected_all[i].size()) < all_capacity)
                    selected_all[i].push_back(p);
            }
            candidate_sources.push_back({{"mine", inputs[i].mine + 1},
                                         {"split", inputs[i].split},
                                         {"instances", inputs[i].samples.size()},
                                         {"candidate_positions", merged.size()},
                                         {"by_event_family", event_counts}});
        }
        struct Target {
            Position position;
            size_t slot;
        };
        std::vector<std::vector<Target>> targets(jobs.size());
        std::map<std::pair<int, int>, size_t> job_index;
        for (size_t j = 0; j < jobs.size(); ++j)
            job_index[jobs[j]] = j;
        for (size_t i = 0; i < inputs.size(); ++i) {
            all[i].resize(selected_all[i].size());
            for (size_t k = 0; k < selected_all[i].size(); ++k)
                targets[job_index.at({int(i), selected_all[i][k].sample})].push_back(
                    {selected_all[i][k], k});
        }
        // Position metadata is no longer needed after selections have been made.
        positions.clear();
        positions.shrink_to_fit();
        pool.run(jobs.size(), [&](size_t job) {
            if (targets[job].empty())
                return;
            auto [i, sample] = jobs[job];
            auto &wanted = targets[job];
            std::sort(wanted.begin(), wanted.end(), [](const Target &a, const Target &b) {
                return a.position.step < b.position.step;
            });
            size_t next = 0;
            auto info = eventInfo(*inputs[i].problem, sample);
            trajectory(
                inputs[i], sample, trajectory_seed,
                [&](int step, int truck, const auto &mask, int legal, double time, auto &sol) {
                    if (next == wanted.size() || wanted[next].position.step != step)
                        return;
                    Snapshot shot{
                        inputs[i].problem->getHeuristicValue(sample, truck, mask, range, &sol),
                        mask};
                    Json source = {{"scenario", "C" + std::to_string(inputs[i].mine + 1) + "_D1"},
                                   {"split", inputs[i].split},
                                   {"sample", sample},
                                   {"instance_fnv1a64", inputs[i].hashes.at(sample)},
                                   {"step", step},
                                   {"time", time},
                                   {"legal_actions", legal},
                                   {"event_family", info.family},
                                   {"first_disturbance_time", info.first_time},
                                   {"after_disturbance", !info.normal && time >= info.first_time}};
                    while (next < wanted.size() && wanted[next].position.step == step) {
                        const auto &t = wanted[next++];
                        require(t.position.time == time && t.position.legal_actions == legal,
                                "trajectory replay changed");
                        source["sampling_priority"] = t.position.priority;
                        all[i][t.slot] = {shot, source};
                    }
                });
            require(next == wanted.size(), "trajectory replay missed selected positions");
        });
        audit = {{"trajectory_seed", trajectory_seed},
                 {"snapshot_seed", sampling_seed},
                 {"candidate_sources", candidate_sources},
                 {"sampling", "uniform_random_priorities_without_replacement_within_mine"},
                 {"trajectory_policy", "uniform_legal_actions"},
                 {"feature_materialization", "selected_positions_only"}};
    }
    Reference select(int count) const {
        require(count > 0, "snapshot count must be positive");
        Reference out;
        const auto &banks = all;
        std::vector<size_t> mines;
        for (size_t i = 0; i < banks.size(); ++i)
            mines.push_back(i);
        // Round-robin prefixes are nested for all E and differ by at most one
        // state per mine. Each retained state has weight 1/E in D_sem.
        for (int k = 0; k < count; ++k) {
            size_t i = mines[size_t(k) % mines.size()], slot = size_t(k) / mines.size();
            require(slot < banks.at(i).size(),
                    std::string("insufficient distinct trajectory positions for mixed source") +
                        " E=" + std::to_string(count) + " in C" + std::to_string(i + 1) +
                        "; increase input instances");
            const auto &item = banks[i][slot];
            out.shots.push_back(item.shot);
            out.sources.push_back(item.source);
        }
        out.weights.assign(out.shots.size(), 1. / out.shots.size());
        out.summary = summarize(out.sources);
        out.summary["source"] = "mixed";
        return out;
    }
};
} // namespace gpaco::snapshots
