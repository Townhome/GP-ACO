#pragma once
#include "gp/model.h"
#include "problem/simulator.h"
namespace gpaco {
struct SearchOptions {
    int budget = 1000, ants = 40, local_trials = 100;
    double alpha = .5;
};
struct Dataset {
    int domain = 0;
    std::unique_ptr<TruckBase> truth, forecast;
    Dataset(const fs::path &root, int mine, int scale);
    void reveal(int environment);
};
struct Result {
    double objective = std::numeric_limits<double>::infinity();
    uint64_t evaluations = 0, evaluation_hash = 0, route_hash = 0, prefix_hash = 0,
             prefix_checks = 0;
    uint64_t repair_evaluations = 0, local_evaluations = 0;
    int generations = 0;
};
class Search {
    const SearchOptions &o;
    Dataset &data;
    TruckBase *pro;
    const Models &models;
    int sample, rnd, env = 0, generation = 0, local_left = 0, stagnant = 0;
    uint64_t seed;
    Ptr prefix, incumbent, generation_best, local;
    Matrix tau;
    double minimum = 1e-6, maximum = 1, search_best = std::numeric_limits<double>::infinity();
    size_t problem_size = 1;
    std::mt19937_64 rng;
    Result result;
    Ptr makePrefix(const Ptr &previous, double cutoff);
    Ptr repair(const Routes *source);
    void checkPrefix(const Sol &candidate) const;
    void publish(const Ptr &candidate, const char *type);
    Ptr construct(size_t ant) const;
    void update();
    void localTrial();

  public:
    Search(const SearchOptions &options, Dataset &dataset, const Models &model, int instance,
           uint64_t seed);
    ~Search();
    Search(const Search &) = delete;
    Search &operator=(const Search &) = delete;
    void run(int environment);
    bool active() const { return result.evaluations < uint64_t(o.budget); }
    const Result &outcome() const { return result; }
    const Ptr &bestSolution() const { return incumbent; }
};
Models loadModel(const fs::path &path, int library = 5);
} // namespace gpaco
