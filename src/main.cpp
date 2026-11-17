#include "gp/training.h"
#include "aco/search.h"
#include <iostream>
namespace gpaco {
struct Options {
    fs::path data = "data", output = "results";
    int threads = 0;
    uint64_t seed = 2026;
    bool quick = false;
};
Options parse(int argc, char **argv) {
    Options o;
    std::set<std::string> seen;
    for (int i = 1; i < argc; ++i) {
        std::string option = argv[i];
        require(seen.insert(option).second, "Repeated option: " + option);
        if (option == "--quick") {
            o.quick = true;
            continue;
        }
        require(i + 1 < argc, "Missing value for " + option);
        std::string value = argv[++i];
        if (option == "--data")
            o.data = value;
        else if (option == "--output")
            o.output = value;
        else if (option == "--threads" || option == "--seed") {
            require(!value.empty() && value.front() != '-',
                    "Expected a non-negative integer for " + option);
            size_t used = 0;
            auto number = std::stoull(value, &used);
            require(used == value.size(), "Invalid integer for " + option);
            if (option == "--threads") {
                require(number <= uint64_t(std::numeric_limits<int>::max()),
                        "Thread count too large");
                o.threads = int(number);
            } else
                o.seed = number;
        } else
            require(false, "Unknown option: " + option);
    }
    o.data = fs::absolute(o.data);
    o.output = fs::absolute(o.output);
    return o;
}
int run(const Options &o) {
    TrainingOptions training;
    SearchOptions search;
    if (o.quick) {
        training.population = 8;
        training.initial_pool = 32;
        training.rule_budget = 96;
        training.tasks = 2;
        training.train_limit = 10;
        training.snapshots = 60;
        training.cells = 12;
        training.depth = 4;
        training.kmeans = 5;
        training.log_every = 1;
        search.budget = 200;
    }
    // Verify all inputs before creating or reusing a model.
    Json inputs = Json::object();
    for (int mine = 1; mine <= 3; ++mine) {
        auto name = "C" + std::to_string(mine) + "_D1_train.txt";
        inputs[name] = fileHash(o.data / "graph" / name);
        for (int scale = 1; scale <= 4; ++scale) {
            name = "C" + std::to_string(mine) + "_D" + std::to_string(scale) + "_test.txt";
            inputs[name] = fileHash(o.data / "graph" / name);
        }
    }
    Json protocol = {{"schema", "GPACO_RELEASE_RUN_1"},
                     {"quick", o.quick},
                     {"seed", o.seed},
                     {"training", training.json()},
                     {"inputs", inputs},
                     {"online",
                      {{"environments", 5},
                       {"samples", o.quick ? 5 : 200},
                       {"budget", search.budget},
                       {"ants", search.ants},
                       {"library", 5},
                       {"alpha", search.alpha},
                       {"beta", 4},
                       {"rho", .3},
                       {"temperature", .1},
                       {"local_trials", search.local_trials}}}};
    fs::create_directories(o.output);
    FileLock lock(o.output / ".run.lock");
    auto manifest = o.output / "run.json";
    if (fs::exists(manifest))
        require(readJson(manifest).at("protocol") == protocol,
                "Output directory contains a different run; choose a new --output");
    atomicWrite(manifest, Json{{"status", "running"}, {"protocol", protocol}}.dump(2));
    Pool pool(o.threads);
    std::cout << "GP-ACO: " << (o.quick ? "quick validation" : "full protocol")
              << ", workers=" << pool.workers() << std::endl;
    try {
        trainFrozenJointArchive(training, o.data, o.output / "offline", o.seed, pool);
        const auto model_file = o.output / "offline/model.json";
        const auto model_hash = fileHash(model_file);
        auto model = loadModel(model_file);
        std::ostringstream summary;
        summary << "scenario,instances,environments,mean_rmse,mean_instance_seconds\n"
                << std::setprecision(17);
        uint64_t total_evaluations = 0;
        for (int mine = 1; mine <= 3; ++mine)
            for (int scale = 1; scale <= 4; ++scale) {
                std::string scenario = "C" + std::to_string(mine) + "_D" + std::to_string(scale);
                Dataset data(o.data, mine, scale);
                std::vector<int> selected;
                if (o.quick)
                    selected = {0, 40, 80, 120, 160};
                else {
                    selected.resize(200);
                    std::iota(selected.begin(), selected.end(), 0);
                }
                // Register streams only on the coordinator; workers own disjoint searches.
                std::vector<std::unique_ptr<Search>> searches;
                for (int sample : selected)
                    searches.push_back(
                        std::make_unique<Search>(search, data, model, sample, o.seed));
                std::vector<std::vector<Json>> rows(selected.size()), routes(selected.size());
                double mean_rmse = 0, total_seconds = 0;
                for (int environment = 0; environment < 5; ++environment) {
                    data.reveal(environment);
                    pool.run(selected.size(), [&](size_t i) {
                        auto start = Clock::now();
                        searches[i]->run(environment);
                        double duration = seconds(start);
                        const auto &r = searches[i]->outcome();
                        rows[i].push_back({{"instance", selected[i]},
                                           {"event_group", selected[i] / 40 + 1},
                                           {"environment", environment},
                                           {"rmse", r.objective},
                                           {"evaluations", r.evaluations},
                                           {"seconds", duration},
                                           {"repair_evaluations", r.repair_evaluations},
                                           {"local_evaluations", r.local_evaluations},
                                           {"prefix_hash", r.prefix_hash},
                                           {"route_hash", r.route_hash},
                                           {"evaluation_hash", r.evaluation_hash}});
                        routes[i].push_back(searches[i]->bestSolution()->variable().m_truck_routes);
                    });
                }
                std::ostringstream csv;
                csv << "instance,event_group,environment,rmse,evaluations,seconds,repair_"
                       "evaluations,local_evaluations,prefix_hash,route_hash,evaluation_hash\n"
                    << std::setprecision(17);
                const std::vector<std::string> columns = {
                    "instance",    "event_group", "environment",        "rmse",
                    "evaluations", "seconds",     "repair_evaluations", "local_evaluations",
                    "prefix_hash", "route_hash",  "evaluation_hash"};
                for (const auto &instance : rows)
                    for (const auto &row : instance) {
                        for (size_t c = 0; c < columns.size(); ++c) {
                            if (c)
                                csv << ',';
                            csv << row.at(columns[c]).dump();
                        }
                        csv << '\n';
                        mean_rmse += row.at("rmse").get<double>() / (5 * selected.size());
                        total_seconds += row.at("seconds").get<double>();
                        total_evaluations += row.at("evaluations").get<uint64_t>();
                    }
                atomicWrite(o.output / (scenario + ".csv"), csv.str());
                atomicWrite(o.output / (scenario + "_routes.json"),
                            Json{{"instances", selected}, {"routes", routes}}.dump());
                summary << scenario << ',' << selected.size() << ",5," << mean_rmse << ','
                        << total_seconds / selected.size() << '\n';
                std::cout << "[test] " << scenario << " instances=" << selected.size()
                          << " mean RMSE=" << mean_rmse << std::endl;
            }
        require(fileHash(model_file) == model_hash, "Frozen model changed during testing");
        atomicWrite(o.output / "summary.csv", summary.str());
        atomicWrite(manifest, Json{{"status", "complete"},
                                   {"protocol", protocol},
                                   {"workers", pool.workers()},
                                   {"model_hash", model_hash},
                                   {"objective_evaluations", total_evaluations}}
                                  .dump(2));
        std::cout << "Completed: " << o.output.string() << std::endl;
        return 0;
    } catch (...) {
        atomicWrite(manifest, Json{{"status", "failed"}, {"protocol", protocol}}.dump(2));
        throw;
    }
}
} // namespace gpaco
int main(int argc, char **argv) {
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--help") {
            std::cout << "GP-ACO: shared D1 offline training + online search on all 12 scenarios.\n"
                      << "Run from the repository root:\n"
                      << "  gpaco [--threads 0] [--seed 2026] [--data data] [--output results] "
                         "[--quick]\n"
                      << "--quick reduces training and tests one instance from each E1-E5 group.\n";
            return 0;
        }
    try {
        return gpaco::run(gpaco::parse(argc, argv));
    } catch (const std::exception &error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
