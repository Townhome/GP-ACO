#pragma once
#include "gp/model.h"
namespace gpaco {
struct TrainingOptions {
    int population = 100, tasks = 20, snapshots = 2000, cells = 200, depth = 6;
    int anchors = 5, initial_pool = 2000, tournament = 7, kmeans = 50;
    int train_limit = 0, log_every = 5, objective_grain = 1, semantic_grain = 64;
    uint64_t rule_budget = 102000;
    double mutation = .15, crossover = 1, gating = .5, kmeans_tolerance = 1e-4;
    Json json() const {
        return {{"population", population},
                {"tasks", tasks},
                {"snapshots", snapshots},
                {"cells", cells},
                {"depth", depth},
                {"anchors", anchors},
                {"initial_pool", initial_pool},
                {"tournament", tournament},
                {"kmeans", kmeans},
                {"train_limit", train_limit},
                {"rule_budget", rule_budget},
                {"mutation", mutation},
                {"crossover", crossover},
                {"gating", gating},
                {"kmeans_tolerance", kmeans_tolerance},
                {"training_scenarios", {"C1_D1", "C2_D1", "C3_D1"}},
                {"reference_source", "D1_train_only"},
                {"anchor_updates", false},
                {"normalization", "union_of_D1_train_only"}};
    }
};
void trainFrozenJointArchive(const TrainingOptions &options, const fs::path &data_root,
                             const fs::path &output, uint64_t seed, Pool &pool, bool reuse = true);
} // namespace gpaco
