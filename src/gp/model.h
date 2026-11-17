#pragma once
#include "core/common.h"
#include "gp/tree_operations.h"
#include "core/random.h"
namespace gpaco {
struct Snapshot {
    std::vector<std::vector<Real>> action_features;
    std::vector<int> action_mask;
};
struct GPRule {
    GOftn *tree;
    Real fitness = 1e9;
    std::vector<int> cached_decisions;
    std::vector<Real> coords;
    int task_id = -1;
    std::vector<Real> fitness_vector;

    GPRule() : tree(nullptr) {}
    ~GPRule() { TreeOperations::deleteTree(tree); }
    GPRule(const GPRule &rhs)
        : tree(rhs.tree ? rhs.tree->clone() : nullptr), fitness(rhs.fitness),
          cached_decisions(rhs.cached_decisions), coords(rhs.coords), task_id(rhs.task_id),
          fitness_vector(rhs.fitness_vector) {}
    GPRule(GPRule &&rhs)
        : tree(rhs.tree), fitness(rhs.fitness), cached_decisions(rhs.cached_decisions),
          coords(rhs.coords), task_id(rhs.task_id), fitness_vector(rhs.fitness_vector) {
        rhs.tree = nullptr;
    }
    GPRule &operator=(const GPRule &rhs) {
        if (&rhs != this) {
            TreeOperations::deleteTree(tree);
            tree = rhs.tree ? rhs.tree->clone() : nullptr;
            fitness = rhs.fitness;
            cached_decisions = rhs.cached_decisions;
            coords = rhs.coords;
            task_id = rhs.task_id;
            fitness_vector = rhs.fitness_vector;
        }
        return *this;
    }
    GPRule &operator=(GPRule &&rhs) {
        if (&rhs != this) {
            TreeOperations::deleteTree(tree);
            tree = rhs.tree;
            fitness = rhs.fitness;
            cached_decisions = rhs.cached_decisions;
            coords = rhs.coords;
            task_id = rhs.task_id;
            fitness_vector = rhs.fitness_vector;
            rhs.tree = nullptr;
        }
        return *this;
    }
};
using Rule = std::shared_ptr<GPRule>;
class SemanticEngine {
    std::vector<Snapshot> reference_set;
    // Double weights avoid accumulation drift when Real is configured as float.
    std::vector<double> reference_weights;

  public:
    void setReferenceSet(const std::vector<Snapshot> &shots,
                         const std::vector<double> &weights = {}) {
        reference_set = shots;
        reference_weights = weights;
    }

    // D_sem = 1/M * sum( I(pi_i != pi_j) )
    Real calculateDistance(const GPRule &r1, const GPRule &r2) {
        double mismatches = 0;
        for (size_t i = 0; i < reference_set.size(); ++i) {
            if (r1.cached_decisions[i] != r2.cached_decisions[i]) {
                mismatches += reference_weights.empty() ? 1.0 : reference_weights.at(i);
            }
        }
        return reference_weights.empty() ? mismatches / reference_set.size() : mismatches;
    }

    void embedding(GPRule &rule, const std::vector<std::shared_ptr<GPRule>> &anchors) {
        rule.coords.resize(anchors.size());
        for (size_t i = 0; i < anchors.size(); ++i) {
            rule.coords[i] = calculateDistance(rule, *anchors[i]);
        }
    }
};
class ArchiveBase : public TreeOperations {
  protected:
    int m_id_rnd = -1, m_anchor_size = 5, m_cell_size = 200, m_num_task = 20;
    int m_tournament_size = 7, m_kmeans_iter = 50;
    Real m_gating_ratio = .5, m_kmeans_tol = 1e-4;
    std::unordered_map<std::string, Real> m_data_range;
    std::vector<std::vector<int>> m_task_group;
    std::vector<Real> m_best_fitness;
    SemanticEngine m_engine;
    std::vector<Snapshot> m_reference_set;
    std::map<int, Rule> m_cells;
    std::vector<std::vector<Real>> m_centroids;
    std::vector<Rule> m_anchors;
    int getNearestCentroid(const std::vector<Real> &p) {
        int best = 0;
        Real min_d = 1e9;
        for (int i = 0; i < m_centroids.size(); ++i) {
            Real d = 0;
            for (size_t k = 0; k < p.size(); ++k)
                d += std::pow(p[k] - m_centroids[i][k], 2);
            if (d < min_d) {
                min_d = d;
                best = i;
            }
        }
        return best;
    }
    int tournamentSelect() {
        int best = -1;
        auto it = m_cells.begin();
        for (int i = 0; i < m_tournament_size; ++i) {
            it = m_cells.begin();
            int rnd = randomStream(m_id_rnd).uniform.nextNonStd<int>(0, m_cells.size());
            std::advance(it, rnd);
            auto candidate = it->second;
            if (best == -1 || candidate->fitness < m_cells[best]->fitness) {
                best = it->first;
            }
        }
        return best;
    }
};
Json treeJson(GOftn *tree);
GOftn *readTree(const Json &json, int depth = 0);
Json ruleJson(const Rule &rule);
Rule readRule(const Json &json);
struct Models {
    std::vector<Rule> rules;
    std::unordered_map<std::string, Real> range;
    double gpaco_rho = .3, gpaco_beta = 4, temperature = .1;
};
} // namespace gpaco
