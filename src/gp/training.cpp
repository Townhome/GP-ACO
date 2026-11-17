#include "gp/training.h"
#include "gp/snapshots.h"
#include <iostream>
namespace gpaco {
struct Data {
    fs::path path;
    std::string scenario, split;
    std::unique_ptr<TruckBase> pro;
    std::vector<uint64_t> instance_hashes;
    Data(const fs::path &root, int mine, int scale, const std::string &part) : split(part) {
        scenario = "C" + std::to_string(mine) + "_D" + std::to_string(scale);
        path = root / "graph" / (scenario + "_" + split + ".txt");
        check(fs::is_regular_file(path), "missing input " + path.string());
        // Fingerprint actual records, ignoring separator labels and CRLF/LF.
        std::ifstream in(path);
        std::string line, block;
        while (std::getline(in, line)) {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            if (line.find('=') != std::string::npos) {
                if (!block.empty())
                    instance_hashes.push_back(hashText(block));
                block.clear();
            } else
                block += line + "\n";
        }
        if (!block.empty())
            instance_hashes.push_back(hashText(block));
        const int shovels[] = {3, 5, 7, 9};
        ParamMap p;
        p["ShovelSize"] = shovels[scale - 1];
        p["TruckSize"] = shovels[scale - 1] * (mine == 3 ? 3 : 2);
        p["UnloadSize"] = mine + 1;
        p["WindowSize"] = Real(600);
        p["DataFile"] = path.string();

        pro = std::make_unique<TruckBase>(p);
        check(pro->getNumSample() > 0 && size_t(pro->getNumSample()) == instance_hashes.size(),
              "record count mismatch " + path.string());
    }
    Json identity() const {
        return {{"file", fs::relative(path, path.parent_path().parent_path()).generic_string()},
                {"fnv1a64", fileHash(path)},
                {"instances", pro->getNumSample()},
                {"scenario", scenario},
                {"split", split}};
    }
};
int decision(const Rule &r, const Snapshot &snapshot) {
    int best = -1;
    double score = -std::numeric_limits<double>::infinity();
    for (size_t a = 0; a < snapshot.action_mask.size(); ++a)
        if (snapshot.action_mask[a]) {
            double value = r->tree->eval(snapshot.action_features[a]);
            if (std::isnan(value))
                value = -std::numeric_limits<double>::infinity();
            if (best < 0 || value > score) {
                best = int(a);
                score = value;
            }
        }
    check(best >= 0, "snapshot has no legal action");
    return best;
}
double greedy(TruckBase &pro, int sample, const Rule &r, const Range &range, uint64_t seed) {
    Sol sol;
    sol.variable().m_sample_id = sample;
    pro.resetSolutionWithSeed(&sol, seed);
    for (;;) {
        int t = pro.choseTruck(sample, &sol);
        if (t < 0)
            break;
        auto mask = pro.updateMask(sample, t, &sol);
        if (std::find(mask.begin(), mask.end(), 1) == mask.end())
            break;
        Snapshot shot{pro.getHeuristicValue(sample, t, mask, range, &sol), mask};
        pro.updateState(sample, t, decision(r, shot), &sol);
    }
    double fit = pro.evaluate(sample, &sol);
    check(std::isfinite(fit) && fit >= 0, "invalid greedy RMSE");
    return fit;
}
Range trainingRange(const std::vector<std::unique_ptr<Data>> &data) {
    Range out = data[0]->pro->getDataRange();
    for (size_t i = 1; i < data.size(); ++i)
        for (const auto &kv : data[i]->pro->getDataRange()) {
            if (kv.first.rfind("min_", 0) == 0)
                out[kv.first] = std::min(out[kv.first], kv.second);
            else
                out[kv.first] = std::max(out[kv.first], kv.second);
        }
    for (const auto &kv : out)
        check(std::isfinite(kv.second), "invalid training normalization");
    check(out.at("max_grade") > out.at("min_grade"), "zero grade range");
    for (const char *k : {"max_quantity", "max_capacity", "max_port", "max_time", "max_shift"})
        check(out.at(k) > 0, "zero normalization divisor");
    return out;
}
class JointTrainer : public ArchiveBase {
  protected:
    const TrainingOptions &s;

    uint64_t seed, evaluated = 0, objectives = 0;
    Pool &pool;
    std::vector<std::unique_ptr<Data>> &train;
    std::vector<std::pair<int, int>> instances;
    std::vector<double> weights;
    Json reference_export;
    std::ofstream history;
    std::map<std::string, double> phase_seconds;
    std::map<std::string, uint64_t> phase_jobs;
    double training_seconds = 0;
    void parallel(const std::string &phase, size_t count, const std::function<void(size_t)> &work,
                  size_t grain = 1) {
        auto started = Clock::now();
        pool.run(count, work, grain);
        phase_seconds[phase] += std::chrono::duration<double>(Clock::now() - started).count();
        phase_jobs[phase] += count;
    }
    void evaluateBatch(std::vector<Rule> &rules, uint64_t first) {
        size_t group_size = m_task_group.at(0).size(), shots = m_reference_set.size();
        for (size_t i = 0; i < rules.size(); ++i) {
            rules[i]->task_id = int((first + i) % m_num_task);
            check(m_task_group.at(rules[i]->task_id).size() == group_size,
                  "unequal task group sizes");
            rules[i]->fitness_vector.resize(group_size);
            rules[i]->cached_decisions.resize(shots);
        }
        // Thousands of independent rule/instance jobs keep large CPU allocations
        // busy even when the GP population is smaller than the worker count.
        parallel(
            "objective", rules.size() * group_size,
            [&](size_t job) {
                auto &r = rules[job / group_size];
                size_t j = job % group_size;
                auto [mine, sample] = instances.at(m_task_group.at(r->task_id).at(j));
                r->fitness_vector[j] = greedy(*train[mine]->pro, sample, r, m_data_range,
                                              instanceSeed(seed, mine, sample, 0, 0));
            },
            s.objective_grain);
        size_t blocks = (shots + s.semantic_grain - 1) / s.semantic_grain;
        parallel("semantics", rules.size() * blocks, [&](size_t job) {
            auto &r = rules[job / blocks];
            size_t start = (job % blocks) * s.semantic_grain;
            for (size_t i = start; i < std::min(shots, start + s.semantic_grain); ++i)
                r->cached_decisions[i] = decision(r, m_reference_set[i]);
        });
        // Fixed-order reduction after workers finish: no shared RNG or best-value writes.
        for (const auto &r : rules) {
            const auto &group = m_task_group.at(r->task_id);
            for (size_t j = 0; j < group.size(); ++j)
                m_best_fitness[group[j]] = std::min(m_best_fitness[group[j]], r->fitness_vector[j]);
            objectives += group.size();
        }
        evaluated += rules.size();
    }
    std::vector<Rule> chooseAnchors(std::vector<Rule> &candidates) {
        std::sort(candidates.begin(), candidates.end(),
                  [](const Rule &a, const Rule &b) { return a->fitness < b->fitness; });
        int valid = std::min(int(candidates.size()),
                             std::max(int(candidates.size() * m_gating_ratio), m_anchor_size + 1));
        check(valid >= m_anchor_size, "not enough gated anchor candidates");
        std::vector<Rule> chosen{candidates[0]};
        std::vector<bool> used(valid, false);
        used[0] = true;
        std::vector<Real> nearest(valid, Real(1e9));
        while (int(chosen.size()) < m_anchor_size) {
            auto latest = chosen.back();
            parallel(
                "anchor_distance", valid,
                [&](size_t i) {
                    if (!used[i])
                        nearest[i] = std::min(nearest[i],
                                              m_engine.calculateDistance(*candidates[i], *latest));
                },
                s.objective_grain);
            int best = -1;
            Real distance = -1;
            for (int i = 0; i < valid; ++i)
                if (!used[i] && nearest[i] > distance) {
                    best = i;
                    distance = nearest[i];
                }
            check(best >= 0, "anchor selection failed");
            used[best] = true;
            chosen.push_back(candidates[best]);
        }
        return chosen;
    }
    void embed(const std::vector<Rule> &rules) {
        parallel(
            "embedding", rules.size(), [&](size_t i) { m_engine.embedding(*rules[i], m_anchors); },
            s.objective_grain);
    }
    void buildCVT(const std::vector<std::vector<Real>> &points) {
        std::vector<int> order(points.size());
        std::iota(order.begin(), order.end(), 0);
        randomStream(m_id_rnd).uniform.shuffle(order.begin(), order.end());
        m_centroids.clear();
        for (int i = 0; i < m_cell_size; ++i)
            m_centroids.push_back(points[order[i % points.size()]]);
        std::vector<int> labels(points.size());
        for (int iteration = 0; iteration < m_kmeans_iter; ++iteration) {
            parallel(
                "cvt_assignment", points.size(),
                [&](size_t i) { labels[i] = getNearestCentroid(points[i]); }, s.objective_grain);
            // Reduce in original point order to preserve floating-point tie
            // decisions and cell assignments, regardless of worker timing.
            std::vector<std::vector<Real>> sums(m_cell_size, std::vector<Real>(m_anchor_size, 0));
            std::vector<int> counts(m_cell_size, 0);
            for (size_t i = 0; i < points.size(); ++i) {
                for (int k = 0; k < m_anchor_size; ++k)
                    sums[labels[i]][k] += points[i][k];
                ++counts[labels[i]];
            }
            Real shift = 0;
            for (int cell = 0; cell < m_cell_size; ++cell)
                if (counts[cell] > 0) {
                    std::vector<Real> next(m_anchor_size);
                    Real squared = 0;
                    for (int k = 0; k < m_anchor_size; ++k) {
                        next[k] = sums[cell][k] / counts[cell];
                        Real delta = next[k] - m_centroids[cell][k];
                        squared += delta * delta;
                    }
                    shift += squared;
                    m_centroids[cell] = std::move(next);
                }
            if (shift < m_kmeans_tol)
                break;
        }
    }
    void archive(const std::vector<Rule> &rules, bool embedded = false) {
        if (!embedded)
            embed(rules);
        std::vector<int> labels(rules.size());
        parallel(
            "archive_assignment", rules.size(),
            [&](size_t i) { labels[i] = getNearestCentroid(rules[i]->coords); }, s.objective_grain);
        for (size_t i = 0; i < rules.size(); ++i) {
            auto found = m_cells.find(labels[i]);
            if (found == m_cells.end() || rules[i]->fitness < found->second->fitness)
                m_cells[labels[i]] = rules[i];
        }
    }
    void aggregate(const std::vector<Rule> &rules) {
        for (const auto &r : rules) {
            const auto &group = m_task_group.at(r->task_id);
            double gap = 0;
            for (size_t j = 0; j < group.size(); ++j) {
                double best = m_best_fitness[group[j]];
                gap += (r->fitness_vector[j] - best) / std::max(best, 1e-12);
            }
            r->fitness = gap / group.size();
            check(std::isfinite(r->fitness), "non-finite training fitness");
        }
    }
    void log(int batch) {
        std::vector<Rule> elites;
        Real fitness = 0;
        for (const auto &cell : m_cells) {
            elites.push_back(cell.second);
            fitness += cell.second->fitness;
        }
        std::vector<double> disagreement(m_reference_set.size(), 0.);
        // Mean pairwise Hamming distance can be obtained from per-state action
        // counts in O(states*cells), instead of O(states*cells*cells).
        if (elites.size() > 1)
            parallel(
                "diversity", m_reference_set.size(),
                [&](size_t i) {
                    std::vector<size_t> counts(m_reference_set[i].action_mask.size(), 0);
                    for (const auto &r : elites)
                        ++counts.at(r->cached_decisions[i]);
                    double same = 0;
                    for (auto n : counts)
                        same += double(n) * double(n);
                    double n = double(elites.size());
                    disagreement[i] = (n * n - same) / (n * (n - 1)) * weights[i];
                },
                s.semantic_grain);
        double diversity = std::accumulate(disagreement.begin(), disagreement.end(), 0.);
        history << batch << ',' << evaluated << ',' << objectives << ',' << m_cells.size() << ','
                << std::setprecision(17) << fitness / Real(elites.size()) << ',' << diversity
                << '\n';
        history.flush();
        if (batch % s.log_every == 0 || evaluated == s.rule_budget)
            std::cout << "[joint GP] "
                      << "shared_D1"
                      << " seed=" << seed << " rules=" << evaluated << '/' << s.rule_budget
                      << " cells=" << m_cells.size() << std::endl;
    }

  public:
    JointTrainer(const TrainingOptions &settings, uint64_t random_seed, Pool &executor,
                 std::vector<std::unique_ptr<Data>> &datasets, const fs::path &folder,
                 const snapshots::Reference &reference)
        : s(settings), seed(random_seed), pool(executor), train(datasets) {
        m_anchor_size = s.anchors;
        m_cell_size = s.cells;
        m_num_task = s.tasks;
        m_tournament_size = s.tournament;
        m_gating_ratio = s.gating;
        m_kmeans_iter = s.kmeans;
        m_kmeans_tol = Real(s.kmeans_tolerance);
        m_data_range = trainingRange(train);
        setOperators({FunType::Add, FunType::Subtract, FunType::Multiply, FunType::Divide,
                      FunType::Min, FunType::Max, FunType::Sin, FunType::If, FunType::Sigmoid});
        std::vector<int> terminals(16);
        std::iota(terminals.begin(), terminals.end(), 0);
        setTerminals(terminals);
        setMaxDepth(s.depth);
        setMutationP(s.mutation);
        m_task_group.resize(m_num_task);
        std::mt19937_64 shuffle(instanceSeed(seed, 37, 0, 0, 0));
        int expected = -1;
        for (int mine = 0; mine < 3; ++mine) {
            int count = s.train_limit ? s.train_limit : train[mine]->pro->getNumSample();
            check(count <= train[mine]->pro->getNumSample() && count % s.tasks == 0,
                  "training count must be divisible by task count");
            if (expected < 0)
                expected = count;
            check(expected == count, "joint training requires equal sample counts per mine");
            std::vector<int> order(count);
            std::iota(order.begin(), order.end(), 0);
            std::shuffle(order.begin(), order.end(), shuffle);
            for (int i = 0; i < count; ++i) {
                m_task_group[i % s.tasks].push_back(int(instances.size()));
                instances.emplace_back(mine, order[i]);
            }
        }
        m_reference_set = reference.shots;
        weights = reference.weights;
        reference_export = snapshots::json(reference);
        m_best_fitness.assign(instances.size(), std::numeric_limits<Real>::max());
        m_engine.setReferenceSet(m_reference_set, weights);
        history.open(folder / "history.csv", std::ios::trunc);
        check(bool(history), "cannot open history.csv");
        history << "batch,rule_evaluations,objective_evaluations,occupied_cells,training_relative_"
                   "gap,training_diversity\n";
        m_id_rnd = addRandom(.0001 + .9998 * double(instanceSeed(seed, 31, 0, 0, 0) >> 11) /
                                         9007199254740992.0);
    }
    ~JointTrainer() {
        if (m_id_rnd >= 0)
            deleteRandom(m_id_rnd);
    }
    Json run(const Json &protocol, const fs::path &folder) {
        auto training_started = Clock::now();
        std::vector<Rule> initial(s.initial_pool);
        // Tree generation stays on the coordinator; only deterministic evaluation is parallel.
        for (auto &r : initial) {
            r = std::make_shared<GPRule>();
            r->tree = halfAndHalf(m_id_rnd);
        }

        evaluateBatch(initial, 0);
        aggregate(initial);
        m_anchors = chooseAnchors(initial);
        embed(initial);
        std::vector<std::vector<Real>> coords;
        for (auto &r : initial)
            coords.push_back(r->coords);
        buildCVT(coords);
        archive(initial, true);
        log(0);
        initial.clear();
        int batch = 0;
        while (evaluated < s.rule_budget) {
            int count = int(std::min<uint64_t>(s.population, s.rule_budget - evaluated));
            std::vector<Rule> children;
            while (int(children.size()) < count) {
                int a = tournamentSelect(), b = a;
                // A collapsed archive can still mutate its sole parent; never spin forever.
                if (m_cells.size() > 1) {
                    for (int tries = 0; tries < 32 && b == a; ++tries)
                        b = tournamentSelect();
                    if (b == a) {
                        auto it = m_cells.begin();
                        if (it->first == a)
                            ++it;
                        b = it->first;
                    }
                }
                auto p = m_cells.at(a), q = m_cells.at(b);
                auto x = std::make_shared<GPRule>(*p), y = std::make_shared<GPRule>(*q);
                if (s.crossover >= 1. ||
                    (s.crossover > 0 && randomStream(m_id_rnd).uniform.next() < s.crossover)) {
                    crossoverTree(x->tree, q->tree, m_id_rnd);
                    crossoverTree(y->tree, p->tree, m_id_rnd);
                }
                mutateTree(&x->tree, m_id_rnd);
                mutateTree(&y->tree, m_id_rnd);
                children.push_back(x);
                if (int(children.size()) < count)
                    children.push_back(y);
            }
            uint64_t first = evaluated;
            evaluateBatch(children, first);
            std::vector<Rule> all = children;
            for (const auto &cell : m_cells)
                all.push_back(cell.second);
            aggregate(all);
            archive(children);
            log(++batch);
        }
        training_seconds = std::chrono::duration<double>(Clock::now() - training_started).count();
        Json archive = Json::array(), anchors = Json::array();
        for (const auto &cell : m_cells) {
            auto row = ruleJson(cell.second);
            row["cell"] = cell.first;
            row["rule_id"] = "cell_" + std::to_string(cell.first);
            row["coords"] = cell.second->coords;
            row["training_decisions"] = cell.second->cached_decisions;
            archive.push_back(row);
        }
        for (const auto &anchor : m_anchors) {
            auto row = ruleJson(anchor);
            row["training_decisions"] = anchor->cached_decisions;
            anchors.push_back(row);
        }
        check(evaluated == s.rule_budget, "incorrect rule budget");
        size_t per_rule = m_task_group.at(0).size();
        check(objectives == evaluated * per_rule, "unequal objective budget");
        atomicWrite(folder / "reference.json", reference_export.dump());
        Json diagnostics = {
            {"instance_index", instances},
            {"task_groups", m_task_group},
            {"per_instance_best", m_best_fitness},
            {"rule_evaluations", evaluated},
            {"objective_evaluations", objectives},
            {"archive_cells", archive.size()},
            {"snapshot_weight_sum", std::accumulate(weights.begin(), weights.end(), 0.)}};

        atomicWrite(folder / "training.json", diagnostics.dump(2));

        Json performance = {{"phase_seconds", phase_seconds},
                            {"phase_jobs", phase_jobs},
                            {"worker_threads", pool.workers()},
                            {"training_seconds", training_seconds}};
        atomicWrite(folder / "performance.json", performance.dump(2));
        return {{"schema", "GPACO_JOINT_ARCHIVE_1"},
                {"protocol", protocol},
                {"archive", archive},
                {"anchors", anchors},
                {"centroids", m_centroids},
                {"normalization", m_data_range},
                {"frozen", true},
                {"reference_fnv1a64", fileHash(folder / "reference.json")},
                {"rule_evaluations", evaluated},
                {"objective_evaluations", objectives}};
    }
};
void trainFrozenJointArchive(const TrainingOptions &options, const fs::path &root,
                             const fs::path &folder, uint64_t seed, Pool &pool, bool resume) {
    const auto &s = options;
    uint64_t snapshot_seed = instanceSeed(seed, 0, 0, 71, 0);
    uint64_t trajectory_seed = instanceSeed(seed, 0, 0, 72, 0);
    fs::create_directories(folder);
    FileLock lock(folder / ".run.lock");
    std::vector<std::unique_ptr<Data>> train;
    std::vector<snapshots::Input> inputs;
    Json identities = Json::array(), training_hashes = Json::array();
    for (int mine = 1; mine <= 3; ++mine) {
        train.push_back(std::make_unique<Data>(root, mine, 1, "train"));
        auto &data = *train.back();
        int count = s.train_limit ? s.train_limit : data.pro->getNumSample();
        check(count > 0 && count <= data.pro->getNumSample() && count % s.tasks == 0,
              "training count must be positive, within the input and divisible by --tasks");
        snapshots::Input input{mine - 1, data.pro.get(), "train", {}, data.instance_hashes};
        for (int sample = 0; sample < count; ++sample) {
            input.samples.push_back(sample);
            training_hashes.push_back(data.instance_hashes.at(sample));
        }
        inputs.push_back(std::move(input));
        identities.push_back(data.identity());
    }
    for (const auto &input : inputs)
        check(input.samples.size() == inputs.front().samples.size(),
              "joint training requires equal D1 counts");
    Json protocol = {{"parameters", s.json()},
                     {"configuration",
                      {{"K", s.anchors},
                       {"P", s.initial_pool},
                       {"E", s.snapshots},
                       {"source", "mixed"},
                       {"id", "joint_shared"}}},
                     {"seed", seed},
                     {"data", identities},
                     {"training_record_hashes", training_hashes}};
    auto manifest = folder / "train_manifest.json";
    if (fs::exists(manifest)) {
        check(resume, "training output exists; use a new output directory");
        auto old = readJson(manifest);
        check(old.at("protocol") == protocol, "training data or options differ from saved model");
        if (old.at("status") == "complete") {
            for (auto it = old.at("artifacts").begin(); it != old.at("artifacts").end(); ++it)
                check(fileHash(folder / it.key()) == it.value().get<uint64_t>(),
                      "training artifact changed: " + it.key());
            std::cout << "[model reused] frozen joint archive " << (folder / "model.json").string()
                      << std::endl;
            return;
        }
    } else
        check(!fs::exists(folder / "model.json"), "untracked joint model already exists");
    atomicWrite(manifest, Json{{"status", "running"}, {"protocol", protocol}}.dump(2));
    auto started = Clock::now();
    try {
        auto range = trainingRange(train);
        snapshots::Bank bank(inputs, (s.snapshots + 2) / 3, range, trajectory_seed, snapshot_seed,
                             pool);
        auto reference = bank.select(s.snapshots);
        JointTrainer trainer(s, seed, pool, train, folder, reference);
        auto model = trainer.run(protocol, folder);
        for (const auto &row : model.at("archive")) {
            auto rule = readRule(row);
            check(treeJson(rule->tree) == row.at("tree"), "joint model tree round-trip mismatch");
        }
        for (size_t mine = 0; mine < train.size(); ++mine)
            check(train[mine]->identity() == identities.at(mine),
                  "D1 input changed during joint training");
        atomicWrite(folder / "model.json", model.dump(2));
        Json hashes = Json::object();
        for (const char *name :
             {"model.json", "reference.json", "training.json", "history.csv", "performance.json"})
            hashes[name] = fileHash(folder / name);
        atomicWrite(manifest,
                    Json{{"status", "complete"},
                         {"protocol", protocol},
                         {"artifacts", hashes},
                         {"seconds", std::chrono::duration<double>(Clock::now() - started).count()}}
                        .dump(2));
    } catch (...) {
        atomicWrite(manifest, Json{{"status", "failed"}, {"protocol", protocol}}.dump(2));
        throw;
    }
}
} // namespace gpaco
