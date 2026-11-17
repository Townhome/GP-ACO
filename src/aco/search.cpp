#include "aco/search.h"
#include "problem/local_search.h"
#include <cstring>
namespace gpaco {
namespace {
double unit(std::mt19937_64 &rng) {
    return double(rng() >> 11) * 0x1.0p-53;
}
inline void evaporate(Matrix &tau, Real rho, Real tau_min, Real tau_max) {
    for (auto &truck : tau)
        for (auto &position : truck)
            for (auto &value : position)
                value = std::clamp((1 - rho) * value, tau_min, tau_max);
}
inline void deposit(Matrix &tau, const Routes &routes, Real delta, Real tau_min, Real tau_max) {
    if (tau.size() != routes.size())
        throw std::runtime_error("ACO pheromone/route dimensions disagree");
    for (size_t truck = 0; truck < routes.size(); ++truck)
        for (size_t pos = 0; pos < routes[truck].size(); ++pos) {
            if (tau[truck].empty())
                throw std::runtime_error("Empty ACO pheromone matrix");
            while (pos >= tau[truck].size())
                tau[truck].emplace_back(tau[truck].front().size(), tau_max);
            if (routes[truck][pos] < 0 ||
                static_cast<size_t>(routes[truck][pos]) >= tau[truck][pos].size())
                throw std::runtime_error("Invalid ACO route node");
            auto &value = tau[truck][pos][routes[truck][pos]];
            value = std::clamp(value + delta, tau_min, tau_max);
        }
}
inline std::pair<Real, Real> bounds(Real objective, Real rho, size_t problem_size) {
    if (!std::isfinite(objective) || objective < 0 || !problem_size || !std::isfinite(rho) ||
        rho <= 0 || rho >= 1)
        throw std::runtime_error("Invalid ACO objective/problem size");
    const Real maximum = Real(1) / (rho * std::max(objective, Real(1e-12)));
    if (!std::isfinite(maximum))
        throw std::runtime_error("ACO pheromone bound overflow");
    return {std::max(Real(1e-16), maximum / (2 * problem_size)), std::max(Real(1e-12), maximum)};
}
inline double finiteScore(double score) {
    if (std::isfinite(score))
        return score;
    return std::isnan(score) ? 0 : std::copysign(std::numeric_limits<double>::max(), score);
}
inline std::vector<double> probabilities(const std::vector<double> &scores,
                                         const std::vector<double> &pheromones, double alpha,
                                         double beta, double temperature) {
    if (scores.empty() || scores.size() != pheromones.size() || !(temperature > 0))
        throw std::runtime_error("GP-ACO transition: invalid transition inputs");
    double maximum_score = -std::numeric_limits<double>::infinity(), maximum_tau = 1e-300;
    for (double score : scores)
        maximum_score = std::max(maximum_score, finiteScore(score));
    for (double tau : pheromones)
        maximum_tau = std::max(maximum_tau, tau);
    const double log_tau_max = std::log(maximum_tau);
    double maximum = -std::numeric_limits<double>::infinity();
    std::vector<double> weights(scores.size());
    for (size_t j = 0; j < scores.size(); ++j) {
        // A wide subtraction avoids overflow for opposite extreme GP outputs.
        // Negative infinity is harmless here: exp(-infinity) is zero.
        long double difference = static_cast<long double>(finiteScore(scores[j])) -
                                 static_cast<long double>(maximum_score);
        double heuristic = beta == 0 ? 0 : static_cast<double>(difference / temperature * beta);
        double trail =
            alpha == 0 ? 0 : alpha * (std::log(std::max(pheromones[j], 1e-300)) - log_tau_max);
        weights[j] = trail + heuristic;
        maximum = std::max(maximum, weights[j]);
    }
    if (!std::isfinite(maximum))
        throw std::runtime_error("GP-ACO transition: non-finite transition normalization");
    double sum = 0;
    for (double &weight : weights) {
        weight = std::exp(weight - maximum);
        sum += weight;
    }
    for (double &weight : weights)
        weight /= sum;
    return weights;
}
} // namespace
Dataset::Dataset(const fs::path &root, int mine, int scale) {
    require(mine >= 1 && mine <= 3 && scale >= 1 && scale <= 4, "Invalid scenario");
    domain = mine * 10 + scale;
    const int shovels[] = {3, 5, 7, 9};
    auto path =
        root / "graph" / ("C" + std::to_string(mine) + "_D" + std::to_string(scale) + "_test.txt");
    require(fs::is_regular_file(path), "Missing data: " + path.string());
    ParamMap p;
    p["ShovelSize"] = shovels[scale - 1];
    p["TruckSize"] = shovels[scale - 1] * (mine == 3 ? 3 : 2);
    p["UnloadSize"] = mine + 1;
    p["WindowSize"] = Real(600);
    p["DataFile"] = path.string();
    truth = std::make_unique<TruckBase>(p);
    forecast = std::make_unique<TruckBase>(*truth);
    require(truth->getNumSample() == 200, "Expected 200 test instances");
}
void Dataset::reveal(int environment) {
    for (int i = 0; i < truth->getNumSample(); ++i) {
        require(environment >= 0 && environment < 5 && i >= 0 && i < truth->getNumSample(),
                "Invalid reveal index");
        const double cutoff = truth->getShiftTime() * environment / 5.;
        for (size_t j = 0; j < forecast->m_shovel_down_time[i].size(); ++j)
            forecast->m_shovel_down_time[i][j] = truth->m_shovel_down_time[i][j] <= cutoff
                                                     ? truth->m_shovel_down_time[i][j]
                                                     : truth->m_shift_time;
        for (size_t j = 0; j < forecast->m_truck_down_time[i].size(); ++j)
            forecast->m_truck_down_time[i][j] = truth->m_truck_down_time[i][j] <= cutoff
                                                    ? truth->m_truck_down_time[i][j]
                                                    : truth->m_shift_time;
        bool known = truth->m_event_time[i] <= cutoff;
        forecast->m_event_time[i] = known ? truth->m_event_time[i] : truth->m_shift_time;
        forecast->m_road_predict_d[i] =
            known ? truth->m_road_predict_d[i] : truth->m_road_predict[i];
        for (auto &shovel : forecast->m_load_time_normal[i])
            for (auto &n : shovel)
                n.setStd(0);
        for (auto &n : forecast->m_unload_time_normal[i])
            n.setStd(0);
    }
}
Search::Search(const SearchOptions &options, Dataset &dataset, const Models &model, int instance,
               uint64_t root_seed)
    : o(options), data(dataset), pro(data.forecast.get()), models(model), sample(instance),
      seed(root_seed) {
    rnd = addRandom(.4321);
}
Search::~Search() {
    deleteRandom(rnd);
}
Ptr Search::makePrefix(const Ptr &old, double cutoff) {
    auto s = std::make_shared<Sol>();
    s->variable().m_sample_id = sample;
    data.truth->resetSolutionWithSeed(s.get(), key(seed, data.domain, sample, 0, 0, 0));
    if (old && cutoff > 0) {
        auto &routes = old->variable().m_truck_routes;
        std::mt19937_64 fallback(key(seed, data.domain, sample, 0, 0, 971));
        for (;;) {
            int t = data.truth->choseTruck(sample, s.get());
            if (t < 0)
                break;
            const auto &times = s->variable().m_arrive_time[t];
            if ((times.empty() ? 0 : times.back()) >= cutoff)
                break;
            auto mask = data.truth->updateMask(sample, t, s.get());
            std::vector<int> legal;
            for (size_t n = 0; n < mask.size(); ++n)
                if (mask[n])
                    legal.push_back(int(n));
            if (legal.empty())
                break;
            size_t pos = s->variable().m_truck_routes[t].size();
            int node = pos < routes[t].size() ? routes[t][pos] : -1;
            if (node < 0 || node >= int(mask.size()) || !mask[node])
                node = legal[size_t(fallback() % legal.size())];
            data.truth->updateState(sample, t, node, s.get());
        }
    }
    {
        for (auto &shovel : s->variable().m_load_time_normal)
            for (auto &n : shovel)
                n.setStd(0);
        for (auto &n : s->variable().m_unload_time_normal)
            n.setStd(0);
    }
    return s;
}
void Search::checkPrefix(const Sol &candidate) const {
    const auto &p = prefix->variable().m_truck_routes;
    const auto &c = candidate.variable().m_truck_routes;
    require(c.size() == p.size(), "Route dimension mismatch");
    for (size_t t = 0; t < p.size(); ++t)
        require(c[t].size() >= p[t].size() && std::equal(p[t].begin(), p[t].end(), c[t].begin()),
                "Frozen executed prefix changed");
}
Ptr Search::repair(const Routes *source) {
    auto s = std::make_shared<Sol>(*prefix);
    for (;;) {
        int t = pro->choseTruck(sample, s.get());
        if (t < 0)
            return s;
        auto mask = pro->updateMask(sample, t, s.get());
        std::vector<int> legal;
        for (size_t n = 0; n < mask.size(); ++n)
            if (mask[n])
                legal.push_back(int(n));
        if (legal.empty())
            return s;
        size_t pos = s->variable().m_truck_routes[t].size();
        int node = -1;
        if (source && pos < (*source)[t].size()) {
            int n = (*source)[t][pos];
            if (n >= 0 && n < int(mask.size()) && mask[n])
                node = n;
        }
        if (node < 0)
            node = legal[size_t(rng() % legal.size())];
        pro->updateState(sample, t, node, s.get());
    }
}
void Search::publish(const Ptr &candidate, const char *type) {
    require(active(), "Objective budget exceeded");
    if (std::strcmp(type, "ant") != 0)
        candidate->setFitness(pro->evaluate(sample, candidate.get()));
    double value = candidate->fitness();
    require(std::isfinite(value) && value >= 0, "Invalid objective");
    checkPrefix(*candidate);
    ++result.evaluations;
    ++result.prefix_checks;
    if (std::strcmp(type, "repair") == 0)
        ++result.repair_evaluations;
    if (std::strcmp(type, "local") == 0)
        ++result.local_evaluations;
    result.evaluation_hash =
        mix(result.evaluation_hash ^ routeHash(*candidate) ^ hashText(Json(value).dump()));
    if (!incumbent || value < incumbent->fitness())
        incumbent = candidate;
    result.objective = incumbent->fitness();
    result.route_hash = routeHash(*incumbent);
}
Ptr Search::construct(size_t ant) const {
    auto candidate = std::make_shared<Sol>(*prefix);
    std::mt19937_64 draw(key(seed, data.domain, sample, env, generation, int(ant)));
    for (;;) {
        int truck = pro->choseTruck(sample, candidate.get());
        if (truck < 0)
            break;
        auto mask = pro->updateMask(sample, truck, candidate.get());
        std::vector<int> legal;
        for (size_t n = 0; n < mask.size(); ++n)
            if (mask[n])
                legal.push_back(int(n));
        if (legal.empty())
            break;
        int selected = legal.front();
        if (legal.size() > 1) {
            auto features =
                pro->getHeuristicValue(sample, truck, mask, models.range, candidate.get(), 16);
            auto rule = models.rules[ant % models.rules.size()];
            size_t pos = candidate->variable().m_truck_routes[truck].size();
            std::vector<double> scores, pheromones;
            for (int node : legal) {
                for (auto &value : features[node])
                    if (!std::isfinite(value))
                        value = 0;
                double score = rule->tree->eval(features[node]);
                if (!std::isfinite(score))
                    score = std::isnan(score)
                                ? 0
                                : std::copysign(std::numeric_limits<double>::max(), score);
                scores.push_back(score);
                pheromones.push_back(pos < tau[truck].size() ? tau[truck][pos][node] : maximum);
            }
            auto p =
                probabilities(scores, pheromones, o.alpha, models.gpaco_beta, models.temperature);
            double target = unit(draw), cumulative = 0;
            selected = legal.back();
            for (size_t j = 0; j < legal.size(); ++j) {
                cumulative += p[j];
                if (target < cumulative) {
                    selected = legal[j];
                    break;
                }
            }
        }
        pro->updateState(sample, truck, selected, candidate.get());
    }
    candidate->setFitness(pro->evaluate(sample, candidate.get()));
    require(std::isfinite(candidate->fitness()) && candidate->fitness() >= 0,
            "Invalid constructed objective");
    return candidate;
}
void Search::update() {
    double rho = models.gpaco_rho;
    if (generation_best->fitness() < search_best) {
        search_best = generation_best->fitness();
        stagnant = 0;
        std::tie(minimum, maximum) = bounds(incumbent->fitness(), Real(rho), problem_size);
    } else
        ++stagnant;
    evaporate(tau, Real(rho), Real(minimum), Real(maximum));
    deposit(tau, generation_best->variable().m_truck_routes,
            Real(1 / std::max(double(generation_best->fitness()), 1e-12)), Real(minimum),
            Real(maximum));
    if (stagnant >= std::max(1, o.budget / o.ants / 2)) {
        for (auto &truck : tau)
            for (auto &pos : truck)
                std::fill(pos.begin(), pos.end(), Real(maximum));
        search_best = std::numeric_limits<double>::infinity();
        stagnant = 0;
    }
    ++generation;
    result.generations = generation;
    generation_best.reset();
    local.reset();
    local_left = 0;
}
void Search::localTrial() {
    auto modified = std::make_shared<Sol>(*local);
    double start = env / 5.;
    double a = unit(rng), b = unit(rng);
    double left = start + (1 - start) * std::min(a, b),
           right = start + (1 - start) * std::max(a, b);
    int op = int(rng() % 5), pairs = 0, long_routes = 0;
    for (const auto &route : modified->variable().m_truck_routes) {
        pairs += route.size() >= 2;
        long_routes += route.size() >= 4;
    }
    if (op == 0 && pairs >= 2)
        swapPairsBetweenRoutes(*pro, modified->variable(), Real(left), Real(right), rnd);
    if (op == 1 && long_routes)
        reversePairsInRoute(*pro, modified->variable(), Real(left), Real(right), rnd);
    if (op == 2 && pairs)
        randomResetPairs(*pro, modified->variable(), Real(left), Real(right), rnd);
    if (op == 3 && pairs)
        relocateSegmentInRoute(*pro, modified->variable(), Real(left), Real(right), rnd);
    if (op == 4)
        randomResetRange(*pro, modified->variable(), Real(left), Real(right), rnd);
    auto candidate = repair(&modified->variable().m_truck_routes);
    publish(candidate, "local");
    if (candidate->fitness() < local->fitness())
        local = candidate;
    if (--local_left == 0 || !active()) {
        generation_best = local;
        update();
    }
}
void Search::run(int environment) {
    require(environment >= 0 && environment < 5, "Invalid environment");
    env = environment;
    auto old = incumbent, old_prefix = prefix;
    prefix = makePrefix(old, pro->getShiftTime() * env / 5.);
    if (old_prefix) {
        auto &before = old_prefix->variable().m_truck_routes;
        auto &after = prefix->variable().m_truck_routes;
        for (size_t t = 0; t < before.size(); ++t)
            require(after[t].size() >= before[t].size() &&
                        std::equal(before[t].begin(), before[t].end(), after[t].begin()),
                    "Executed history changed at transition");
    }
    result = Result{};
    result.prefix_hash = routeHash(*prefix);
    incumbent.reset();
    generation = local_left = stagnant = 0;
    generation_best.reset();
    local.reset();
    search_best = std::numeric_limits<double>::infinity();
    problem_size = 1;
    rng.seed(key(seed, data.domain, sample, env, 0, 913));
    resetUniform(rnd, key(seed, data.domain, sample, env, 0, 911));
    if (env == 0) {
        minimum = 1e-6;
        maximum = 1;
        tau.assign(pro->getNumTruck(), std::vector<std::vector<Real>>(
                                           std::max(1, pro->getMaxRouteLength(sample)),
                                           std::vector<Real>(pro->getNumNodes(), Real(maximum))));
    } else {
        publish(repair(old ? &old->variable().m_truck_routes : nullptr), "repair");
        std::tie(minimum, maximum) =
            bounds(incumbent->fitness(), Real(models.gpaco_rho),
                   std::max(size_t(1), actionCount(incumbent->variable().m_truck_routes) *
                                           pro->getNumNodes()));
        for (auto &truck : tau)
            for (auto &pos : truck)
                for (auto &value : pos)
                    value = Real(std::clamp(double(value), minimum, maximum));
    }
    while (active()) {
        if (local_left) {
            localTrial();
            continue;
        }
        size_t count = std::min(size_t(o.ants), size_t(o.budget - result.evaluations));
        generation_best.reset();
        problem_size = 1;
        for (size_t ant = 0; ant < count; ++ant) {
            auto candidate = construct(ant);
            publish(candidate, "ant");
            if (!generation_best || candidate->fitness() < generation_best->fitness())
                generation_best = candidate;
            problem_size =
                std::max(problem_size,
                         actionCount(candidate->variable().m_truck_routes) * pro->getNumNodes());
        }
        if (o.local_trials > 0 && generation_best->fitness() < search_best && active()) {
            local = generation_best;
            local_left = std::min(o.local_trials, int(o.budget - result.evaluations));
        } else
            update();
    }
    generation_best.reset();
    local.reset();
    require(result.evaluations == uint64_t(o.budget) && result.prefix_checks == result.evaluations,
            "Search budget/prefix audit failed");
}
Models loadModel(const fs::path &path, int library) {
    auto json = readJson(path);
    require(json.at("schema") == "GPACO_JOINT_ARCHIVE_1" && json.at("frozen") == true,
            "Expected a frozen joint GP-ACO archive");
    std::vector<Json> ranked;
    for (const auto &row : json.at("archive"))
        ranked.push_back(row);
    std::sort(ranked.begin(), ranked.end(), [](const Json &a, const Json &b) {
        double x = a.at("fitness"), y = b.at("fitness");
        return x != y ? x < y : a.at("cell").get<int>() < b.at("cell").get<int>();
    });
    require(ranked.size() >= size_t(library), "Insufficient archive cells for the rule library");
    Models model;
    model.range = json.at("normalization").get<Range>();
    for (int i = 0; i < library; ++i)
        model.rules.push_back(readRule(ranked[i]));
    for (const char *name :
         {"max_quantity", "min_grade", "max_grade", "max_capacity", "max_time", "max_shift"})
        require(model.range.count(name) && std::isfinite(model.range.at(name)),
                "Invalid training normalization");
    require(model.range.at("max_grade") > model.range.at("min_grade"), "Zero grade range");
    return model;
}
} // namespace gpaco
