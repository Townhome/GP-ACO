#include "gp/model.h"
namespace gpaco {
Json treeJson(GOftn *tree) {
    require(tree != nullptr, "Null GP-ACO tree");
    Json j = {{"type", tree->m_type}, {"children", Json::array()}};
    if (tree->m_type == 0)
        j["constant"] = static_cast<ConstNode *>(tree)->getConst();
    for (int k = 0; k < tree->m_numChildren; ++k)
        j["children"].push_back(treeJson(tree->m_children[k]));
    return j;
}
GOftn *readTree(const Json &j, int depth) {
    require(depth <= 4096, "GP-ACO model exceeds supported recursion depth");
    int type = j.at("type").get<int>();
    require(type >= -9 && type <= 16, "Invalid GP-ACO tree node");
    double value = j.value("constant", 0.);
    require(std::isfinite(value), "Invalid constant");
    TreeOperations operations;
    GOftn *tree = operations.buildNode(type, value);
    try {
        require(j.at("children").size() == size_t(tree->m_numChildren), "Wrong GP-ACO arity");
        for (int k = 0; k < tree->m_numChildren; ++k)
            tree->m_children[k] = readTree(j.at("children").at(k), depth + 1);
    } catch (...) {
        TreeOperations::deleteTree(tree);
        throw;
    }
    return tree;
}
Json ruleJson(const Rule &rule) {
    return {{"tree", treeJson(rule->tree)},
            {"fitness", rule->fitness},
            {"task", rule->task_id},
            {"fitness_vector", rule->fitness_vector}};
}
Rule readRule(const Json &j) {
    Rule r = std::make_shared<GPRule>();
    r->tree = readTree(j.at("tree"));
    r->fitness = j.at("fitness").get<double>();
    r->task_id = j.value("task", -1);
    r->fitness_vector = j.value("fitness_vector", std::vector<Real>{});
    require(std::isfinite(r->fitness), "Invalid GP-ACO fitness");
    return r;
}
} // namespace gpaco
