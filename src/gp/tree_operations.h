#ifndef TREE_OPERATIONS_H
#define TREE_OPERATIONS_H

#include "gp/tree_nodes.h"

namespace gpaco {

class TreeOperations {
  protected:
    std::vector<FunType> m_operators; // operator set
    std::vector<int> m_terminals;     // terminal set
    int m_max_depth;                  // max depth of the tree
    Real m_mutation_p;                // mutation probability

  public:
    GOftn *halfAndHalf(int id_rnd, int depth = 0);
    GOftn *createRandomNode(int depth, int type, int id_rnd);
    GOftn *createRandomTree(int depth, int type, int id_rnd);
    GOftn *buildNode(int type, Real val = 0);
    void mutateTree(GOftn **inTree, int id_rnd, int depth = 0);
    void crossoverTree(GOftn *treeToAlter, GOftn *donatingTree, int id_rnd);
    static void deleteTree(GOftn *inTree);
    void setOperators(const std::vector<FunType> &ops) { m_operators = ops; }
    void setTerminals(const std::vector<int> &tms) { m_terminals = tms; }
    void setMaxDepth(int depth) { m_max_depth = depth; }
    void setMutationP(Real p) { m_mutation_p = p; }
};

} // namespace gpaco

#endif // !TREE_OPERATIONS_H
