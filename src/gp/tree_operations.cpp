#include "gp/tree_operations.h"
#include "core/random.h"
#include <stdexcept>
namespace gpaco {
void TreeOperations::crossoverTree(GOftn *treeToAlter, GOftn *donatingTree, int id_rnd) {
    if ((treeToAlter->m_numChildren > 0) && (donatingTree->m_numChildren > 0)) {
        int childToAlter = randomStream(id_rnd).uniform.nextNonStd<int>(
            0, treeToAlter->m_numChildren); // randomly select side of treeToAlter and donatingTree
        int childToDonate =
            randomStream(id_rnd).uniform.nextNonStd<int>(0, donatingTree->m_numChildren);
        deleteTree(treeToAlter->m_children[childToAlter]); // delete side of treeToAlter
        treeToAlter->m_children[childToAlter] =
            (donatingTree->m_children[childToDonate])->clone(); // clone side of donatingTree
    } else
        return;
}

void TreeOperations::mutateTree(GOftn **inTree, int id_rnd, int depth) {
    Real randNum0t1 = randomStream(id_rnd).uniform.next();
    ;
    if (randNum0t1 < m_mutation_p) {
        // create new node
        int gen_type = randomStream(id_rnd).uniform.nextNonStd<int>(0, 2);
        GOftn *newNode = createRandomNode(depth + 1, gen_type,
                                          id_rnd); // create a random node to replace current node
        int childrenToMove = std::min(newNode->m_numChildren, (*inTree)->m_numChildren);
        for (int i = 0; i < childrenToMove; i++) {
            newNode->m_children[i] = (*inTree)->m_children[i]->clone();
        }
        int numNewChildren = newNode->m_numChildren - childrenToMove;
        for (int i = childrenToMove; i < (childrenToMove + numNewChildren); i++) {
            newNode->m_children[i] = halfAndHalf(id_rnd, depth + 1);
        }
        deleteTree(*inTree); // free memory held by old node
        *inTree = newNode;   // replace old ptr with ptr to new
    }
    for (int i = 0; i < (*inTree)->m_numChildren; i++) {
        mutateTree(&(*inTree)->m_children[i], id_rnd, depth + 1);
    }
}

GOftn *TreeOperations::halfAndHalf(int id_rnd, int depth) {
    // int gen_type = randomStream(id_rnd).uniform.nextNonStd<int>(0, 2);
    int gen_type = 0;
    return createRandomTree(depth, gen_type, id_rnd);
}

GOftn *TreeOperations::createRandomTree(int depth, int type, int id_rnd) {
    GOftn *retFtn = createRandomNode(depth, type, id_rnd);

    for (int i = 0; i < retFtn->m_numChildren; i++) {
        retFtn->m_children[i] = createRandomTree(depth + 1, type, id_rnd);
    }
    return retFtn;
}

GOftn *TreeOperations::createRandomNode(int depth, int type, int id_rnd) {
    GOftn *retFtn = nullptr;
    if (depth == 0) {
        int randn = randomStream(id_rnd).uniform.nextNonStd<int>(0, m_operators.size());
        int opt_type = -1 * ((int)m_operators[randn] + 1);
        retFtn = buildNode(opt_type);
    } else if (depth < m_max_depth) {
        int randn = 0, node_type = 0;
        Real val = randomStream(id_rnd).uniform.next();

        if (type == 0)
            randn = randomStream(id_rnd).uniform.nextNonStd<int>(0, m_operators.size() +
                                                                        m_terminals.size() + 1);
        else
            randn = randomStream(id_rnd).uniform.nextNonStd<int>(0, m_operators.size());

        if (randn < m_operators.size())
            node_type = -1 * ((int)m_operators[randn] + 1);
        else if (randn < m_operators.size() + m_terminals.size())
            node_type = m_terminals[randn - m_operators.size()] + 1;
        else
            node_type = 0;
        retFtn = buildNode(node_type, val);
    } else {
        int randn = 0, ter_type = 0;
        Real val = randomStream(id_rnd).uniform.next();

        randn = randomStream(id_rnd).uniform.nextNonStd<int>(0, m_terminals.size() + 1);
        if (randn < m_terminals.size())
            ter_type = m_terminals[randn] + 1;
        else
            ter_type = 0;
        retFtn = buildNode(ter_type, val);
    }
    return retFtn;
}

void TreeOperations::deleteTree(GOftn *inTree) {
    if (!inTree)
        return;
    for (int i = 0; i < inTree->m_numChildren; i++) { // free any m_children first
        deleteTree(inTree->m_children[i]);
    }
    delete inTree;
}

GOftn *TreeOperations::buildNode(int type, Real val) {
    if (type == 0)
        return new ConstNode(val);
    else if (type > 0) {
        InputNode *retFtn = new InputNode(type - 1);
        return retFtn;
    } else if (type == -1)
        return new Add();
    else if (type == -2)
        return new Subtract();
    else if (type == -3)
        return new Multiply();
    else if (type == -4)
        return new Divide();
    else if (type == -5)
        return new Min();
    else if (type == -6)
        return new Max();
    else if (type == -7)
        return new Sin();
    else if (type == -8)
        return new If();
    else if (type == -9)
        return new Sigmoid();
    else
        throw std::runtime_error("Invalid GP node type");
}
} // namespace gpaco
