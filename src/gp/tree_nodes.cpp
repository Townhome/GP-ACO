#include "gp/tree_nodes.h"

#include <iostream>
#include <algorithm>
#include <cmath>

namespace gpaco {

//
//		ConstNode
//

ConstNode::ConstNode(Real preSetVal) : GOftn() {
    m_numChildren = 0;
    m_constVal = preSetVal;
    m_label = "Const: " + std::to_string(m_constVal);
    m_type = 0;
}
Real ConstNode::eval(const std::vector<Real> &inVal) {
    return m_constVal;
}
ConstNode *ConstNode::clone() {
    return new ConstNode(m_constVal);
}
//
//		InputNode
//
InputNode::InputNode(int inputIndex) : GOftn() {
    m_numChildren = 0;
    setValues(inputIndex);
}
Real InputNode::eval(const std::vector<Real> &inVal) {
    return inVal[m_inputIndex];
}
void InputNode::setValues(int inIndex) {
    m_label = "InputVal: " + std::to_string(inIndex);
    m_inputIndex = inIndex;
    m_type = inIndex + 1;
}
InputNode *InputNode::clone() {
    InputNode *retNode = new InputNode(m_inputIndex);
    return retNode;
}
//
//		Add
//
Add::Add() : GOftn() {
    m_numChildren = 2;
    m_label = "Add";
    m_type = -1;
}
Real Add::eval(const std::vector<Real> &inVal) {
    if (m_children[0] && m_children[1]) {
        return m_children[0]->eval(inVal) + m_children[1]->eval(inVal);
    } else {
        std::cerr << "left and right not defined in add" << std::endl;
        return -1.0;
    }
}
Add *Add::clone() {
    Add *retNode = new Add();
    for (int i = 0; i < m_numChildren; i++) {
        retNode->m_children[i] = m_children[i]->clone();
    }
    return retNode;
}
//
//		Subtract
//
Subtract::Subtract() : GOftn() {
    m_numChildren = 2;
    m_label = "Subtract";
    m_type = -2;
}
Real Subtract::eval(const std::vector<Real> &inVal) {
    if (m_children[0] && m_children[1]) {
        return m_children[0]->eval(inVal) - m_children[1]->eval(inVal);
    } else {
        std::cerr << "left and right not defined in subtract" << std::endl;
        return -1.0;
    }
}
Subtract *Subtract::clone() {
    Subtract *retNode = new Subtract();
    for (int i = 0; i < m_numChildren; i++) {
        retNode->m_children[i] = m_children[i]->clone();
    }
    return retNode;
}
//
//		Multiply
//
Multiply::Multiply() : GOftn() {
    m_numChildren = 2;
    m_label = "Multiply";
    m_type = -3;
}
Real Multiply::eval(const std::vector<Real> &inVal) {
    if (m_children[0] && m_children[1]) {
        return m_children[0]->eval(inVal) * m_children[1]->eval(inVal);
    } else {
        std::cerr << "left and right not defined in multiply" << std::endl;
        return -1.0;
    }
}
Multiply *Multiply::clone() {
    Multiply *retNode = new Multiply();
    for (int i = 0; i < m_numChildren; i++) {
        retNode->m_children[i] = m_children[i]->clone();
    }
    return retNode;
}
//
//		Divide
//
Divide::Divide() : GOftn() {
    m_numChildren = 2;
    m_label = "Divide";
    m_type = -4;
}
Real Divide::eval(const std::vector<Real> &inVal) {
    if (m_children[0] && m_children[1]) {
        if (m_children[1]->eval(inVal) == 0)
            return 1.0;
        else
            return m_children[0]->eval(inVal) / m_children[1]->eval(inVal);
    } else {
        std::cerr << "left and right not defined in divide" << std::endl;
        return -1.0;
    }
}
Divide *Divide::clone() {
    Divide *retNode = new Divide;
    for (int i = 0; i < m_numChildren; i++) {
        retNode->m_children[i] = m_children[i]->clone();
    }
    return retNode;
}
//
//		Min
//
Min::Min() : GOftn() {
    m_numChildren = 2;
    m_label = "Min";
    m_type = -5;
}
Real Min::eval(const std::vector<Real> &inVal) {
    if (m_children[0] && m_children[1]) {
        Real left = m_children[0]->eval(inVal);
        Real right = m_children[1]->eval(inVal);
        return left < right ? left : right;
    } else {
        std::cerr << "left and right not defined in min" << std::endl;
        return -1.0;
    }
}
Min *Min::clone() {
    Min *retNode = new Min;
    for (int i = 0; i < m_numChildren; i++) {
        retNode->m_children[i] = m_children[i]->clone();
    }
    return retNode;
}
//
//		Max
//
Max::Max() : GOftn() {
    m_numChildren = 2;
    m_label = "Max";
    m_type = -6;
}
Real Max::eval(const std::vector<Real> &inVal) {
    if (m_children[0] && m_children[1]) {
        Real left = m_children[0]->eval(inVal);
        Real right = m_children[1]->eval(inVal);
        return left > right ? left : right;
    } else {
        std::cerr << "left and right not defined in max" << std::endl;
        return -1.0;
    }
}
Max *Max::clone() {
    Max *retNode = new Max;
    for (int i = 0; i < m_numChildren; i++) {
        retNode->m_children[i] = m_children[i]->clone();
    }
    return retNode;
}
//
//		Sin
//
Sin::Sin() : GOftn() {
    m_numChildren = 1;
    m_label = "Sin";
    m_type = -7;
}
Real Sin::eval(const std::vector<Real> &inVal) {
    if (m_children[0]) {
        Real son = m_children[0]->eval(inVal);
        return std::sin(son);
    } else {
        std::cerr << "son not defined in sin" << std::endl;
        return -1.0;
    }
}
Sin *Sin::clone() {
    Sin *retNode = new Sin;
    for (int i = 0; i < m_numChildren; i++) {
        retNode->m_children[i] = m_children[i]->clone();
    }
    return retNode;
}
//
//		If
//
If::If() : GOftn() {
    m_numChildren = 3;
    m_label = "If";
    m_type = -8;
}
Real If::eval(const std::vector<Real> &inVal) {
    if (m_children[0] && m_children[1] && m_children[2]) {
        Real judge = m_children[0]->eval(inVal);
        Real left = m_children[1]->eval(inVal);
        Real right = m_children[2]->eval(inVal);
        return judge > 0 ? left : right;
    } else {
        std::cerr << "son not defined in if" << std::endl;
        return -1.0;
    }
}
If *If::clone() {
    If *retNode = new If;
    for (int i = 0; i < m_numChildren; i++) {
        retNode->m_children[i] = m_children[i]->clone();
    }
    return retNode;
}
//
//		Sigmoid
//
Sigmoid::Sigmoid() : GOftn() {
    m_numChildren = 1;
    m_label = "Sigmoid";
    m_type = -9;
}
Real Sigmoid::eval(const std::vector<Real> &inVal) {
    if (m_children[0]) {
        Real son = m_children[0]->eval(inVal);
        return 1 / (1 + std::exp(-1 * son));
    } else {
        std::cerr << "son not defined in sigmoid" << std::endl;
        return -1.0;
    }
}
Sigmoid *Sigmoid::clone() {
    Sigmoid *retNode = new Sigmoid;
    for (int i = 0; i < m_numChildren; i++) {
        retNode->m_children[i] = m_children[i]->clone();
    }
    return retNode;
}
} // namespace gpaco
