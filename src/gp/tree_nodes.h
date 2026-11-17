#ifndef TREE_BASIC_ELEMENTS_H
#define TREE_BASIC_ELEMENTS_H

#include "core/types.h"

#include <string>
#include <memory>
#include <vector>

namespace gpaco {

//---------------------------------------------------------------------------
// 1. Function Operators
//---------------------------------------------------------------------------

// All function types

enum class FunType { Add, Subtract, Multiply, Divide, Min, Max, Sin, If, Sigmoid, FuncNum };

// Master abstract class: GOftn: Genetic Optimize Function

class GOftn {
  public:
    std::string m_label;
    int m_numChildren = 0;
    GOftn *m_children[4];
    Real m_perfScore;
    int m_type;
    GOftn() : m_children{nullptr, nullptr, nullptr, nullptr} {}
    virtual ~GOftn() {}
    virtual Real eval(const std::vector<Real> &inVal) = 0; // setting the 0 makes it a PURE
    virtual GOftn *clone() = 0;                            // make a deep copy of the current tree
    std::string &getLabel() { return m_label; };
};

// Class for storing constant values

class ConstNode : public GOftn {
    Real m_constVal;

  public:
    ConstNode(Real preSetVal);
    virtual Real eval(const std::vector<Real> &inVal);
    ConstNode *clone();
    Real getConst() { return m_constVal; }
};

// Class for using inputs

class InputNode : public GOftn {
    int m_inputIndex;

  public:
    InputNode(int inputIndex);
    virtual Real eval(const std::vector<Real> &inVal);
    InputNode *clone();
    void setValues(int inIndex);
};

// Addition

class Add : public GOftn {
  public:
    Add();
    virtual Real eval(const std::vector<Real> &inVal);
    Add *clone();
};

// Subtraction

class Subtract : public GOftn {
  public:
    Subtract();
    virtual Real eval(const std::vector<Real> &inVal);
    Subtract *clone();
};

// Multiplication

class Multiply : public GOftn {
  public:
    Multiply();
    virtual Real eval(const std::vector<Real> &inVal);
    Multiply *clone();
};

// Division

class Divide : public GOftn {
  public:
    Divide();
    virtual Real eval(const std::vector<Real> &inVal);
    Divide *clone();
};

// Min

class Min : public GOftn {
  public:
    Min();
    virtual Real eval(const std::vector<Real> &inVal);
    Min *clone();
};

// Max

class Max : public GOftn {
  public:
    Max();
    virtual Real eval(const std::vector<Real> &inVal);
    Max *clone();
};

// Sin

class Sin : public GOftn {
  public:
    Sin();
    virtual Real eval(const std::vector<Real> &inVal);
    Sin *clone();
};

// If

class If : public GOftn {
  public:
    If();
    virtual Real eval(const std::vector<Real> &inVal);
    If *clone();
};

// Sigmoid

class Sigmoid : public GOftn {
  public:
    Sigmoid();
    virtual Real eval(const std::vector<Real> &inVal);
    Sigmoid *clone();
};

//---------------------------------------------------------------------------
// 2. Truck Terminal Sets
//---------------------------------------------------------------------------

enum class Terminals {

    /* Public truck terminals */

    CTC,       // current truck capacity, t^c_cur
    GradeTG,   // grade target, G^b
    GradeALP,  // average grade across all loading point, G^l
    RGradeALP, // rest average grade across all loading point, g^r
    PALP,      // proportion of remaining ore across all loading point, p^l

    /* Point selection terminals */

    GradeCLP,  // average grade of current loading point (one by unloading point), G^l_i
    RGradeCLP, // rest average grade of current loading point (one by unloading point), g^r_i
    PCLP, // proportion of remaining ore at current loading point (one by unloading point), p^l_i

    CTGUP,  // current truck grade of upcoming point, t^g_cur
    OTGUP,  // other truck grade of upcoming point, t^g_cur-hat
    ALGALP, // average load grade across all loading point, g^l
    WGUP,   // window grade of the unloading point (one by loading point), u^g_j
    AWGAUP, // average window grade across all unloading point, u^g
    ErrUP,  // ore flow error of unloading point (one by loading point), u^e_j
    ErrAUP, // average ore flow error across all unloading point, u^e
    ACOH,   // aco heuristic value of upcoming point, h^a_m

    TerminalNum
};

} // namespace gpaco

#endif // !TREE_BASIC_ELEMENTS_H
