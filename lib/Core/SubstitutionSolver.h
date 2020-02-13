#ifndef KLEE_SUBSTITUTIONSOLVER_H
#define KLEE_SUBSTITUTIONSOLVER_H

#include "klee/Expr.h"
#include "klee/Solver.h"

#include <vector>

namespace klee {
  class ExecutionState;
  class Solver;  

  class SubstitutionSolver {
  public:

    Solver *solver;
    bool simplifyExprs;

  public:

    SubstitutionSolver(Solver *_solver, bool _simplifyExprs = true) 
      : solver(_solver), simplifyExprs(_simplifyExprs) {

    }

    ~SubstitutionSolver() {
      delete solver;
    }

    void setTimeout(time::Span t) {
      solver->setCoreSolverTimeout(t);
    }
    
    char *getConstraintLog(const Query& query) {
      return solver->getConstraintLog(query);
    }

    bool evaluate(const ExecutionState&, ref<Expr>, Solver::Validity &result);

    bool mustBeTrue(const ExecutionState&, ref<Expr>, bool &result);

    bool mustBeFalse(const ExecutionState&, ref<Expr>, bool &result);

    bool mayBeTrue(const ExecutionState&, ref<Expr>, bool &result);

    bool mayBeFalse(const ExecutionState&, ref<Expr>, bool &result);

    bool getValue(const ExecutionState &, ref<Expr> expr, 
                  ref<ConstantExpr> &result);

    bool getInitialValues(const ExecutionState&, 
                          const std::vector<const Array*> &objects,
                          std::vector< std::vector<unsigned char> > &result);

    std::pair<ref<Expr>, ref<Expr>> getRange(const ExecutionState&, ref<Expr> query);
  };

}

#endif
