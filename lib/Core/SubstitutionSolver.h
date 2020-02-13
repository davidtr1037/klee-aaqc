#ifndef KLEE_SUBSTITUTIONSOLVER_H
#define KLEE_SUBSTITUTIONSOLVER_H

#include "klee/Expr.h"
#include "klee/Solver.h"
#include "klee/SolverImpl.h"

#include <vector>

namespace klee {
  class ExecutionState;
  class Solver;  

  class SubstitutionSolver : public SolverImpl {
  public:

    Solver *solver;

  public:

    SubstitutionSolver(Solver *solver) : solver(solver) {

    }

    ~SubstitutionSolver() {
      delete solver;
    }

    bool computeTruth(const Query &query, bool &isValid) {
      return solver->impl->computeTruth(query, isValid);
    }

    bool computeValue(const Query& query, ref<Expr> &result) {
      return solver->impl->computeValue(query, result);
    }

    bool computeInitialValues(const Query& query,
                              const std::vector<const Array*> &objects,
                              std::vector< std::vector<unsigned char> > &values,
                              bool &hasSolution) {
      return solver->impl->computeInitialValues(query, objects, values, hasSolution);
    }

    SolverRunStatus getOperationStatusCode() {
      return solver->impl->getOperationStatusCode();
    }

    char *getConstraintLog(const Query& query) {
      return solver->getConstraintLog(query);
    }

    void setCoreSolverTimeout(time::Span timeout) {
      solver->impl->setCoreSolverTimeout(timeout);
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
