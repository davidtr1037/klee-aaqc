#include "SubstitutionSolver.h"

#include "klee/Config/Version.h"
#include "klee/ExecutionState.h"
#include "klee/Solver.h"
#include "klee/Statistics.h"

#include "CoreStats.h"

using namespace klee;
using namespace llvm;

bool SubstitutionSolver::evaluate(const ExecutionState& state,
                                  ref<Expr> expr,
                                  Solver::Validity &result) {
  expr = state.unfold(expr);
  return solver->evaluate(Query(state.rewrittenConstraints, expr), result);
}

bool SubstitutionSolver::mustBeTrue(const ExecutionState& state,
                                    ref<Expr> expr,
                                    bool &result) {
  expr = state.unfold(expr);
  return solver->mustBeTrue(Query(state.rewrittenConstraints, expr), result);
}

bool SubstitutionSolver::mustBeFalse(const ExecutionState& state,
                                     ref<Expr> expr,
                                     bool &result) {
  return mustBeTrue(state, Expr::createIsZero(expr), result);
}

bool SubstitutionSolver::mayBeTrue(const ExecutionState& state,
                                   ref<Expr> expr,
                                   bool &result) {
  bool res;
  if (!mustBeFalse(state, expr, res)) {
    return false;
  }
  result = !res;
  return true;
}

bool SubstitutionSolver::mayBeFalse(const ExecutionState& state,
                                    ref<Expr> expr,
                                    bool &result) {
  bool res;
  if (!mustBeTrue(state, expr, res)) {
    return false;
  }
  result = !res;
  return true;
}

bool SubstitutionSolver::getValue(const ExecutionState& state,
                                  ref<Expr> expr,
                                  ref<ConstantExpr> &result) {
  expr = state.unfold(expr);
  return solver->getValue(Query(state.rewrittenConstraints, expr), result);
}

bool SubstitutionSolver::getInitialValues(const ExecutionState& state,
                                          const std::vector<const Array*> &objects,
                                          std::vector<std::vector<unsigned char>> &result) {
  if (objects.empty()) {
    return true;
  }

  return solver->getInitialValues(Query(state.rewrittenConstraints,
                                                ConstantExpr::alloc(0, Expr::Bool)),
                                          objects, result);
}

std::pair<ref<Expr>, ref<Expr>>
SubstitutionSolver::getRange(const ExecutionState& state, ref<Expr> expr) {
  return solver->getRange(Query(state.constraints, expr));
}
