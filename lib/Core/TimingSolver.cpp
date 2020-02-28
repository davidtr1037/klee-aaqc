//===-- TimingSolver.cpp --------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "TimingSolver.h"

#include "klee/Config/Version.h"
#include "klee/ExecutionState.h"
#include "klee/Solver.h"
#include "klee/Statistics.h"
#include "klee/TimerStatIncrementer.h"

#include "CoreStats.h"

using namespace klee;
using namespace llvm;

cl::opt<bool> CollectQueryStats("collect-query-stats", cl::init(false), cl::desc("..."));
cl::opt<bool> UseIsomorphismCache("use-iso-cache", cl::init(false), cl::desc("..."));
cl::opt<bool> ValidateCaching("validate-caching", cl::init(false), cl::desc("..."));

/***/

bool SolverQuery::isEqual(const SolverQuery &other) const {
  if (constraints.size() != other.constraints.size()) {
    return false;
  }

  if (expr->compare(*other.expr) != 0) {
    return false;
  }

  auto iter1 = constraints.begin();
  auto iter2 = other.constraints.begin();
  while (iter1 != constraints.end()) {
    ref<Expr> e1 = *iter1;
    ref<Expr> e2 = *iter2;
    if (e1->compare(*e2) != 0) {
      return false;
    }

    iter1++; iter2++;
  }
  return true;
}

bool SolverQuery::isIsomorphic(const SolverQuery &other) const {
  if (constraints.size() != other.constraints.size()) {
    return false;
  }

  /* we must preserve the same mapping for all constraints (pc and expr) */
  ArrayMapping map;
  if (!expr->isIsomorphic(*other.expr, map)) {
    return false;
  }

  auto iter1 = constraints.begin();
  auto iter2 = other.constraints.begin();
  while (iter1 != constraints.end()) {
    ref<Expr> e1 = *iter1;
    ref<Expr> e2 = *iter2;
    if (!e1->isIsomorphic(*e2, map)) {
      return false;
    }

    iter1++; iter2++;
  }
  return true;
}

void SolverQuery::dump() const {
  llvm::errs() << "Constraints [\n";
  for (ConstraintManager::const_iterator i = constraints.begin();
      i != constraints.end(); i++) {
    (*i)->dump();
  }
  llvm::errs() << "]\n";
  llvm::errs() << "Query [\n";
  expr->dump();
  llvm::errs() << "]\n";
}

bool TimingSolver::evaluate(const ExecutionState& state, ref<Expr> expr,
                            Solver::Validity &result) {
  ref<Expr> ade = expr;
  expr = state.unfold(expr);
  // Fast path, to avoid timer and OS overhead.
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(expr)) {
    result = CE->isTrue() ? Solver::True : Solver::False;
    return true;
  }

  TimerStatIncrementer timer(stats::solverTime);

  if (simplifyExprs)
    expr = state.constraints.simplifyExpr(expr);

  if (CollectQueryStats) {
    collectStats(state, expr);
  }

  bool success = false;
  if (shouldCacheQuery(ade)) {
    Query query(state.constraints, ade);
    std::vector<ref<Expr>> required;
    sliceConstraints(query, required);
    SolverQuery q(required, ade);

    CacheResult *cachedResult = lookupQuery(state, q);
    if (cachedResult) {
      if (cachedResult->mustBeTrue()) {
        result = Solver::True;
        success = true;
      } else if (cachedResult->mustBeFalse()) {
        result = Solver::False;
        success = true;
      } else if (cachedResult->isUnknown()) {
        result = Solver::Unknown;
        success = true;
      } else {
        /* must be false or unknown... */
        assert(cachedResult->mayBeFalse());
        success = solver->evaluate(Query(state.rewrittenConstraints, expr), result);
        cachedResult->setValue(result);
      }
      if (ValidateCaching) {
        Solver::Validity test;
        success = solver->evaluate(Query(state.rewrittenConstraints, expr), test);
        assert(result == test);
      }
    } else {
      success = solver->evaluate(Query(state.rewrittenConstraints, expr), result);
      if (success) {
        CacheResult newResult(result);
        insertQuery(state, q, newResult);
      }
    }
  } else {
    success = solver->evaluate(Query(state.rewrittenConstraints, expr), result);
  }

  //bool success = solver->evaluate(Query(state.rewrittenConstraints, expr), result);

  state.queryCost += timer.check();

  return success;
}

bool TimingSolver::mustBeTrue(const ExecutionState& state, ref<Expr> expr, 
                              bool &result, bool useCache) {
  ref<Expr> ade = expr;
  expr = state.unfold(expr);
  // Fast path, to avoid timer and OS overhead.
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(expr)) {
    result = CE->isTrue() ? true : false;
    return true;
  }

  TimerStatIncrementer timer(stats::solverTime);

  if (simplifyExprs)
    expr = state.constraints.simplifyExpr(expr);

  if (useCache && CollectQueryStats) {
    collectStats(state, expr);
  }

  bool success = false;
  if (useCache && shouldCacheQuery(ade)) {
    Query query(state.constraints, ade);
    std::vector<ref<Expr>> required;
    sliceConstraints(query, required);
    SolverQuery q(required, ade);
    CacheResult *cachedResult = lookupQuery(state, q);
    if (cachedResult) {
      result = cachedResult->mustBeTrue();
      success = true;
      if (ValidateCaching) {
        bool test;
        success = solver->mustBeTrue(Query(state.rewrittenConstraints, expr), test);
        assert(result == test);
      }
    } else {
      success = solver->mustBeTrue(Query(state.rewrittenConstraints, expr), result);
      CacheResult newResult;
      if (result) {
        newResult.setMustBeTrue();
      } else {
        newResult.setMayBeFalse();
      }
      insertQuery(state, q, newResult);
    }
  } else {
    success = solver->mustBeTrue(Query(state.rewrittenConstraints, expr), result);
  }

  //bool success = solver->mustBeTrue(Query(state.rewrittenConstraints, expr), result);

  state.queryCost += timer.check();

  return success;
}

bool TimingSolver::mustBeFalse(const ExecutionState& state, ref<Expr> expr,
                               bool &result) {
  return mustBeTrue(state, Expr::createIsZero(expr), result);
}

bool TimingSolver::mayBeTrue(const ExecutionState& state, ref<Expr> expr, 
                             bool &result) {
  bool res;
  if (!mustBeFalse(state, expr, res))
    return false;
  result = !res;
  return true;
}

bool TimingSolver::mayBeFalse(const ExecutionState& state, ref<Expr> expr, 
                              bool &result) {
  bool res;
  if (!mustBeTrue(state, expr, res))
    return false;
  result = !res;
  return true;
}

bool TimingSolver::getValue(const ExecutionState& state, ref<Expr> expr, 
                            ref<ConstantExpr> &result) {
  expr = state.unfold(expr);
  // Fast path, to avoid timer and OS overhead.
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(expr)) {
    result = CE;
    return true;
  }
  
  TimerStatIncrementer timer(stats::solverTime);

  if (simplifyExprs)
    expr = state.constraints.simplifyExpr(expr);

  //ConstraintManager cm;
  //fillConstraints(state, cm, expr);
  //bool success = solver->getValue(Query(cm, expr), result);
  bool success = solver->getValue(Query(state.rewrittenConstraints, expr), result);
  //bool success = solver->getValue(Query(state.constraints, expr), result);

  state.queryCost += timer.check();

  return success;
}

bool 
TimingSolver::getInitialValues(const ExecutionState& state, 
                               const std::vector<const Array*>
                                 &objects,
                               std::vector< std::vector<unsigned char> >
                                 &result) {
  if (objects.empty())
    return true;

  TimerStatIncrementer timer(stats::solverTime);

  //ConstraintManager cm;
  //fillConstraints(state, cm, nullptr);
  //bool success = solver->getInitialValues(Query(cm, ConstantExpr::alloc(0, Expr::Bool)),
  //                                        objects, result);
  bool success = solver->getInitialValues(Query(state.rewrittenConstraints,
                                                ConstantExpr::alloc(0, Expr::Bool)),
                                          objects, result);
  
  state.queryCost += timer.check();
  
  return success;
}

std::pair< ref<Expr>, ref<Expr> >
TimingSolver::getRange(const ExecutionState& state, ref<Expr> expr) {
  return solver->getRange(Query(state.constraints, expr));
}

void TimingSolver::fillConstraints(const ExecutionState &state,
                                   ConstraintManager &cm,
                                   ref<Expr> q) {
  std::vector<ref<Expr>> conditions;
  for (ref<Expr> e : state.rewrittenConstraints) {
    cm.addConstraintNoOptimize(e);
    conditions.push_back(e);
  }

  //ref<Expr> extra = nullptr;
  //if (!q.isNull()) {
  //  conditions.push_back(q);
  //}

  //extra = state.build(conditions);
  //cm.addConstraint(extra);
}

ref<Expr> TimingSolver::canonicalizeQuery(ref<Expr> query,
                                          bool &negationUsed) {
  ref<Expr> negated = Expr::createIsZero(query);
  if (query.compare(negated) < 0) {
    negationUsed = false;
    return query;
  } else {
    negationUsed = true;
    return negated;
  }
}

void TimingSolver::collectStats(const ExecutionState &state, ref<Expr> expr) {
  if (!expr->flag) {
    return;
  }

  ref<Expr> u = state.unfold(expr);
  if (isa<ConstantExpr>(u)) {
    return;
  }

  bool wasNegated;
  expr = canonicalizeQuery(expr, wasNegated);

  /* slice the path constraints */
  Query query(state.constraints, expr);
  std::vector<ref<Expr>> required;
  sliceConstraints(query, required);
  SolverQuery q(required, expr);
  queries_count++;

  bool found;

  found = false;
  for (SolverQuery other : queries) {
    if (q.isEqual(other)) {
      found = true;
      break;
    }
  }
  if (!found) {
    queries.push_back(q);
  }

  found = false;
  for (SolverQuery other : equivalent) {
    if (q.isIsomorphic(other)) {
      found = true;
      break;
    }
  }
  if (!found) {
    equivalent.push_back(q);
  }
}

bool TimingSolver::shouldCacheQuery(ref<Expr> expr) {
  return UseIsomorphismCache & expr->flag;
}

CacheResult *TimingSolver::lookupQuery(const ExecutionState &state, SolverQuery &query) {
  for (CacheEntry &e : cache) {
    if (query.isIsomorphic(e.q)) {
      return &e.result;
    }
  }
  return nullptr;
}

void TimingSolver::insertQuery(const ExecutionState &state, SolverQuery &query, CacheResult &result) {
  cache.push_back(CacheEntry(query, result));
}
