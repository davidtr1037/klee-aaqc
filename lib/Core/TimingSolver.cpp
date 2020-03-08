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
#include "klee/Internal/Support/ErrorHandling.h"

#include "CoreStats.h"

using namespace klee;
using namespace llvm;

cl::opt<bool> CollectQueryStats("collect-query-stats", cl::init(false), cl::desc("..."));
cl::opt<bool> UseIsomorphismCache("use-iso-cache", cl::init(false), cl::desc("..."));
cl::opt<bool> ValidateCaching("validate-caching", cl::init(false), cl::desc("..."));
cl::opt<bool> UseMapCache("use-map-cache", cl::init(true), cl::desc("..."));

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
  allQueriesCount++;

  if (simplifyExprs)
    expr = state.constraints.simplifyExpr(expr);

  if (CollectQueryStats) {
    collectStats(state, ade);
  }

  bool success = false;
  if (shouldCacheQuery(ade)) {
    if (simplifyExprs) {
      TimerStatIncrementer timer(stats::cachingTime);
      ade = state.constraints.simplifyExpr(ade);
    }
    if (isa<ConstantExpr>(ade)) {
      result = dyn_cast<ConstantExpr>(ade)->isTrue() ? Solver::True : Solver::False;
      return true;
    }

    SolverQuery q = buildQuery(state, ade);
    CacheResult cachedResult;
    bool found = lookupQuery(state, q, cachedResult);
    if (found) {
      success = true;
      if (cachedResult.mustBeTrue()) {
        result = Solver::True;
      } else if (cachedResult.mustBeFalse()) {
        result = Solver::False;
      } else if (cachedResult.isUnknown()) {
        result = Solver::Unknown;
      } else {
        /* may be false or may be true... */
        assert(cachedResult.mayBeFalse() || cachedResult.mayBeTrue());
        success = solver->evaluate(Query(state.rewrittenConstraints, expr), result);
        CacheResult newResult(result);
        insertQuery(state, q, newResult);
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
  allQueriesCount++;

  if (simplifyExprs)
    expr = state.constraints.simplifyExpr(expr);

  if (CollectQueryStats) {
    if (useCache) {
      collectStats(state, ade);
    } else {
      unhandledQueries++;
    }
  }

  bool success = false;
  if (useCache && shouldCacheQuery(ade)) {
    if (simplifyExprs) {
      TimerStatIncrementer timer(stats::cachingTime);
      ade = state.constraints.simplifyExpr(ade);
    }
    if (isa<ConstantExpr>(ade)) {
      result = dyn_cast<ConstantExpr>(ade)->isTrue() ? Solver::True : Solver::False;
      return true;
    }

    SolverQuery q = buildQuery(state, ade);
    CacheResult cachedResult;
    bool found = lookupQuery(state, q, cachedResult);
    if (found) {
      if (cachedResult.mayBeTrue()) {
        CacheResult newResult;
        success = solver->mustBeTrue(Query(state.rewrittenConstraints, expr), result);
        if (result) {
          /* mayByTrue and mustBeTrue --> mustBeTrue */
          newResult.setMustBeTrue();
        } else {
          /* mayByTrue and mayBeFalse --> unknown */
          newResult.setUnknown();
        }
        insertQuery(state, q, newResult);
      } else {
        success = true;
        result = cachedResult.mustBeTrue();
      }

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
  allQueriesCount++;

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
  allQueriesCount++;

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

ref<Expr> TimingSolver::canonicalizeQuery(ref<Expr> expr,
                                          bool &negationUsed) {
  ref<EqExpr> eq = dyn_cast<EqExpr>(expr);
  if (!eq.isNull()) {
    ref<ConstantExpr> c = dyn_cast<ConstantExpr>(eq->left);
    if (!c.isNull() && c->isFalse()) {
      negationUsed = true;
      return eq->right;
    }
  }

  negationUsed = false;
  return expr;
}

SolverQuery TimingSolver::buildQuery(const ExecutionState &state,
                                     ref<Expr> expr) {
  TimerStatIncrementer timer(stats::cachingTime);
  assert(!isa<ConstantExpr>(expr));
  Query query(state.constraints, expr);
  std::vector<ref<Expr>> required;
  sliceConstraints(query, required);
  return SolverQuery(required, expr);
}

void TimingSolver::collectStats(const ExecutionState &state, ref<Expr> expr) {
  TimerStatIncrementer timer(stats::cachingTime);
  if (simplifyExprs) {
    expr = state.constraints.simplifyExpr(expr);
  }

  if (isa<ConstantExpr>(expr)) {
    return;
  }

  if (expr->flag) {
    addressDependentQueries++;
  }

  bool wasNegated;
  expr = canonicalizeQuery(expr, wasNegated);

  /* slice the path constraints */
  SolverQuery q = buildQuery(state, expr);
  relevantQueries++;

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
  /* TODO: handle only address dependent expressions? */
  return UseIsomorphismCache;
}

bool TimingSolver::lookupQuery(const ExecutionState &state,
                               SolverQuery &query,
                               CacheResult &result) {
  TimerStatIncrementer timer(stats::cachingTime);
  bool negated;
  ref<Expr> expr = canonicalizeQuery(query.expr, negated);
  SolverQuery canonicalQuery(query.constraints, expr);

  if (UseMapCache) {
    auto i = queryMap.find(canonicalQuery);
    if (i != queryMap.end()) {
      if (negated) {
        result = i->second.negate();
      } else {
        result = i->second;
      }
      return true;
    }
    return false;
  } else {
    for (CacheEntry &e : queryList) {
      if (canonicalQuery.isIsomorphic(e.q)) {
        if (negated) {
          result = e.result.negate();
        } else {
          result = e.result;
        }
        return true;
      }
    }
    return false;
  }
}

void TimingSolver::insertQuery(const ExecutionState &state, SolverQuery &query, CacheResult &result) {
  TimerStatIncrementer timer(stats::cachingTime);
  bool negated;
  query.expr = canonicalizeQuery(query.expr, negated);
  if (negated) {
    if (UseMapCache) {
      queryMap.insert(std::make_pair(query, result.negate()));
    } else {
      queryList.push_back(CacheEntry(query, result.negate()));
    }
  } else {
    if (UseMapCache) {
      queryMap.insert(std::make_pair(query, result));
    } else {
      queryList.push_back(CacheEntry(query, result));
    }
  }
}

void TimingSolver::dump() const {
  klee_message("Statistics");
  klee_message("- All queries: %lu", allQueriesCount);
  klee_message("- Unhandled queries: %lu", unhandledQueries);
  klee_message("- Relevant queries: %lu", relevantQueries);
  klee_message("- Relevant address dependent queries: %lu", addressDependentQueries);
  klee_message("- Equal queries: %lu", queries.size());
  klee_message("- Isomorphic queries: %lu", equivalent.size());
  if (UseMapCache) {
    klee_message("- Cache: %lu", queryMap.size());
  } else {
    klee_message("- Cache: %lu", queryList.size());
  }
}
