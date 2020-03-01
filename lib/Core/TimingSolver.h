//===-- TimingSolver.h ------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_TIMINGSOLVER_H
#define KLEE_TIMINGSOLVER_H

#include "klee/Expr.h"
#include "klee/Solver.h"
#include "klee/Internal/System/Time.h"

#include <vector>

namespace klee {
  class ExecutionState;
  class Solver;  

  /* TODO: remove */
  struct SolverQuery {

  public:
    std::vector<ref<Expr>> constraints;
    ref<Expr> expr;

    SolverQuery(std::vector<ref<Expr>> &constraints, ref<Expr> expr)
      : constraints(constraints), expr(expr) {
    }

    bool isEqual(const SolverQuery &other) const;

    bool isIsomorphic(const SolverQuery &other) const;

    void dump() const;
  };

  struct CacheResult {
    bool _isTrue;
    bool _isFalse;
    bool _isUnknown;

    CacheResult(): _isTrue(false), _isFalse(false), _isUnknown(false) {

    }

    CacheResult(bool isTrue, bool isFalse, bool isUnknown) :
      _isTrue(isTrue), _isFalse(isFalse), _isUnknown(isUnknown) {
      assert(!(isTrue && isFalse));
    }

    CacheResult(Solver::Validity validity) {
      setValue(validity);
    }

    bool mustBeTrue() {
      return _isTrue && !_isFalse && !_isUnknown;
    }

    bool mustBeFalse() {
      return !_isTrue && _isFalse && !_isUnknown;
    }

    bool mayBeTrue() {
      return _isTrue && !_isFalse && _isUnknown;
    }

    bool mayBeFalse() {
      return !_isTrue && _isFalse && _isUnknown;
    }

    bool isUnknown() {
      return !_isTrue && !_isFalse && _isUnknown;
    }

    void setMustBeTrue() {
      _isTrue = true; _isFalse = false; _isUnknown = false;
    }

    void setMustBeFalse() {
      _isTrue = false; _isFalse = true; _isUnknown = false;
    }

    void setMayBeTrue() {
      _isTrue = true; _isFalse = false; _isUnknown = true;
    }

    void setMayBeFalse() {
      _isTrue = false; _isFalse = true; _isUnknown = true;
    }

    void setUnknown() {
      _isTrue = false; _isFalse = false; _isUnknown = true;
    }

    void setValue(Solver::Validity validity) {
      switch (validity) {
      case Solver::True:
        setMustBeTrue();
        break;
      case Solver::False:
        setMustBeFalse();
        break;
      case Solver::Unknown:
        setUnknown();
        break;
      default:
        assert(false);
        break;
      }
    }

    CacheResult negate() {
      CacheResult result;
      if (mustBeTrue()) {
        result.setMustBeFalse();
      }
      else if (mustBeFalse()) {
        result.setMustBeTrue();
      }
      else if (mayBeFalse()) {
        result.setMayBeTrue();
      }
      else if (mayBeTrue()) {
        result.setMayBeFalse();
      }
      else if (isUnknown()) {
        result.setUnknown();
      } else {
        assert(false);
      }
      return result;
    }

    void dump() const {
      llvm::errs() << _isTrue << _isFalse << _isUnknown << "\n";
    }
  };

  struct CacheEntry {
    SolverQuery q;
    CacheResult result;

    CacheEntry(SolverQuery &q, CacheResult result) : q(q), result(result) {

    }
  };

  /// TimingSolver - A simple class which wraps a solver and handles
  /// tracking the statistics that we care about.
  class TimingSolver {
  public:
    Solver *solver;
    bool simplifyExprs;
    /* TODO: remove */
    std::vector<SolverQuery> queries;
    std::vector<SolverQuery> equivalent;
    uint64_t queries_count = 0;

    typedef std::vector<CacheEntry> Cache;
    Cache cache;

  public:
    /// TimingSolver - Construct a new timing solver.
    ///
    /// \param _simplifyExprs - Whether expressions should be
    /// simplified (via the constraint manager interface) prior to
    /// querying.
    TimingSolver(Solver *_solver, bool _simplifyExprs = true) 
      : solver(_solver), simplifyExprs(_simplifyExprs) {}
    ~TimingSolver() {
      delete solver;
    }

    void setTimeout(time::Span t) {
      solver->setCoreSolverTimeout(t);
    }
    
    char *getConstraintLog(const Query& query) {
      return solver->getConstraintLog(query);
    }

    bool evaluate(const ExecutionState&, ref<Expr>, Solver::Validity &result);

    bool mustBeTrue(const ExecutionState&, ref<Expr>, bool &result, bool useCache = true);

    bool mustBeFalse(const ExecutionState&, ref<Expr>, bool &result);

    bool mayBeTrue(const ExecutionState&, ref<Expr>, bool &result);

    bool mayBeFalse(const ExecutionState&, ref<Expr>, bool &result);

    bool getValue(const ExecutionState &, ref<Expr> expr, 
                  ref<ConstantExpr> &result);

    bool getInitialValues(const ExecutionState&, 
                          const std::vector<const Array*> &objects,
                          std::vector< std::vector<unsigned char> > &result);

    std::pair< ref<Expr>, ref<Expr> >
    getRange(const ExecutionState&, ref<Expr> query);

    void fillConstraints(const ExecutionState &state,
                         ConstraintManager &cm,
                         ref<Expr> q);

    ref<Expr> canonicalizeQuery(ref<Expr> query, bool &negationUsed);

    SolverQuery buildQuery(const ExecutionState &state,
                           ref<Expr> expr);

    void collectStats(const ExecutionState &state, ref<Expr> expr);

    bool shouldCacheQuery(ref<Expr> expr);

    bool lookupQuery(const ExecutionState &state,
                     SolverQuery &query,
                     CacheResult &result);

    void insertQuery(const ExecutionState &state, SolverQuery &query, CacheResult &result);

    void dump() const;
  };
}

#endif
