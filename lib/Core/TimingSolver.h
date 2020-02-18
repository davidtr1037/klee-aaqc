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

  /// TimingSolver - A simple class which wraps a solver and handles
  /// tracking the statistics that we care about.
  class TimingSolver {
  public:
    Solver *solver;
    bool simplifyExprs;
    /* TODO: remove */
    //std::vector<Query> queries;
    std::vector<SolverQuery> queries;

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

    bool mustBeTrue(const ExecutionState&, ref<Expr>, bool &result);

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

    void handleExpr(const ExecutionState &state, ref<Expr> expr);
  };

}

#endif
