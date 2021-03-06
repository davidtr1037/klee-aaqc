//===-- ExprUtil.cpp ------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/util/ExprUtil.h"
#include "klee/util/ExprHashMap.h"

#include "klee/Expr.h"

#include "klee/util/ExprVisitor.h"

#include <set>

using namespace klee;

void klee::findReads(ref<Expr> e, 
                     bool visitUpdates,
                     std::vector< ref<ReadExpr> > &results) {
  // Invariant: \forall_{i \in stack} !i.isConstant() && i \in visited 
  std::vector< ref<Expr> > stack;
  ExprHashSet visited;
  std::set<const UpdateNode *> updates;
  
  if (!isa<ConstantExpr>(e)) {
    visited.insert(e);
    stack.push_back(e);
  }

  while (!stack.empty()) {
    ref<Expr> top = stack.back();
    stack.pop_back();
    if (!top->isSymbolic) {
      continue;
    }

    if (ReadExpr *re = dyn_cast<ReadExpr>(top)) {
      // We memoized so can just add to list without worrying about
      // repeats.
      results.push_back(re);

      if (!isa<ConstantExpr>(re->index) && visited.insert(re->index).second) {
        if (re->index->isSymbolic) {
          stack.push_back(re->index);
        }
      }
      
      if (visitUpdates && re->updates.isSymbolic()) {
        // XXX this is probably suboptimal. We want to avoid a potential
        // explosion traversing update lists which can be quite
        // long. However, it seems silly to hash all of the update nodes
        // especially since we memoize all the expr results anyway. So
        // we take a simple approach of memoizing the results for the
        // head, which often will be shared among multiple nodes.
        if (updates.insert(re->updates.head.get()).second) {
          for (const UpdateNode *un=re->updates.head.get(); un; un=un->next.get()) {
            if (!isa<ConstantExpr>(un->index) && visited.insert(un->index).second) {
              if (un->index->isSymbolic) {
                stack.push_back(un->index);
              }
            }
            if (!isa<ConstantExpr>(un->value) && visited.insert(un->value).second) {
              if (un->value->isSymbolic) {
                stack.push_back(un->value);
              }
            }
          }
        }
      }
    } else if (!isa<ConstantExpr>(top)) {
      Expr *e = top.get();
      for (unsigned i=0; i<e->getNumKids(); i++) {
        ref<Expr> k = e->getKid(i);
        if (!isa<ConstantExpr>(k) && visited.insert(k).second && k->isSymbolic) {
          stack.push_back(k);
        }
      }
    }
  }
}

///

namespace klee {

class SymbolicObjectFinder : public ExprVisitor {
protected:
  Action visitRead(const ReadExpr &re) {
    const UpdateList &ul = re.updates;

    // XXX should we memo better than what ExprVisitor is doing for us?
    for (const UpdateNode *un=ul.head.get(); un; un=un->next.get()) {
      visit(un->index);
      visit(un->value);
    }

    if (ul.root->isSymbolicArray())
      if (results.insert(ul.root).second)
        objects.push_back(ul.root);

    return Action::doChildren();
  }

public:
  std::set<const Array*> results;
  std::vector<const Array*> &objects;
  
  SymbolicObjectFinder(std::vector<const Array*> &_objects)
    : objects(_objects) {}
};

ExprVisitor::Action ConstantArrayFinder::visitRead(const ReadExpr &re) {
  const UpdateList &ul = re.updates;

  // FIXME should we memo better than what ExprVisitor is doing for us?
  for (const UpdateNode *un = ul.head.get(); un; un = un->next.get()) {
    visit(un->index);
    visit(un->value);
  }

  if (ul.root->isConstantArray()) {
    results.insert(ul.root);
  }

  return Action::doChildren();
}

ExprVisitor::Action AddressExprFinder::visitRead(const ReadExpr &e) {
  std::string name = e.updates.root->getName();
  if (!e.updates.root->isAddressArray) {
    isPureAddressExpr = false;
    return Action::skipChildren();
  } else {
    return Action::doChildren();
  }
}

ExprVisitor::Action AddressArrayCollector::visitRead(const ReadExpr &e) {
  if (e.updates.root->isAddressArray) {
    arrays.insert(e.updates.root->getName());
    ids.insert(e.updates.root->id);
  }

  const UpdateNode *h = e.updates.head.get();
  for (const UpdateNode *n = h; n != NULL; n = n->next.get()) {
    /* TODO: may result in infinite recursion? */
    visit(n->value);
  }

  return Action::doChildren();
}

}

template<typename InputIterator>
void klee::findSymbolicObjects(InputIterator begin, 
                               InputIterator end,
                               std::vector<const Array*> &results) {
  SymbolicObjectFinder of(results);
  for (; begin!=end; ++begin)
    of.visit(*begin);
}

void klee::findSymbolicObjects(ref<Expr> e,
                               std::vector<const Array*> &results) {
  findSymbolicObjects(&e, &e+1, results);
}

typedef std::vector< ref<Expr> >::iterator A;
template void klee::findSymbolicObjects<A>(A, A, std::vector<const Array*> &);

typedef std::set< ref<Expr> >::iterator B;
template void klee::findSymbolicObjects<B>(B, B, std::vector<const Array*> &);
