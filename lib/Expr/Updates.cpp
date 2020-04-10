//===-- Updates.cpp -------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/Expr.h"

#include <cassert>

using namespace klee;

///

UpdateNode::UpdateNode(const ref<UpdateNode> _next,
                       const ref<Expr> &_index,
                       const ref<Expr> &_value) 
  : next(_next),
    index(_index),
    value(_value),
    refCount(0) {
  // FIXME: What we need to check here instead is that _value is of the same width 
  // as the range of the array that the update node is part of.
  /*
  assert(_value->getWidth() == Expr::Int8 && 
         "Update value should be 8-bit wide.");
  */
  computeHash();
  if (!next.isNull()) {
    size = 1 + next->size;
  } else {
    size = 1;
  }

  isSymbolic = (!next.isNull() && next->isSymbolic) || index->isSymbolic || value->isSymbolic;
}

extern "C" void vc_DeleteExpr(void*);

// This is deliberately empty to avoid recursively deleting UpdateNodes
// (i.e. ``if (--next->refCount==0) delete next;``).
// UpdateList manages the correct destruction of a chain UpdateNodes
// non-recursively.

int UpdateNode::compare(const UpdateNode &b) const {
  if (int i = index.compare(b.index)) 
    return i;
  return value.compare(b.value);
}

void UpdateNode::computeHash() {
  hashValue = index->hash() ^ value->hash();
  checksum = index->getChecksum() ^ value->getChecksum();
  if (!next.isNull()) {
    hashValue ^= next->hash();
    checksum ^= next->getChecksum();
  }
  //return hashValue;
}

bool UpdateNode::isIsomorphic(const UpdateNode &b,
                              ArrayMapping &map) const {
  return index->isIsomorphic(*b.index, map) && value->isIsomorphic(*b.value, map);
}

///

UpdateList::UpdateList(const Array *_root, const ref<UpdateNode> &_head)
  : root(_root),
    head(_head) {

}

void UpdateList::extend(const ref<Expr> &index, const ref<Expr> &value) {
  
  if (root) {
    assert(root->getDomain() == index->getWidth());
    assert(root->getRange() == value->getWidth());
  }

  head = new UpdateNode(head, index, value);
}

int UpdateList::compare(const UpdateList &b) const {
  if (root->name != b.root->name)
    return root->name < b.root->name ? -1 : 1;

  // Check the root itself in case we have separate objects with the
  // same name.
  if (root != b.root)
    return root < b.root ? -1 : 1;

  if (getSize() < b.getSize()) return -1;
  else if (getSize() > b.getSize()) return 1;    

  // XXX build comparison into update, make fast
  const UpdateNode *an=head.get(), *bn=b.head.get();
  for (; an && bn; an=an->next.get(),bn=bn->next.get()) {
    if (an==bn) { // exploit shared list structure
      return 0;
    } else {
      if (int res = an->compare(*bn))
        return res;
    }
  }
  assert(!an && !bn);  
  return 0;
}

unsigned UpdateList::hash() const {
  unsigned res = 0;
  for (unsigned i = 0, e = root->name.size(); i != e; ++i)
    res = (res * Expr::MAGIC_HASH_CONSTANT) + root->name[i];
  if (!head.isNull())
    res ^= head->hash();
  return res;
}

unsigned UpdateList::getChecksum() const {
  return 1337;
}

bool UpdateList::isIsomorphic(const UpdateList &b, ArrayMapping &map) const {
  if (getSize() != b.getSize()) {
    return false;
  }

  if (root->size != b.root->size) {
    return false;
  }

  if (root->constantValues.size() != b.root->constantValues.size()) {
    return false;
  }

  for (unsigned int i = 0; i < root->constantValues.size(); i++) {
    ref<ConstantExpr> e1 = root->constantValues[i];
    ref<ConstantExpr> e2 = b.root->constantValues[i];
    if (e1->compareContents(*e2) != 0) {
      return false;
    }
  }

  unsigned index = 0;
  bool canSkip = false;

  const UpdateNode *an = head.get(), *bn = b.head.get();
  for (; an && bn; an = an->next.get(), bn = bn->next.get()) {
    if (canSkip && index < 8) {
      index++;
      continue;
    } else {
      canSkip = false;
    }

    /* TODO: exploit shared list structure? */
    if (!an->isIsomorphic(*bn, map)) {
        return false;
    }
    ref<ReadExpr> v = dyn_cast<ReadExpr>(an->value);
    if (!v.isNull() && v->isAddressValue()) {
      //an->value->dump();
      //an->index->dump();
      if (!canSkip) {
        canSkip = true;
        index = 1;
      }
    }
  }

  assert(!an && !bn);
  return map.add(root, b.root);
}
