//===-- ExecutionState.cpp ------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Memory.h"
#include "MemoryManager.h"

#include "klee/ExecutionState.h"

#include "klee/Expr.h"
#include "klee/Internal/Module/Cell.h"
#include "klee/Internal/Module/InstructionInfoTable.h"
#include "klee/Internal/Module/KInstruction.h"
#include "klee/Internal/Module/KModule.h"
#include "klee/OptionCategories.h"
#include "klee/util/ExprUtil.h"
#include "klee/util/ArrayCache.h"
#include "klee/Internal/Support/ErrorHandling.h"

#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/raw_ostream.h"

#include <cassert>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <list>
#include <stdarg.h>

using namespace llvm;
using namespace klee;

namespace {
cl::opt<bool> DebugLogStateMerge(
    "debug-log-state-merge", cl::init(false),
    cl::desc("Debug information for underlying state merging (default=false)"),
    cl::cat(MergeCat));
}

cl::opt<bool> klee::UseLocalSymAddr("use-local-sym-addr", cl::init(false), cl::desc("..."));

cl::opt<bool> klee::ReuseArrays("reuse-arrays", cl::init(true), cl::desc("..."));

cl::opt<unsigned> klee::UseKContext("use-kcontext", cl::init(0), cl::desc("..."));

cl::opt<bool> klee::UseGlobalID("use-global-id", cl::init(true), cl::desc("..."));

cl::opt<bool> klee::UseGlobalRewriteCache("use-global-rewrite-cache", cl::init(true), cl::desc("..."));

/***/

StackFrame::StackFrame(KInstIterator _caller, KFunction *_kf)
  : caller(_caller), kf(_kf), callPathNode(0), 
    minDistToUncoveredOnReturn(0), varargs(0) {
  locals = new Cell[kf->numRegisters];
}

StackFrame::StackFrame(const StackFrame &s) 
  : caller(s.caller),
    kf(s.kf),
    callPathNode(s.callPathNode),
    allocas(s.allocas),
    minDistToUncoveredOnReturn(s.minDistToUncoveredOnReturn),
    varargs(s.varargs) {
  locals = new Cell[s.kf->numRegisters];
  for (unsigned i=0; i<s.kf->numRegisters; i++)
    locals[i] = s.locals[i];
}

StackFrame::~StackFrame() { 
  delete[] locals; 
}

AddressRecord::AddressRecord(uint64_t c, ref<Expr> alpha) : alpha(alpha), refCount(0) {
  address = ConstantExpr::create(c, Context::get().getPointerWidth());
  for (unsigned i = 0; i < 8; i++) {
    uint64_t value = (c >> (i * 8)) & 0xff;
    ref<ConstantExpr> e = ConstantExpr::create(value, Expr::Int8);
    bytes.push_back(e);
  }
  constraint = EqExpr::create(address, alpha);
}

void RebaseID::dump() const {
  errs() << "RID: " << "\n";
  errs() << "- Instruction: " << info->id << "\n";
  errs() << "- Size: " << size << "\n";
  errs() << "- Arrays: [ ";
  for (uint64_t a : arrays) {
    errs() << a << " ";
  }
  errs() << "]\n";
  errs() << "- Addresses: [ ";
  for (uint64_t a : addrs) {
    errs() << a << " ";
  }
  errs() << "]\n";
}

RebaseCache *RebaseCache::instance = nullptr;

UpdateList RebaseCache::find(const ExecutionState &state, ObjectState *os, const UpdateList &ul) {
  /* get dependent arrays */
  std::set<const Array *> arrays;
  os->getArrays(arrays);

  /* generate array ID's */
  Arrays ids;
  for (const Array *array : arrays) {
    ids.push_back(array->id);
  }

  const ExecutionState::History &h = state.getHistory();
  for (auto i = h.rbegin(); i != h.rend(); i++) {
    const RebaseID &rid = *i;
    for (RebaseInfo &ri : rebased) {
      if (ri.rid != rid) {
        continue;
      }

      std::vector<uint64_t> intersection;
      set_intersection(rid.arrays.begin(),
                       rid.arrays.end(),
                       ids.begin(),
                       ids.end(),
                       std::inserter(intersection, intersection.begin()));
      if (intersection.empty()) {
        continue;
      }

      UpdateList updates(nullptr, nullptr);

      auto i = ri.arrays.find(os->object->address);
      if (i == ri.arrays.end()) {
        updates = state.rewriteUL(ul, nullptr);
        ri.arrays.insert(std::make_pair(os->object->address, updates.root));
      } else {
        updates = state.rewriteUL(ul, i->second);
      }
      return updates;
    }
  }

  auto i = unrebased.find(os->object->address);
  if (i == unrebased.end()) {
    UpdateList updates = state.rewriteUL(ul, NULL);
    unrebased.insert(std::make_pair(os->object->address, updates.root));
    return updates;
  } else {
    return state.rewriteUL(ul, i->second);
  }
}

/***/

std::map<const Array *, const Array *> ExecutionState::globalRewriteCache;

uint64_t ExecutionState::globalArrayID = 0;

ExecutionState::ExecutionState(KFunction *kf, MemoryManager *memory) :
    memory(memory),
    arrayID(0),
    pc(kf->instructions),
    prevPC(pc),

    weight(1),
    depth(0),

    instsSinceCovNew(0),
    coveredNew(false),
    forkDisabled(false),
    ptreeNode(0),
    steppedInstructions(0),
    local_next_slot(0) {
  pushFrame(0, kf);
}

/* TODO: add rewritten constraints? */
ExecutionState::ExecutionState(const std::vector<ref<Expr> > &assumptions)
    : arrayID(0), constraints(assumptions), ptreeNode(0), local_next_slot(0) {}

ExecutionState::~ExecutionState() {
  for (unsigned int i=0; i<symbolics.size(); i++)
  {
    const MemoryObject *mo = symbolics[i].first;
    assert(mo->refCount > 0);
    mo->refCount--;
    if (mo->refCount == 0)
      delete mo;
  }

  for (auto cur_mergehandler: openMergeStack){
    cur_mergehandler->removeOpenState(this);
  }


  while (!stack.empty()) popFrame();
}

ExecutionState::ExecutionState(const ExecutionState& state):
    fnAliases(state.fnAliases),
    addressConstraints(state.addressConstraints),
    //cache(state.cache),
    memory(state.memory),
    history(state.history),
    arrayID(state.arrayID),
    rewriteCache(state.rewriteCache),
    pc(state.pc),
    prevPC(state.prevPC),
    stack(state.stack),
    incomingBBIndex(state.incomingBBIndex),

    addressSpace(state.addressSpace),
    constraints(state.constraints),

    queryCost(state.queryCost),
    weight(state.weight),
    depth(state.depth),

    pathOS(state.pathOS),
    symPathOS(state.symPathOS),

    instsSinceCovNew(state.instsSinceCovNew),
    coveredNew(state.coveredNew),
    forkDisabled(state.forkDisabled),
    coveredLines(state.coveredLines),
    ptreeNode(state.ptreeNode),
    symbolics(state.symbolics),
    arrayNames(state.arrayNames),
    openMergeStack(state.openMergeStack),
    steppedInstructions(state.steppedInstructions),
    rewrittenConstraints(state.rewrittenConstraints),
    local_next_slot(state.local_next_slot),
    rebaseConstraints(state.rebaseConstraints)
{
  for (unsigned int i=0; i<symbolics.size(); i++)
    symbolics[i].first->refCount++;

  for (auto cur_mergehandler: openMergeStack)
    cur_mergehandler->addOpenState(this);
}

ExecutionState *ExecutionState::branch() {
  depth++;

  ExecutionState *falseState = new ExecutionState(*this);
  falseState->coveredNew = false;
  falseState->coveredLines.clear();

  weight *= .5;
  falseState->weight -= weight;

  return falseState;
}

void ExecutionState::pushFrame(KInstIterator caller, KFunction *kf) {
  stack.push_back(StackFrame(caller,kf));
}

void ExecutionState::popFrame() {
  StackFrame &sf = stack.back();
  for (const MemoryObject* mo : sf.allocas) {
    if (UseLocalSymAddr) {
      /* TODO: avoid lookup... */
      const ObjectState *os = addressSpace.findObject(mo);
      for (const SubObject &o : os->getSubObjects()) {
        removeAddressConstraint(o.info.arrayID);
      }
    }
    unbindObject(mo);
  }
  stack.pop_back();
}

void ExecutionState::addSymbolic(const MemoryObject *mo, const Array *array) { 
  mo->refCount++;
  symbolics.push_back(std::make_pair(mo, array));
}

void ExecutionState::addConstraint(ref<Expr> e) {
  if (!constraints.mayHaveAddressConstraints() && !e->flag) {
    /* both PC and expression are free of address constraints... */
    /* TODO: something better than copy? */
    constraints.addConstraint(e);
    rewrittenConstraints = constraints;
  } else {
    ref<Expr> rewritten = unfold(e);
    rewrittenConstraints.addConstraint(rewritten);
    if (!isa<ConstantExpr>(rewritten)) {
      constraints.addConstraint(e);
    }
  }
}
///

std::string ExecutionState::getFnAlias(std::string fn) {
  std::map < std::string, std::string >::iterator it = fnAliases.find(fn);
  if (it != fnAliases.end())
    return it->second;
  else return "";
}

void ExecutionState::addFnAlias(std::string old_fn, std::string new_fn) {
  fnAliases[old_fn] = new_fn;
}

void ExecutionState::removeFnAlias(std::string fn) {
  fnAliases.erase(fn);
}

/**/

llvm::raw_ostream &klee::operator<<(llvm::raw_ostream &os, const MemoryMap &mm) {
  os << "{";
  MemoryMap::iterator it = mm.begin();
  MemoryMap::iterator ie = mm.end();
  if (it!=ie) {
    os << "MO" << it->first->id << ":" << it->second;
    for (++it; it!=ie; ++it)
      os << ", MO" << it->first->id << ":" << it->second;
  }
  os << "}";
  return os;
}

bool ExecutionState::merge(const ExecutionState &b) {
  if (DebugLogStateMerge)
    llvm::errs() << "-- attempting merge of A:" << this << " with B:" << &b
                 << "--\n";
  if (pc != b.pc)
    return false;

  // XXX is it even possible for these to differ? does it matter? probably
  // implies difference in object states?
  if (symbolics!=b.symbolics)
    return false;

  {
    std::vector<StackFrame>::const_iterator itA = stack.begin();
    std::vector<StackFrame>::const_iterator itB = b.stack.begin();
    while (itA!=stack.end() && itB!=b.stack.end()) {
      // XXX vaargs?
      if (itA->caller!=itB->caller || itA->kf!=itB->kf)
        return false;
      ++itA;
      ++itB;
    }
    if (itA!=stack.end() || itB!=b.stack.end())
      return false;
  }

  std::set< ref<Expr> > aConstraints(constraints.begin(), constraints.end());
  std::set< ref<Expr> > bConstraints(b.constraints.begin(), 
                                     b.constraints.end());
  std::set< ref<Expr> > commonConstraints, aSuffix, bSuffix;
  std::set_intersection(aConstraints.begin(), aConstraints.end(),
                        bConstraints.begin(), bConstraints.end(),
                        std::inserter(commonConstraints, commonConstraints.begin()));
  std::set_difference(aConstraints.begin(), aConstraints.end(),
                      commonConstraints.begin(), commonConstraints.end(),
                      std::inserter(aSuffix, aSuffix.end()));
  std::set_difference(bConstraints.begin(), bConstraints.end(),
                      commonConstraints.begin(), commonConstraints.end(),
                      std::inserter(bSuffix, bSuffix.end()));
  if (DebugLogStateMerge) {
    llvm::errs() << "\tconstraint prefix: [";
    for (std::set<ref<Expr> >::iterator it = commonConstraints.begin(),
                                        ie = commonConstraints.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
    llvm::errs() << "\tA suffix: [";
    for (std::set<ref<Expr> >::iterator it = aSuffix.begin(),
                                        ie = aSuffix.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
    llvm::errs() << "\tB suffix: [";
    for (std::set<ref<Expr> >::iterator it = bSuffix.begin(),
                                        ie = bSuffix.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
  }

  // We cannot merge if addresses would resolve differently in the
  // states. This means:
  // 
  // 1. Any objects created since the branch in either object must
  // have been free'd.
  //
  // 2. We cannot have free'd any pre-existing object in one state
  // and not the other

  if (DebugLogStateMerge) {
    llvm::errs() << "\tchecking object states\n";
    llvm::errs() << "A: " << addressSpace.objects << "\n";
    llvm::errs() << "B: " << b.addressSpace.objects << "\n";
  }
    
  std::set<const MemoryObject*> mutated;
  MemoryMap::iterator ai = addressSpace.objects.begin();
  MemoryMap::iterator bi = b.addressSpace.objects.begin();
  MemoryMap::iterator ae = addressSpace.objects.end();
  MemoryMap::iterator be = b.addressSpace.objects.end();
  for (; ai!=ae && bi!=be; ++ai, ++bi) {
    if (ai->first != bi->first) {
      if (DebugLogStateMerge) {
        if (ai->first < bi->first) {
          llvm::errs() << "\t\tB misses binding for: " << ai->first->id << "\n";
        } else {
          llvm::errs() << "\t\tA misses binding for: " << bi->first->id << "\n";
        }
      }
      return false;
    }
    if (ai->second != bi->second) {
      if (DebugLogStateMerge)
        llvm::errs() << "\t\tmutated: " << ai->first->id << "\n";
      mutated.insert(ai->first);
    }
  }
  if (ai!=ae || bi!=be) {
    if (DebugLogStateMerge)
      llvm::errs() << "\t\tmappings differ\n";
    return false;
  }
  
  // merge stack

  ref<Expr> inA = ConstantExpr::alloc(1, Expr::Bool);
  ref<Expr> inB = ConstantExpr::alloc(1, Expr::Bool);
  for (std::set< ref<Expr> >::iterator it = aSuffix.begin(), 
         ie = aSuffix.end(); it != ie; ++it)
    inA = AndExpr::create(inA, *it);
  for (std::set< ref<Expr> >::iterator it = bSuffix.begin(), 
         ie = bSuffix.end(); it != ie; ++it)
    inB = AndExpr::create(inB, *it);

  // XXX should we have a preference as to which predicate to use?
  // it seems like it can make a difference, even though logically
  // they must contradict each other and so inA => !inB

  std::vector<StackFrame>::iterator itA = stack.begin();
  std::vector<StackFrame>::const_iterator itB = b.stack.begin();
  for (; itA!=stack.end(); ++itA, ++itB) {
    StackFrame &af = *itA;
    const StackFrame &bf = *itB;
    for (unsigned i=0; i<af.kf->numRegisters; i++) {
      ref<Expr> &av = af.locals[i].value;
      const ref<Expr> &bv = bf.locals[i].value;
      if (av.isNull() || bv.isNull()) {
        // if one is null then by implication (we are at same pc)
        // we cannot reuse this local, so just ignore
      } else {
        av = SelectExpr::create(inA, av, bv);
      }
    }
  }

  for (std::set<const MemoryObject*>::iterator it = mutated.begin(), 
         ie = mutated.end(); it != ie; ++it) {
    const MemoryObject *mo = *it;
    const ObjectState *os = addressSpace.findObject(mo);
    const ObjectState *otherOS = b.addressSpace.findObject(mo);
    assert(os && !os->readOnly && 
           "objects mutated but not writable in merging state");
    assert(otherOS);

    ObjectState *wos = addressSpace.getWriteable(mo, os);
    for (unsigned i=0; i<mo->size; i++) {
      ref<Expr> av = wos->read8(i);
      ref<Expr> bv = otherOS->read8(i);
      wos->write(i, SelectExpr::create(inA, av, bv));
    }
  }

  /* export some API for clearing constraints? */
  constraints = ConstraintManager();
  rewrittenConstraints = ConstraintManager();
  for (std::set< ref<Expr> >::iterator it = commonConstraints.begin(), 
         ie = commonConstraints.end(); it != ie; ++it) {
    addConstraint(*it);
  }
  addConstraint(OrExpr::create(inA, inB));

  return true;
}

void ExecutionState::dumpStack(llvm::raw_ostream &out) const {
  unsigned idx = 0;
  const KInstruction *target = prevPC;
  for (ExecutionState::stack_ty::const_reverse_iterator
         it = stack.rbegin(), ie = stack.rend();
       it != ie; ++it) {
    const StackFrame &sf = *it;
    Function *f = sf.kf->function;
    const InstructionInfo &ii = *target->info;
    out << "\t#" << idx++;
    std::stringstream AssStream;
    AssStream << std::setw(8) << std::setfill('0') << ii.assemblyLine;
    out << AssStream.str();
    out << " in " << f->getName().str() << " (";
    // Yawn, we could go up and print varargs if we wanted to.
    unsigned index = 0;
    for (Function::arg_iterator ai = f->arg_begin(), ae = f->arg_end();
         ai != ae; ++ai) {
      if (ai!=f->arg_begin()) out << ", ";

      out << ai->getName().str();
      // XXX should go through function
      ref<Expr> value = sf.locals[sf.kf->getArgRegister(index++)].value;
      if (value.get() && isa<ConstantExpr>(value))
        out << "=" << value;
    }
    out << ")";
    if (ii.file != "")
      out << " at " << ii.file << ":" << ii.line;
    out << "\n";
    target = sf.caller;
  }
}

void ExecutionState::unbindObject(const MemoryObject *mo) {
  addressSpace.unbindObject(mo);
}

void ExecutionState::addAddressConstraint(uint64_t id,
                                          uint64_t address,
                                          ref<Expr> alpha) {
  if (addressConstraints.find(id) != addressConstraints.end()) {
    assert(0);
  }

  ref<AddressRecord> record = new AddressRecord(address, alpha);
  addressConstraints[id] = record;
  //cache[alpha->hash()] = record;
}

void ExecutionState::updateAddressConstraint(uint64_t id,
                                             uint64_t address) {
  auto i = addressConstraints.find(id);
  if (i == addressConstraints.end()) {
    assert(0);
  }

  ref<AddressRecord> old = i->second;
  ref<AddressRecord> record = new AddressRecord(address, old->alpha);
  addressConstraints[id] = record;
  //cache[alpha->hash()] = record;
}

bool ExecutionState::hasAddressConstraint(uint64_t id) {
  return addressConstraints.find(id) != addressConstraints.end();
}

ref<AddressRecord> ExecutionState::getAddressConstraint(uint64_t id) const {
  auto i = addressConstraints.find(id);
  if (i == addressConstraints.end()) {
    assert(false);
  } else {
    return i->second;
  }
}

void ExecutionState::removeAddressConstraint(uint64_t id) {
  auto i = addressConstraints.find(id);
  if (i == addressConstraints.end()) {
    assert(false);
  }
  addressConstraints.erase(i);
}

ref<Expr> ExecutionState::build(ref<Expr> e) const {
  /* collect dependencies */
  AddressArrayCollector collector;
  collector.visit(e);

  ref<Expr> all = ConstantExpr::create(1, Expr::Bool);
  for (uint64_t id : collector.ids) {
    ref<AddressRecord> ar = getAddressConstraint(id);
    ref<Expr> eq = ar->constraint;
    all = AndExpr::create(all, eq);
  }

  return all;
}

ref<Expr> ExecutionState::build(std::vector<ref<Expr>> &es) const {
  std::set<uint64_t> all_arrays;

  for (ref<Expr> e : es) {
    /* collect dependencies */
    AddressArrayCollector collector;
    collector.visit(e);
    for (uint64_t id : collector.ids) {
      all_arrays.insert(id);
    }
  }

  ref<Expr> all = ConstantExpr::create(1, Expr::Bool);
  for (uint64_t id : all_arrays) {
    ref<AddressRecord> ar = getAddressConstraint(id);
    ref<Expr> eq = ar->constraint;
    all = AndExpr::create(all, eq);
  }

  return all;
}

void ExecutionState::dumpAddressConstraints() const {
  for (auto &i : addressConstraints) {
    ref<AddressRecord> ar = i.second;
    errs() << i.first << " " << ar->constraint << "\n";
  }
}

void ExecutionState::computeRewrittenConstraints() {
  rewrittenConstraints.clear();
  for (ref<Expr> e : constraints) {
    ref<Expr> rewritten = unfold(e);
    rewrittenConstraints.addConstraint(rewritten);
  }
}

ref<Expr> ExecutionState::unfold(const ref<Expr> address) const {
  if (!address->flag) {
    /* may not contain address expressions */
    return address;
  }

  /* a common case where the address is just (A) */
  if (isa<ConcatExpr>(address)) {
    ConcatExpr *concat = dyn_cast<ConcatExpr>(address);
    /* TODO: what if it's a partial concat (7 bytes...)? */
    if (concat->isPureAddress) {
      ReadExpr *re = dyn_cast<ReadExpr>(concat->getLeft());
      if (re && re->updates.root->isAddressArray) {
        ref<AddressRecord> ar = getAddressConstraint(re->updates.root->id);
        return ar->address;
      }
    }
  }

  /* a common case where the address is (C + A) */
  if (isa<AddExpr>(address)) {
    AddExpr *add = dyn_cast<AddExpr>(address);
    ConstantExpr *leftCE = dyn_cast<ConstantExpr>(add->left);
    if (leftCE && isa<ConcatExpr>(add->right)) {
      ConcatExpr *concat = dyn_cast<ConcatExpr>(add->right);
      if (concat->isPureAddress) {
        ReadExpr *re = dyn_cast<ReadExpr>(concat->getLeft());
        if (re && re->updates.root->isAddressArray) {
          /* TODO: use directly ConstantExpr? */
          ref<AddressRecord> ar = getAddressConstraint(re->updates.root->id);
          return AddExpr::create(ar->address, leftCE);
        }
      }
    }
  }

  /* a common case where we have (A == 0) */
  if (isa<EqExpr>(address)) {
    EqExpr *eq = dyn_cast<EqExpr>(address);
    ConstantExpr *leftCE = dyn_cast<ConstantExpr>(eq->left);
    if (leftCE && leftCE->getZExtValue() == 0) {
      if (isa<ConcatExpr>(eq->right)) {
        ConcatExpr *concat = dyn_cast<ConcatExpr>(eq->right);
        if (concat->isPureAddress) {
          return ConstantExpr::alloc(0, Expr::Bool);
        }
      }
    }
  }

  //AddressUnfolder unfolder(*this);
  //ref<Expr> unfolded = unfolder.visit(address);

  //ReadExprOptimizer optimizer(*this, unfolder.arrays);
  //ref<Expr> optimized = optimizer.visit(unfolded);

  SubstVisitor subst(*this);
  ref<Expr> optimized = subst.visit(address);

  assert(!optimized->flag);
  return optimized;
}

UpdateList ExecutionState::rewriteUL(const UpdateList &ul, const Array *array) const {
  struct WriteUpdate {
    ref<Expr> index;
    ref<Expr> value;
    bool needed;
    WriteUpdate(ref<Expr> index, ref<Expr> value, bool needed) :
      index(index), value(value), needed(needed) {

    }
  };

  std::vector<ref<ConstantExpr>> constants(ul.root->size);
  for (unsigned i = 0; i < ul.root->size; i++) {
    constants[i] = ul.root->constantValues[i];
  }

  std::list<WriteUpdate> writes;
  for (const UpdateNode *un = ul.head.get(); un; un = un->next.get()) {
    ref<Expr> index = un->index;
    ref<Expr> value = un->value;
    writes.push_front(WriteUpdate(index, value, true));
  }

  bool canSetConstants = true;
  for (WriteUpdate &u : writes) {
    ref<Expr> index = u.index;
    ref<Expr> value = u.value;
    if (canSetConstants && isa<ConstantExpr>(index)) {
      uint64_t offset = dyn_cast<ConstantExpr>(index)->getZExtValue();
      if (isa<ConstantExpr>(value)) {
        constants[offset] = dyn_cast<ConstantExpr>(value);
        u.needed = false;
      }
    } else {
      canSetConstants = false;
    }
  }

  ///* TODO: use a list? */
  //std::vector<std::pair<ref<Expr>, ref<Expr>>> writes;
  //for (const UpdateNode *un = ul.head; un; un = un->next) {
  //  ref<Expr> index = un->index;
  //  ref<Expr> value = un->value;
  //  if (isa<ConstantExpr>(index)) {
  //    /* the index is concrete */
  //    uint64_t offset = dyn_cast<ConstantExpr>(index)->getZExtValue();
  //    if (!wasOverwritten[offset]) {
  //      if (isa<ConstantExpr>(value)) {
  //        /* the value is concrete */
  //        constants[offset] = dyn_cast<ConstantExpr>(value);
  //      } else {
  //        /* the value is symbolic */
  //        writes.push_back(std::make_pair(index, value));
  //      }
  //      wasOverwritten[offset] = true;
  //    }
  //  } else {
  //    /* the index is symbolic */
  //    writes.push_back(std::make_pair(index, value));
  //  }
  //}

  bool reusing = (array != nullptr);
  if (!array) {
    static unsigned rwid = 0;
    std::string name = "rewritten_arr" + llvm::utostr(++rwid);
    ArrayCache *arrayCache = memory->getArrayCache();
    array = arrayCache->CreateArray(name,
                                    ul.root->size,
                                    &constants[0],
                                    &constants[0] + constants.size());
    klee_message("new array: %s (from %s)",
                 array->getName().data(),
                 ul.root->getName().data());
  }

  UpdateList updates = UpdateList(array, 0);

  if (reusing) {
    for (unsigned int i = 0; i < array->size; i++) {
      ref<ConstantExpr> initialized = array->constantValues[i];
      if (initialized->compareContents(*constants[i].get()) != 0) {
        updates.extend(ConstantExpr::create(i, Expr::Int32), constants[i]);
      }
    }
  }

  for (WriteUpdate &u : writes) {
    /* TODO: try to implement with two lists to avoid re-scan... */
    if (u.needed) {
      updates.extend(u.index, u.value);
    }
  }

  return updates;
}

UpdateList ExecutionState::initializeRewrittenUL(ObjectState *os,
                                                 const UpdateList &ul) const {
  if (!ReuseArrays) {
    return rewriteUL(ul, NULL);
  }

  RebaseCache *rc = RebaseCache::getRebaseCache();
  return rc->find(*this, os, ul);
}

/* TODO: move to AddressSpace? */
bool ExecutionState::findRewrittenObject(const UpdateList &ul,
                                         const MemoryObject *&mo,
                                         ObjectState *&os) const {
  for (auto i : addressSpace.objects) {
    mo = i.first;
    os = i.second;
    if (os->updates.root == ul.root) {
      return true;
    }
  }

  for (auto i : addressSpace.rewrittenObjects) {
    mo = i.first;
    os = i.second;
    if (os->updates.root == ul.root) {
      return true;
    }
  }

  for (auto i : addressSpace.deallocatedObjects) {
    mo = i.first;
    os = i.second;
    if (os->updates.root == ul.root) {
      return true;
    }
  }

  return false;
}

UpdateList ExecutionState::getRewrittenUL(const UpdateList &ul) const {
  assert(ul.root);
  if (ul.head.isNull()) {
    /* if the list is empty, nothing to rewrite... */
    return ul;
  }

  const Array *rewritten = getRewrittenArray(ul.root);
  if (!rewritten) {
    UpdateList updates = rewriteUL(ul, nullptr);
    ExecutionState *writable = const_cast<ExecutionState *>(this);
    writable->updateRewrittenArray(ul.root, updates.root);
    return updates;
  } else {
    return rewriteUL(ul, rewritten);
  }

  ///* TODO: add cache? */
  //ObjectState *os = nullptr;
  //const MemoryObject *mo = nullptr;
  //bool found = findRewrittenObject(ul, mo, os);

  ///* the object must be there... */
  //assert(found);
  ///* the object must hold the latest updates */
  //assert(os->updates.getSize() >= ul.getSize());

  //ExecutionState *writable = const_cast<ExecutionState *>(this);
  //writable->addressSpace.addRewrittenObject(mo, os);

  //if (!os->rewrittenUpdates.root) {
  //  UpdateList updates = initializeRewrittenUL(os, ul);
  //  os->rewrittenUpdates = updates;
  //  os->pulledUpdates = ul.getSize();
  //  os->minUpdates = ul.getSize();
  //  return os->rewrittenUpdates;
  //} else {
  //  return rewriteUL(ul, os->rewrittenUpdates.root);
  //}

  //if (!os->rewrittenUpdates.root) {
  //  UpdateList updates = initializeRewrittenUL(os, ul);
  //  os->rewrittenUpdates = updates;
  //  os->pulledUpdates = ul.getSize();
  //} else {
  //  UpdateList updates = rewriteUL(ul, os->rewrittenUpdates.root);
  //  os->rewrittenUpdates = updates;
  //}

  //ExecutionState *writable = const_cast<ExecutionState *>(this);
  //writable->addressSpace.addRewrittenObject(mo, os);

  //return os->rewrittenUpdates;

  //if (!os->rewrittenUpdates.root) {
  //  UpdateList updates = initializeRewrittenUL(os, ul);
  //  os->rewrittenUpdates = updates;
  //  os->pulledUpdates = ul.getSize();
  //} else {
  //  if (os->pulledUpdates < ul.getSize()) {
  //    /* collect the missing nodes */
  //    std::list<const UpdateNode *> nodes;
  //    unsigned int j = 0;
  //    for (const UpdateNode *n = ul.head; n; n = n->next) {
  //      if (j >= (ul.getSize() - os->pulledUpdates)) {
  //        break;
  //      }
  //      nodes.push_front(n);
  //      j++;
  //    }

  //    /* add the missing updates */
  //    for (const UpdateNode *n : nodes) {
  //      ref<Expr> index = addressSpace.unfold(*this, n->index);
  //      ref<Expr> value = addressSpace.unfold(*this, n->value);
  //      os->rewrittenUpdates.extend(index, value);
  //    }
  //    os->pulledUpdates = ul.getSize();
  //  }
  //}

  //ExecutionState *writable = const_cast<ExecutionState *>(this);
  //writable->addressSpace.addRewrittenObject(mo, os);

  ///* the number of updates which were set as initial values */
  //size_t constants = os->pulledUpdates - os->rewrittenUpdates.getSize();
  //if (ul.getSize() < constants) {
  //  /* TODO: override with updates? */
  //  assert(0);
  //}

  //const UpdateNode *head = nullptr;
  //for (const UpdateNode *n = os->rewrittenUpdates.head; n; n = n->next) {
  //  if (n->getSize() == (ul.getSize() - constants)) {
  //      head = n;
  //      break;
  //  }
  //}

  //return UpdateList(os->rewrittenUpdates.root, head);
}

/* TODO: we don't need to rewrite everything... */
void ExecutionState::updateRewrittenObjects() {
  for (auto i : addressSpace.rewrittenObjects) {
    ObjectState *os = addressSpace.getWriteable(i.first, i.second);
    os->rewrittenUpdates = UpdateList(0, 0);
    os->pulledUpdates = 0;
    os->minUpdates = 0;
  }
  //rewriteCache.clear();
}

uint64_t ExecutionState::allocateArrayID() {
  if (UseGlobalID) {
    return globalArrayID++;
  } else {
    return arrayID++;
  }
}

const Array *ExecutionState::getRewrittenArray(const Array *array) const {
  if (UseGlobalRewriteCache) {
    auto i = globalRewriteCache.find(array);
    if (i == globalRewriteCache.end()) {
      return nullptr;
    } else {
      return i->second;
    }
  } else {
    auto i = rewriteCache.find(array);
    if (i == rewriteCache.end()) {
      return nullptr;
    } else {
      return i->second;
    }
  }
}

void ExecutionState::updateRewrittenArray(const Array *array,
                                          const Array *rewritten) {
  if (UseGlobalRewriteCache) {
    globalRewriteCache[array] = rewritten;
  } else {
    rewriteCache[array] = rewritten;
  }
}

void ExecutionState::addRebaseConstraints(ref<Expr> segment,
                                          ref<Expr> object,
                                          uint64_t offset) {
  ref<Expr> target = AddExpr::create(segment, ConstantExpr::create(offset, Expr::Int64));
  ref<Expr> eq = EqExpr::create(object, target);
  rebaseConstraints.push_back(eq);
}

AllocationContext ExecutionState::getAC() const {
  uint64_t h = 0;
  unsigned int j = 0;

  if (stack.size() <= 2) {
    /* TODO: check what happens here... */
    return AllocationContext();
  }

  for (auto i = stack.rbegin(); i != stack.rend(); i++) {
    if (j == UseKContext) {
      break;
    }

    const StackFrame &sf = *i;
    if (!sf.caller) {
      break;
    }

    h += sf.caller->info->id;
    j++;
  }

  h += prevPC->info->id;

  return AllocationContext(h);
}

/* TODO: check flag? */
//ExprVisitor::Action AddressUnfolder::visitConcat(const ConcatExpr &e) {
//  if (!e.flag) {
//    return Action::skipChildren();
//  }
//
//  auto i = state.getCache().find(e.hash());
//  if (i != state.getCache().end()) {
//    return Action::changeTo(i->second->address);
//  }
//
//  return Action::doChildren();
//}

ExprVisitor::Action AddressUnfolder::visitConcat(const ConcatExpr &e) {
  ref<ReadExpr> re = dyn_cast<ReadExpr>(e.getLeft());
  if (!re.isNull() && re->updates.root->isAddressArray) {
    assert(isa<ConstantExpr>(re->index));
    ref<AddressRecord> ar = state.getAddressConstraint(re->updates.root->id);
    return Action::changeTo(ar->address);
  }

  return Action::doChildren();
}

ExprVisitor::Action AddressUnfolder::visitRead(const ReadExpr &e) {
  if (!e.flag) {
    return Action::skipChildren();
  }

  if (e.updates.root->isAddressArray) {
    ref<ConstantExpr> index = dyn_cast<ConstantExpr>(e.index);
    if (index.isNull()) {
      /* should not happen... */
      assert(false);
    }

    ref<AddressRecord> ar = state.getAddressConstraint(e.updates.root->id);
    return Action::changeTo(ar->bytes[index->getZExtValue()]);
  }

  bool changed = false;

  /* rewrite index (if needed...) */
  assert(e.updates.root);
  ref<Expr> index = e.index;
  if (e.index->flag) {
    index = visit(index);
    changed = true;
  }

  /* rewrite update list (if needed...) */
  UpdateList updates = e.updates;
  if (e.ulflag) {
    updates = UpdateList(e.updates.root, nullptr);
    std::list<const UpdateNode *> nodes;
    for (const UpdateNode *n = e.updates.head.get(); n; n = n->next.get()) {
      nodes.push_front(n);
    }
    for (const UpdateNode *n : nodes) {
      ref<Expr> index = visit(n->index);
      ref<Expr> value = visit(n->value);
      updates.extend(index, value);
    }
    changed = true;
    arrays.insert(e.updates.root->id);
  }

  if (changed) {
    /* TODO: do some caching? */
    return Action::changeTo(ReadExpr::create(updates, index));
  }

  return Action::doChildren();
}

ExprVisitor::Action ReadExprOptimizer::visitRead(const ReadExpr &e) {
  assert(!e.flag && !e.ulflag);

  UpdateList updates = UpdateList(nullptr, nullptr);
  if (arrays.find(e.updates.root->id) != arrays.end()) {
    UpdateList rw = UpdateList(e.updates.root, nullptr);

    std::list<const UpdateNode *> nodes;
    for (const UpdateNode *n = e.updates.head.get(); n; n = n->next.get()) {
      nodes.push_front(n);
    }
    for (const UpdateNode *n : nodes) {
      ref<Expr> index = visit(n->index);
      ref<Expr> value = visit(n->value);
      rw.extend(index, value);
    }
    updates = state.getRewrittenUL(rw);
  } else {
    updates = e.updates;
  }

  ref<Expr> index = visit(e.index);
  return Action::changeTo(ReadExpr::create(updates, index));
}

ExprVisitor::Action SubstVisitor::visitConcat(const ConcatExpr &e) {
  if (e.isPureAddress) {
    ReadExpr *re = dyn_cast<ReadExpr>(e.getLeft());
    if (re && re->updates.root->isAddressArray) {
      assert(isa<ConstantExpr>(re->index));
      ref<AddressRecord> ar = state.getAddressConstraint(re->updates.root->id);
      return Action::changeTo(ar->address);
    } else {
      assert(0);
    }
  }
  return Action::doChildren();
}

ExprVisitor::Action SubstVisitor::visitRead(const ReadExpr &e) {
  if (!e.flag) {
    return Action::skipChildren();
  }

  if (e.updates.root->isAddressArray) {
    ref<ConstantExpr> index = dyn_cast<ConstantExpr>(e.index);
    if (index.isNull()) {
      /* should not happen... */
      assert(false);
    }

    ref<AddressRecord> ar = state.getAddressConstraint(e.updates.root->id);
    return Action::changeTo(ar->bytes[index->getZExtValue()]);
  }

  bool changed = false;

  /* rewrite index (if needed...) */
  assert(e.updates.root);
  ref<Expr> index = e.index;
  if (e.index->flag) {
    index = visit(index);
    changed = true;
  }

  /* rewrite update list (if needed...) */
  UpdateList updates = UpdateList(nullptr, nullptr);
  if (e.ulflag) {
    auto i = cache.find(e.updates);
    if (i == cache.end()) {
      updates = UpdateList(e.updates.root, nullptr);
      std::list<const UpdateNode *> nodes;
      for (const UpdateNode *n = e.updates.head.get(); n; n = n->next.get()) {
        nodes.push_front(n);
      }
      for (const UpdateNode *n : nodes) {
        ref<Expr> index = visit(n->index);
        ref<Expr> value = visit(n->value);
        updates.extend(index, value);
      }
      updates = state.getRewrittenUL(updates);
      cache.insert(std::make_pair(e.updates, updates));
    } else {
      updates = i->second;
    }
    changed = true;
  } else {
    updates = e.updates;
  }

  if (changed) {
    /* TODO: do some caching? */
    return Action::changeTo(ReadExpr::create(updates, index));
  }

  return Action::doChildren();
}
