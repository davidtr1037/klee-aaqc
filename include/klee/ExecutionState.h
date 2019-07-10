//===-- ExecutionState.h ----------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_EXECUTIONSTATE_H
#define KLEE_EXECUTIONSTATE_H

#include "klee/Constraints.h"
#include "klee/Expr.h"
#include "klee/Internal/ADT/TreeStream.h"
#include "klee/Internal/System/Time.h"
#include "klee/MergeHandler.h"
#include "klee/util/ExprVisitor.h"

// FIXME: We do not want to be exposing these? :(
#include "../../lib/Core/AddressSpace.h"
#include "../../lib/Core/MemoryManager.h"
#include "klee/Internal/Module/KInstIterator.h"

#include "llvm/Support/CommandLine.h"

#include <map>
#include <set>
#include <vector>

namespace klee {
class Array;
class CallPathNode;
struct Cell;
struct KFunction;
struct KInstruction;
class MemoryObject;
class PTreeNode;
struct InstructionInfo;

extern llvm::cl::opt<bool> UseLocalSymAddr;

llvm::raw_ostream &operator<<(llvm::raw_ostream &os, const MemoryMap &mm);

struct StackFrame {
  KInstIterator caller;
  KFunction *kf;
  CallPathNode *callPathNode;

  std::vector<const MemoryObject *> allocas;
  Cell *locals;

  /// Minimum distance to an uncovered instruction once the function
  /// returns. This is not a good place for this but is used to
  /// quickly compute the context sensitive minimum distance to an
  /// uncovered instruction. This value is updated by the StatsTracker
  /// periodically.
  unsigned minDistToUncoveredOnReturn;

  // For vararg functions: arguments not passed via parameter are
  // stored (packed tightly) in a local (alloca) memory object. This
  // is set up to match the way the front-end generates vaarg code (it
  // does not pass vaarg through as expected). VACopy is lowered inside
  // of intrinsic lowering.
  MemoryObject *varargs;

  StackFrame(KInstIterator caller, KFunction *kf);
  StackFrame(const StackFrame &s);
  ~StackFrame();
};

struct AddressRecord {
    const ref<ConstantExpr> address;
    const std::vector<ref<ConstantExpr>> bytes;
    const ref<Expr> constraint;
    mutable unsigned refCount;

    AddressRecord(const ref<ConstantExpr> address,
                  const std::vector<ref<ConstantExpr>> bytes,
                  const ref<Expr> constraint) :
      address(address),
      bytes(bytes),
      constraint(constraint),
      refCount(0) {

    }
};

struct RebaseID {
  const InstructionInfo *info;
  size_t size;

  RebaseID() :
    info(nullptr), size(0) {

  }

  RebaseID(const InstructionInfo *info, size_t size) :
    info(info), size(size) {

  }
};

struct RebaseInfo {
  RebaseID rid;
  const MemoryObject *mo;
  ObjectHolder oh;

  RebaseInfo() : mo(nullptr) {

  }

  RebaseInfo(RebaseID &rid, const MemoryObject *mo, ObjectHolder oh) :
    rid(rid), mo(mo), oh(oh) {

  }
};

class RebaseCache {

public:

  RebaseCache(): refCount(0) {

  }

  static RebaseCache *getRebaseCache() {
    if (!instance) {
      instance = new RebaseCache();
    }
    return instance;
  }

  unsigned int refCount;
  std::vector<RebaseInfo> rebased;

  static RebaseCache *instance;
};

/// @brief ExecutionState representing a path under exploration
class ExecutionState {
public:
  typedef std::vector<StackFrame> stack_ty;
  /* the key is an array identifier */
  typedef std::map<uint64_t, ref<AddressRecord>> AddressConstraints;
  /* TODO: change the key to ref<Expr>? */
  typedef std::map<unsigned, ref<AddressRecord>> Cache;
  /* .. */
  ref<RebaseCache> rebaseCache;

private:
  // unsupported, use copy constructor
  ExecutionState &operator=(const ExecutionState &);

  std::map<std::string, std::string> fnAliases;

  AddressConstraints addressConstraints;

  Cache cache;

  MemoryManager *memory;

public:
  // Execution - Control Flow specific

  /// @brief Pointer to instruction to be executed after the current
  /// instruction
  KInstIterator pc;

  /// @brief Pointer to instruction which is currently executed
  KInstIterator prevPC;

  /// @brief Stack representing the current instruction stream
  stack_ty stack;

  /// @brief Remember from which Basic Block control flow arrived
  /// (i.e. to select the right phi values)
  unsigned incomingBBIndex;

  // Overall state of the state - Data specific

  /// @brief Address space used by this state (e.g. Global and Heap)
  AddressSpace addressSpace;

  /// @brief Constraints collected so far
  ConstraintManager constraints;

  /// Statistics and information

  /// @brief Costs for all queries issued for this state, in seconds
  mutable time::Span queryCost;

  /// @brief Weight assigned for importance of this state.  Can be
  /// used for searchers to decide what paths to explore
  double weight;

  /// @brief Exploration depth, i.e., number of times KLEE branched for this state
  unsigned depth;

  /// @brief History of complete path: represents branches taken to
  /// reach/create this state (both concrete and symbolic)
  TreeOStream pathOS;

  /// @brief History of symbolic path: represents symbolic branches
  /// taken to reach/create this state
  TreeOStream symPathOS;

  /// @brief Counts how many instructions were executed since the last new
  /// instruction was covered.
  unsigned instsSinceCovNew;

  /// @brief Whether a new instruction was covered in this state
  bool coveredNew;

  /// @brief Disables forking for this state. Set by user code
  bool forkDisabled;

  /// @brief Set containing which lines in which files are covered by this state
  std::map<const std::string *, std::set<unsigned> > coveredLines;

  /// @brief Pointer to the process tree of the current state
  PTreeNode *ptreeNode;

  /// @brief Ordered list of symbolics: used to generate test cases.
  //
  // FIXME: Move to a shared list structure (not critical).
  std::vector<std::pair<const MemoryObject *, const Array *> > symbolics;

  /// @brief Set of used array names for this state.  Used to avoid collisions.
  std::set<std::string> arrayNames;

  std::string getFnAlias(std::string fn);
  void addFnAlias(std::string old_fn, std::string new_fn);
  void removeFnAlias(std::string fn);

  // The objects handling the klee_open_merge calls this state ran through
  std::vector<ref<MergeHandler> > openMergeStack;

  // The numbers of times this state has run through Executor::stepInstruction
  std::uint64_t steppedInstructions;

  /* TODO: add docs */
  ConstraintManager rewrittenConstraints;

private:
  ExecutionState() : ptreeNode(0) {}

public:
  ExecutionState(KFunction *kf, MemoryManager *memory);

  // XXX total hack, just used to make a state so solver can
  // use on structure
  ExecutionState(const std::vector<ref<Expr> > &assumptions);

  ExecutionState(const ExecutionState &state);

  ~ExecutionState();

  ExecutionState *branch();

  void pushFrame(KInstIterator caller, KFunction *kf);
  void popFrame();

  void addSymbolic(const MemoryObject *mo, const Array *array);
  void addConstraint(ref<Expr> e);

  bool merge(const ExecutionState &b);
  void dumpStack(llvm::raw_ostream &out) const;

  void unbindObject(const MemoryObject *mo);

  void addAddressConstraint(uint64_t id,
                            uint64_t address,
                            ref<Expr> e);

  ref<AddressRecord> getAddressConstraint(uint64_t id) const;

  void removeAddressConstraint(uint64_t id);

  const AddressConstraints &getAddressConstraints() const {
    return addressConstraints;
  }

  const Cache &getCache() const {
    return cache;
  }

  ref<Expr> build(ref<Expr> e) const;

  ref<Expr> build(std::vector<ref<Expr>> &es) const;

  void dumpAddressConstraints() const;

  void computeRewrittenConstraints();

  UpdateList rewriteUL(const UpdateList &ul, bool &changed) const;

  bool findRewrittenObject(const UpdateList &ul, const MemoryObject *&mo, ObjectState *&os) const;

  UpdateList getRewrittenUL(const UpdateList &ul, bool &changed) const;

  void updateRewrittenObjects();
};

class AddressUnfolder : public ExprVisitor {
protected:

  ExprVisitor::Action visitConcat(const ConcatExpr &e);

  ExprVisitor::Action visitRead(const ReadExpr &e);

public:

  AddressUnfolder(const ExecutionState &state) : state(state) {

  }

  const ExecutionState &state;
};
}

#endif
