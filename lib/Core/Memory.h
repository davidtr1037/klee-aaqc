//===-- Memory.h ------------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_MEMORY_H
#define KLEE_MEMORY_H

#include "Context.h"
#include "TimingSolver.h"
#include "AllocationContext.h"
#include "klee/Expr.h"

#include "llvm/ADT/StringExtras.h"

#include <vector>
#include <string>

namespace llvm {
  class Value;
}

namespace klee {

class BitArray;
class MemoryManager;
class Solver;
class ArrayCache;

struct SymbolicAddressInfo {
  uint64_t arrayID;
  ref<Expr> address;

  SymbolicAddressInfo() :
    arrayID(0), address(nullptr) {

  }
};

class MemoryObject {
  friend class STPBuilder;
  friend class ObjectState;
  friend class ExecutionState;

private:
  static int counter;
  mutable unsigned refCount;

public:
  unsigned id;
  uint64_t address;

  /// size in bytes
  unsigned size;
  mutable std::string name;

  bool isLocal;
  mutable bool isGlobal;
  bool isFixed;

  bool isUserSpecified;

  MemoryManager *parent;

  /// "Location" for which this memory object was allocated. This
  /// should be either the allocating instruction or the global object
  /// it was allocated for (or whatever else makes sense).
  const llvm::Value *allocSite;
  
  /// A list of boolean expressions the user has requested be true of
  /// a counterexample. Mutable since we play a little fast and loose
  /// with allowing it to be added to during execution (although
  /// should sensibly be only at creation time).
  mutable std::vector< ref<Expr> > cexPreferences;

  /* TODO: ... */
  mutable SymbolicAddressInfo sainfo;

  /* TODO: ... */
  mutable AllocationContext ac;

  int canFree;

  // DO NOT IMPLEMENT
  MemoryObject(const MemoryObject &b);
  MemoryObject &operator=(const MemoryObject &b);

public:
  // XXX this is just a temp hack, should be removed
  explicit
  MemoryObject(uint64_t _address) 
    : refCount(0),
      id(counter++), 
      address(_address),
      size(0),
      isFixed(true),
      parent(NULL),
      allocSite(0),
      canFree(false) {
  }

  MemoryObject(uint64_t _address, unsigned _size, 
               bool _isLocal, bool _isGlobal, bool _isFixed, bool canFree,
               const llvm::Value *_allocSite,
               MemoryManager *_parent)
    : refCount(0), 
      id(counter++),
      address(_address),
      size(_size),
      name("unnamed"),
      isLocal(_isLocal),
      isGlobal(_isGlobal),
      isFixed(_isFixed),
      isUserSpecified(false),
      parent(_parent), 
      allocSite(_allocSite),
      canFree(canFree) {
  }

  ~MemoryObject();

  /// Get an identifying string for this allocation.
  void getAllocInfo(std::string &result) const;

  void setName(std::string name) const {
    this->name = name;
  }

  bool hasSymbolicAddress() const {
    return !sainfo.address.isNull();
  }

  ref<Expr> getBaseExpr() const {
    if (!hasSymbolicAddress()) {
      return ConstantExpr::create(address, Context::get().getPointerWidth());
    } else {
      return sainfo.address;
    }
  }
  ref<ConstantExpr> getSizeExpr() const { 
    return ConstantExpr::create(size, Context::get().getPointerWidth());
  }
  ref<Expr> getOffsetExpr(ref<Expr> pointer) const {
    return SubExpr::create(pointer, getBaseExpr());
  }
  ref<Expr> getOptimizedOffsetExpr(ref<Expr> pointer, ref<Expr> unfolded) const {
    if (pointer->isSymbolic) {
      /* can't be unfolded to a constant */
      return getOffsetExpr(pointer);
    } else {
      ref<ConstantExpr> c = dyn_cast<ConstantExpr>(unfolded);
      if (c.isNull()) {
        /* should not happen... */
        assert(0);
      } else {
        return ConstantExpr::create(c->getZExtValue() - address, Context::get().getPointerWidth());
      }
    }
  }
  ref<Expr> getBoundsCheckPointer(ref<Expr> pointer) const {
    return getBoundsCheckOffset(getOffsetExpr(pointer));
  }
  ref<Expr> getBoundsCheckPointer(ref<Expr> pointer, unsigned bytes) const {
    return getBoundsCheckOffset(getOffsetExpr(pointer), bytes);
  }

  ref<Expr> getBoundsCheckOffset(ref<Expr> offset) const {
    if (size==0) {
      return EqExpr::create(offset, 
                            ConstantExpr::alloc(0, Context::get().getPointerWidth()));
    } else {
      return UltExpr::create(offset, getSizeExpr());
    }
  }
  ref<Expr> getBoundsCheckOffset(ref<Expr> offset, unsigned bytes) const {
    if (bytes<=size) {
      return UltExpr::create(offset, 
                             ConstantExpr::alloc(size - bytes + 1, 
                                                 Context::get().getPointerWidth()));
    } else {
      return ConstantExpr::alloc(0, Expr::Bool);
    }
  }
};

struct SubObject {
  unsigned int offset;
  uint64_t size;
  SymbolicAddressInfo info;

  SubObject(unsigned offset, uint64_t size, const SymbolicAddressInfo &info) :
    offset(offset), size(size), info(info)
  {

  }
};

class ObjectState {
private:
  friend class AddressSpace;
  /* TODO: remove... */
  friend class ExecutionState;
  friend class RebaseCache;
  unsigned copyOnWriteOwner; // exclusively for AddressSpace

  friend class ObjectHolder;
  unsigned refCount;

  const MemoryObject *object;

  std::vector<SubObject> subObjects;
  std::vector<SubObject> subSegments;

  uint8_t *concreteStore;

  // XXX cleanup name of flushMask (its backwards or something)
  BitArray *concreteMask;

  // mutable because may need flushed during read of const
  mutable BitArray *flushMask;

  ref<Expr> *knownSymbolics;

  // mutable because we may need flush during read of const
  mutable UpdateList updates;

  mutable UpdateList rewrittenUpdates;

  mutable size_t pulledUpdates;

  mutable size_t minUpdates;

public:
  unsigned size;

  bool readOnly;

  unsigned originalSize;
  bool isSplit;

public:
  /// Create a new object state for the given memory object with concrete
  /// contents. The initial contents are undefined, it is the callers
  /// responsibility to initialize the object contents appropriately.
  ObjectState(const MemoryObject *mo);

  /// Create a new object state for the given memory object with symbolic
  /// contents.
  ObjectState(const MemoryObject *mo, const Array *array);

  ObjectState(const ObjectState &os);
  ~ObjectState();

  const MemoryObject *getObject() const { return object; }

  const std::vector<SubObject> &getSubObjects() const {
    return subObjects;
  }

  void addSubObject(unsigned int offset, uint64_t size, const SymbolicAddressInfo &info) {
    subObjects.push_back(SubObject(offset, size, info));
  }

  /* the size of the object when placed in a segment */
  uint64_t getEffectiveSize() const {
    if (subObjects.empty()) {
      return size;
    } else {
      size_t total = 0;
      for (const SubObject &so : subObjects) {
        total += so.size;
      }
      return total;
    }
  }

  const std::vector<SubObject> &getSubSegments() const {
    return subSegments;
  }

  void addSubSegment(unsigned int offset, uint64_t size, const SymbolicAddressInfo &info) {
    subSegments.push_back(SubObject(offset, size, info));
  }

  void setReadOnly(bool ro) { readOnly = ro; }

  // make contents all concrete and zero
  void initializeToZero();
  // make contents all concrete and random
  void initializeToRandom();

  ref<Expr> read(ref<Expr> offset, Expr::Width width) const;
  ref<Expr> read(unsigned offset, Expr::Width width) const;
  ref<Expr> read8(unsigned offset) const;

  // return bytes written.
  void write(unsigned offset, ref<Expr> value);
  void write(ref<Expr> offset, ref<Expr> value);

  void write8(unsigned offset, uint8_t value);
  void write16(unsigned offset, uint16_t value);
  void write32(unsigned offset, uint32_t value);
  void write64(unsigned offset, uint64_t value);
  void print() const;

  /*
    Looks at all the symbolic bytes of this object, gets a value for them
    from the solver and puts them in the concreteStore.
  */
  void flushToConcreteStore(TimingSolver *solver,
                            const ExecutionState &state) const;

  void getArrays(std::set<const Array *> &arrays) const;

  bool isSegment() const;

  const Array *getArray() {
    return updates.root;
  }

private:
  const UpdateList &getUpdates() const;

  void makeConcrete();

  void makeSymbolic();

  ref<Expr> read8(ref<Expr> offset) const;
  void write8(unsigned offset, ref<Expr> value);
  void write8(ref<Expr> offset, ref<Expr> value);

  void fastRangeCheckOffset(ref<Expr> offset, unsigned *base_r, 
                            unsigned *size_r) const;
  void flushRangeForRead(unsigned rangeBase, unsigned rangeSize) const;
  void flushRangeForWrite(unsigned rangeBase, unsigned rangeSize);

  bool isByteConcrete(unsigned offset) const;
  bool isByteFlushed(unsigned offset) const;
  bool isByteKnownSymbolic(unsigned offset) const;

  void markByteConcrete(unsigned offset);
  void markByteSymbolic(unsigned offset);
  void markByteFlushed(unsigned offset);
  void markByteUnflushed(unsigned offset);
  void setKnownSymbolic(unsigned offset, Expr *value);

  ArrayCache *getArrayCache() const;
};
  
} // End klee namespace

#endif
