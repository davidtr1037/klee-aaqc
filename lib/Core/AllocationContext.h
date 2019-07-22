#ifndef KLEE_ALLOCATION_CONTEXT_H
#define KLEE_ALLOCATION_CONTEXT_H

#include "llvm/Support/raw_ostream.h"


namespace klee {

class AllocationContext {

public:

  uint64_t hash;

  AllocationContext() : hash(0) {

  }

  AllocationContext(uint64_t hash) : hash(hash) {

  }

  bool operator==(const AllocationContext &other) {
    return hash == other.hash;
  }

  bool operator!=(const AllocationContext &other) {
    return !(this->operator==(other));
  }

  void dump() const {
    llvm::errs() << "Hash: " << hash << "\n";
  }
};

}

#endif /* KLEE_ALLOCATION_CONTEXT_H */
