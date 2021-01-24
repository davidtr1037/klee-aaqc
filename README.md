Address-Aware Query Caching for KLEE
=============================

The _relocatable addressing model_ enables dynamic merging and splitting of object representations.
More details can be found in this [paper](https://dl.acm.org/doi/10.1145/3395363.3397363).
This tool is an implementation of that model on top of KLEE.

## Build

### Requirements
Install the following packages:
```
sudo apt-get install cmake bison flex libboost-all-dev python perl zlib1g-dev build-essential curl libcap-dev git cmake libncurses5-dev python-minimal python-pip unzip libtcmalloc-minimal4 libgoogle-perftools-dev libsqlite3-dev doxygen
pip3 install tabulate wllvm
```

### LLVM 7.0

```
wget https://releases.llvm.org/7.0.0/llvm-7.0.0.src.tar.xz
wget https://releases.llvm.org/7.0.0/cfe-7.0.0.src.tar.xz
wget https://releases.llvm.org/7.0.0/compiler-rt-7.0.0.src.tar.xz
tar xJf llvm-7.0.0.src.tar.xz
tar xJf cfe-7.0.0.src.tar.xz
tar xJf compiler-rt-7.0.0.src.tar.xz
mv cfe-7.0.0.src llvm-7.0.0.src/tools/clang
mv compiler-rt-7.0.0.src compiler-rt
mkdir llvm-7.0.0.obj
cd llvm-7.0.0.obj
cmake CMAKE_BUILD_TYPE:STRING=Release -DLLVM_ENABLE_THREADS:BOOL=ON -DLLVM_ENABLE_PROJECTS:STRING=compiler-rt ../llvm-7.0.0.src
make -j4
```
Update the following environment variables:
```
export PATH=<llvm_build_dir>/bin:$PATH
export LLVM_COMPILER=clang
```

### minisat

```
git clone https://github.com/stp/minisat.git
cd minisat
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local/ ../
sudo make install
```

### STP

```
git clone https://github.com/stp/stp.git
cd stp
git checkout tags/2.3.3
mkdir build
cd build
cmake ..
make
sudo make install
```

### klee-uclibc
```
git clone https://github.com/klee/klee-uclibc.git
cd klee-uclibc
./configure --make-llvm-lib
make
```

### Our Tool
To build our tool, which is an extension of KLEE, do the following:
```
mkdir <klee_aaqc_build_dir>
cd <klee_aaqc_build_dir>
CXXFLAGS="-fno-rtti" cmake \
    -DENABLE_SOLVER_STP=ON \
    -DENABLE_POSIX_RUNTIME=ON \
    -DENABLE_KLEE_UCLIBC=ON \
    -DKLEE_UCLIBC_PATH=<klee_uclibc_dir> \
    -DLLVM_CONFIG_BINARY=<llvm_build_dir>/bin/llvm-config \
    -DLLVMCC=<llvm_build_dir>/bin/clang \
    -DLLVMCXX=<llvm_build_dir>/bin/clang++ \
    -DENABLE_UNIT_TESTS=OFF \
    -DKLEE_RUNTIME_BUILD_TYPE=Release+Asserts \
    -DENABLE_SYSTEM_TESTS=ON \
    -DENABLE_TCMALLOC=ON \
    <klee_aaqc_dir>
make -j4
```

## Usage
### Command Line Options
We extended KLEE with several command line options.
The main options are:
- _use-sym-addr_: use the symbolic addressing model (required for address-aware query caching)
- _use-iso-cahce_: use the address-aware query caching
- _collect-query-stats_: use the address-aware query caching
- _validate-caching_: validate the correctness of the caching
