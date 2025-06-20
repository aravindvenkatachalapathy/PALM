# PALM-OpenFAST Coupling

This project demonstrates how to couple [PALM](https://palm-model.readthedocs.io/en/latest/) with [OpenFAST](https://openfast.readthedocs.io/en/main/) using the OpenFAST C++ API.

---

## Prerequisites

- CMake ≥ 3.10
- GCC / Clang (C++ compiler)
- OpenFAST (built with C++ API and shared libs)
- YAML-CPP (optional, if your config uses YAML parsing)
- Git

---

## Step 1: Clone and Build OpenFAST

```bash
git clone https://github.com/OpenFAST/OpenFAST.git
cd OpenFAST
mkdir build && cd build

cmake .. -DBUILD_OPENFAST_CPP_API=ON \
         -DBUILD_SHARED_LIBS=ON \
         -DCMAKE_INSTALL_PREFIX=/opt/openfast

make -j$(nproc) openfastcpplib
sudo make install
```

## Step 2
```bash
cd ~
mkdir PALM-OpenFAST-Coupling
cd PALM-OpenFAST-Coupling
```
# File Structure
```graphql
PALM-OpenFAST-Coupling/
├── PALM_OpenFAST_Driver.cpp      # Your coupling driver code
├── CMakeLists.txt                # Project CMake configuration
├── build/                        # Directory for CMake build files
└── config/                       # YAML config and turbine definitions
    ├── palm_openfast_config.yaml
    └── turbine.srv
```
Build your project now

```bash
mkdir build && cd build
cmake ..
make
```
## Notes
Ensure OpenFAST is installed in the path specified by CMAKE_INSTALL_PREFIX or adjust your CMakeLists.txt accordingly.

Update CMakeLists.txt to link against the OpenFAST C++ API and include any necessary dependencies.
