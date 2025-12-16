# trakr_sdk : Trakr Legged Robot
SDK for talking to *Trakr* over a TCP/IP Socket.

This repository includes the explaination and documentation regarding different modes of *Trakr*, and methods to switch between them over Network.

> Before diving into this repository, go through the User Manual provided with the robot to get an understanding of basic controls of Trakr.

## Implementations

The SDK is available with both Python and C++ Implementations.

### 1. C++ SDK

The C++'s implementation features a CMake package, **_trakr_cpp_sdk_**, that can be built from source and installed, and can be included in different CMake workspaces through `find_package(trakr_cpp_sdk)`.

See `cpp/trakr_cpp_sdk/` for its implementation

### 2. Python SDK
The Python's implementation features a Python package, **_trakr_sdk_**, available both as a pre-built *wheel* package, and as source that can be installed as an experimental package.

See `python/` for package, and `examples/python/` for examples to use the package.

## Documentation
Refer to `README.md` inside the respective SDK to see instructions for installation and basic-usage. For a general usage and conceptual understanding, see different README files in`docs/` for their respective explainations.