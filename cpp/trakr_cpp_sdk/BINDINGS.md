# Python Bindings for trakr_sdk
To support whichever platform (amd64/aarch64) this code is required to be run on, the Python bindings are generated over the same C++ libraries used by trakr_cpp_sdk when the package is built.

To build the bindings, following needs to be installed,
- **Python == 3.8**
- **pybind11** : Install with `python3.8 pm pip install pybind11`

The package has the same OS/Architecture support as *trakr_cpp_sdk*

### Installation

1. Set the flag `BUILD_PYTHON_BINDINGS` as `ON` in [`CMakeLists.txt`](CMakeLists.txt)
2. Build the *trakr_cpp_sdk* package
3. Install the package with `sudo make install -j`
> The bindings should be automatically installed under `../../python/trakr_sdk/lib` directory.

### Advanced (NOT Recommended)
The bindings have been tested with Python3.8. To support/install on any other version, you can, at your own risk, change the required Python version/compiler in [`CMakeLists.txt`](CMakeLists.txt), build the package and install it.