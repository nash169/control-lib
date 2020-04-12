# Control Library
Library containing controllers with generic application

### Authors/Maintainers
- Bernardo Fichera (bernardo.fichera@epfl.ch)

### Available Controller
- PID

### Dependencies
This library depends on **eigen** for the linear algebra. The latest git version is required.
```sh
git clone https://gitlab.com/libeigen/eigen.git (git@gitlab.com:libeigen/eigen.git)
cd eigen && mkdir build && cmake .. && (sudo) make install
```
For various utilities the library depends on **corrade** utility library.
```sh
git clone https://github.com/mosra/corrade.git (git@github.com:mosra/corrade.git)
cd corrade && mkdir build && cmake .. && make && (sudo) make install
```
Other (optional) dependencies for improving performances required by Eigen are:
- Linear Algebra -> LAPACK, BLAS, MKL
- Multi-threading -> OPENMP, TBB

### Installation
Clone the repository including the submodules
```sh
git clone --recursive https://github.com/nash169/integrator-lib.git (git@github.com:nash169/integrator-lib.git)
```
**control-lib** relies on WAF compilation tool.
Arch provides an updated version of WAF exec in the standard repo
```sh
sudo pacman -S waf
```
For other distros it is better to download the latest version from the official website and move the executable in the library repo
```sh
wget 'https://waf.io/waf-2.0.19'
mv waf-2.0.19waf-2.0.19 waf && mv waf /path/to/integrator-lib
cd /path/to/kernel-lib
chmod +x waf
```
Compile and install using waf commands
```sh
waf (./waf) configure build
```
or
```sh
waf (./waf) configure && waf (./waf)
```
Install the library (optional)
```sh
(sudo) waf (./waf) install
```
If you want to make clean installation
```sh
(sudo) waf (./waf) distclean configure build install
```

#### Compilation options
It is highly recommended to compile with AVX support
```sh
waf (./waf) configure --optional-flags
```
Enable OPENMP multi-threading
```sh
waf (./waf) configure --multi-threading
```
Enable LAPACK
```sh
waf (./waf) configure --with-lapack
```
Enable BLAS
```sh
waf (./waf) configure --with-blas
```
Enable MKL
```sh
waf (./waf) configure --with-mkl
```
Set MKL multi-threading
```sh
waf (./waf) configure --mkl-threading=<sequential|openmp|tbb>
```
By default MKL uses `sequential` option. If you activate `openmp` this includes `--multi-threading` that will be deactivated in this case. In addition if you choose OpenMP multi-threading it is possible select between the GNU (default), `gnu`, or Intel, `intel`, version through `--mkl-openmp` option. 
Suggested configuration
```sh
waf (./waf) configure --optional-flags --with-lapack --with-blas --with-mkl --mkl-threading=tbb
```
In order to set the desired compiler define the environment variable CXX=<g++,clang++,icpc> (gnu, clang and intel compiler respectively).

### Examples
```sh
./build/src/examples/test
```
