# Control Library
Library containing controllers with generic application

### Authors/Maintainers
- Bernardo Fichera (bernardo.fichera@epfl.ch)

### Available Controller
- PID

### Dependencies
This library depends on **Eigen**
```sh
git clone https://gitlab.com/libeigen/eigen.git (git@gitlab.com:libeigen/eigen.git)
cd eigen && mkdir build && cmake .. && (sudo) make install
```

### Installation
Clone the repository including the submodules
```sh
git clone --recursive https://github.com/nash169/control-lib.git (git@github.com:nash169/control-lib.git)
```
**control-lib** relies on WAF compilation tool.
Arch provides an updated version of WAF exec in the standard repo
```sh
sudo pacman -S waf
```
For other distros it is better to download the latest version from the official website and move the executable in the library repo
```sh
wget 'https://waf.io/waf-2.0.23'
mv waf-2.0.19waf-2.0.23 waf && mv waf /path/to/control-lib
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
In order to set the desired compiler define the environment variable CXX=<g++,clang++,icpc> (gnu, clang and intel compiler respectively).

It is highly recommended to compile with AVX support
```sh
waf (./waf) configure --release
```
Compile static library (default option)
```sh
waf (./waf) configure --static
```
Compile shared library
```sh
waf (./waf) configure --shared
```
Define a specific installation path
```sh
waf (./waf) configure --prefix=/path/to/install/folder
```

### Finding the library
In order to find and link the lib to other projects copy and paste the following file into the waf tools
```sh
scripts/controllib.py
```

### Examples
Once the library is compiled all the examples can be found in
```sh
./build/src/examples/
```