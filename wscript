#!/usr/bin/env python
# encoding: utf-8

import os
import fnmatch

VERSION = "1.0.0"
APPNAME = "libcontrol"

srcdir = "."
blddir = "build"


def options(opt):
    # Load compiler options
    opt.load("compiler_cxx")
    opt.load("compiler_c")

    # Load tools options
    opt.load("eigen", tooldir="waf_tools")
    opt.load("corrade", tooldir="waf_tools")

    # Add options
    opt.add_option("--shared", action="store_true",
                   help="build shared library")
    opt.add_option("--static", action="store_true",
                   help="build static library")
    opt.add_option(
        "--no-avx",
        action="store_true",
        help="build without AVX flags",
        dest="disable_avx",
    )
    opt.add_option(
        "--tests", action="store_true", help="compile tests or not", dest="tests"
    )


def configure(cfg):
    # OSX/Mac uses .dylib and GNU/Linux .so
    cfg.env.SUFFIX = "dylib" if cfg.env["DEST_OS"] == "darwin" else "so"

    # Load compiler configuration
    cfg.load("compiler_cxx")
    cfg.load("compiler_c")

    # Load tools configuration
    cfg.load("eigen", tooldir="waf_tools")
    cfg.load("corrade", tooldir="waf_tools")

    # Set lib type
    cfg.env["lib_type"] = "cxxstlib"
    if cfg.options.shared:
        cfg.env["lib_type"] = "cxxshlib"

    # Compiler flags
    if cfg.env.CXX_NAME in ["icc", "icpc"]:
        common_flags = "-Wall -std=c++14"
        opt_flags = " -O3 -xHost -mtune=native -unroll -g"
    elif cfg.env.CXX_NAME in ["clang"]:
        common_flags = "-Wall -std=c++14"
        opt_flags = " -O3 -march=native -g -faligned-new"
    else:
        gcc_version = int(cfg.env["CC_VERSION"][0] + cfg.env["CC_VERSION"][1])
        if gcc_version < 47:
            common_flags = "-Wall -std=c++0x"
            # cfg.fatal("Compiler should support C++14")
        else:
            common_flags = "-Wall -std=c++14"
        opt_flags = " -O3 -march=native -g"
        if gcc_version >= 71:
            opt_flags = opt_flags + " -faligned-new"

    all_flags = common_flags + opt_flags
    cfg.env["CXXFLAGS"] = cfg.env["CXXFLAGS"] + all_flags.split(" ") + ["-w"]


def build(bld):
    # Check if all the libraries have been loaded correctly
    if len(bld.env.INCLUDES_EIGEN) == 0:
        bld.fatal("Some libraries were not found! Cannot proceed!")

    # Library name
    libname = "libControl"
    bld.get_env()["libname"] = libname

    # Define necessary includes
    includes = "./src"

    # Define necessary libraries
    libs = "EIGEN"
    bld.get_env()["libs"] = libs

    # Compiler flags
    cxxflags = bld.get_env()["CXXFLAGS"]

    # Get source files
    files = []
    for root, dirnames, filenames in os.walk(bld.path.abspath() + "/src/libcontrol/"):
        for filename in fnmatch.filter(filenames, "*.cpp"):
            files.append(os.path.join(root, filename))

    files = [f[len(bld.path.abspath()) + 1:] for f in files]
    libcontrol_srcs = " ".join(files)

    # Build library
    if bld.options.shared:
        bld.shlib(
            features="cxx " + bld.env["lib_type"],
            source=libcontrol_srcs,
            target=libname,
            includes=includes,
            uselib=libs,
            cxxxflags=cxxflags,
        )
    elif bld.options.static:
        bld.stlib(
            features="cxx " + bld.env["lib_type"],
            source=libcontrol_srcs,
            target=libname,
            includes=includes,
            uselib=libs,
            cxxxflags=cxxflags,
        )
    else:
        bld.stlib(
            features="cxx " + bld.env["lib_type"],
            source=libcontrol_srcs,
            target=libname,
            includes=includes,
            uselib=libs,
            cxxxflags=cxxflags,
        )

    # Build examples
    bld.recurse("./src/examples")

    # Define headers to install
    install_files = []
    for root, dirnames, filenames in os.walk(bld.path.abspath() + "/src/"):
        for filename in fnmatch.filter(filenames, "*.hpp"):
            install_files.append(os.path.join(root, filename))
    install_files = [f[len(bld.path.abspath()) + 1:] for f in install_files]

    # Install headers
    for f in install_files:
        end_index = f.rfind("/")
        if end_index == -1:
            end_index = len(f)
        bld.install_files("${PREFIX}/include/" + f[4:end_index], f)

    # Install libraries
    if bld.env["lib_type"] == "cxxstlib":
        bld.install_files("${PREFIX}/lib", blddir + "/lib" + libname + ".a")
    else:
        bld.install_files(
            "${PREFIX}/lib", blddir + "/lib" + libname + "." + bld.env.SUFFIX
        )
