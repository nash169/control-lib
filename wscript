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


def configure(cfg):
    # OSX/Mac uses .dylib and GNU/Linux .so
    cfg.env.SUFFIX = "dylib" if cfg.env["DEST_OS"] == "darwin" else "so"

    # Load compiler configuration
    cfg.load("compiler_cxx")
    cfg.load("compiler_c")

    # Load compiler flags
    cfg.load("flags", tooldir="waf_tools")

    # Load tools configuration
    cfg.load("eigen", tooldir="waf_tools")
    cfg.load("corrade", tooldir="waf_tools")

    # Set lib type
    if cfg.options.shared:
        cfg.env["lib_type"] = "cxxshlib"
    else:
        cfg.env["lib_type"] = "cxxstlib"


def build(bld):
    # Library name
    bld.get_env()["libname"] = "Control"

    # Includes
    includes = []
    includes_path = "src"
    for root, dirnames, filenames in os.walk(bld.path.abspath() + includes_path):
        for filename in fnmatch.filter(filenames, "*.hpp"):
            includes.append(os.path.join(root, filename))
    includes = [f[len(bld.path.abspath()) + 1:] for f in includes]

    # Sources
    sources = []
    sources_path = "src/libcontrol"
    for root, dirnames, filenames in os.walk(bld.path.abspath() + sources_path):
        for filename in fnmatch.filter(filenames, "*.cpp"):
            sources.append(os.path.join(root, filename))
    sources = " ".join([f[len(bld.path.abspath()) + 1:] for f in sources])

    # Build library
    if bld.options.shared:
        bld.shlib(
            features="cxx " + bld.env["lib_type"],
            source=sources,
            target=bld.get_env()["libname"],
            includes=includes_path,
            uselib=bld.get_env()["libs"],
            cxxxflags=bld.get_env()["CXXFLAGS"],
        )
    else:
        bld.stlib(
            features="cxx " + bld.env["lib_type"],
            source=sources,
            target=bld.get_env()["libname"],
            includes=includes_path,
            uselib=bld.get_env()["libs"],
            cxxxflags=bld.get_env()["CXXFLAGS"],
        )

    # Build examples
    bld.recurse("./src/examples")

    # Install headers
    for f in includes:
        end_index = f.rfind("/")
        if end_index == -1:
            end_index = len(f)
        bld.install_files("${PREFIX}/include/" + f[4:end_index], f)

    # Install libraries
    if bld.env["lib_type"] == "cxxstlib":
        bld.install_files(
            "${PREFIX}/lib", blddir + "/lib" + bld.get_env()["libname"] + ".a"
        )
    else:
        bld.install_files(
            "${PREFIX}/lib",
            blddir + "/lib" + bld.get_env()["libname"] + "." + bld.env.SUFFIX,
        )
