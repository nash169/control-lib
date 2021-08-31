#! /usr/bin/env python
# encoding: utf-8

from waflib.Configure import conf
from utils import check_include, check_lib


def options(opt):
    # Options
    opt.add_option(
        "--control-path", type="string", help="path to control-lib", dest="control_path"
    )

    # Required package options
    opt.load("eigen corrade", tooldir="waf_tools")


@conf
def check_control(ctx):
    # Set the search path
    if ctx.options.control_path is None:
        path_check = ["/usr/local", "/usr"]
    else:
        path_check = [ctx.options.control_path]

    # control-lib includes
    check_include(ctx, "CONTROL", [""], [
                  "control_lib/Control.hpp"], path_check)

    # control-lib libs
    check_lib(ctx, "CONTROL", "", ["libControl"], path_check)

    if ctx.env.LIB_CONTROL or ctx.env.STLIB_CONTROL:
        # Add dependencies to require libraries
        if "EIGEN" not in ctx.get_env()["libs"]:
            ctx.get_env()["requires"] += ["EIGEN"]
            ctx.load("eigen", tooldir="waf_tools")

        if "CORRADE" not in ctx.get_env()["libs"]:
            ctx.get_env()["requires"] += ["CORRADE"]
            ctx.load("corrade", tooldir="waf_tools")

        # Add library
        ctx.get_env()["libs"] += ["CONTROL"]


def configure(cfg):
    if not cfg.env.LIB_CONTROL and not cfg.env.STLIB_CONTROL:
        cfg.check_control()
