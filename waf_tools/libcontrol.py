#! /usr/bin/env python
# encoding: utf-8

from waflib.Configure import conf


def options(opt):
    # Required package options
    opt.load("eigen", tooldir="waf_tools")
    opt.load("corrade", tooldir="waf_tools")

    # Options
    opt.add_option(
        "--libcontrol", type="string", help="path to libcontrol", dest="libcontrol"
    )


@conf
def check_libcontrol(ctx, **kwargs):
    # Get directory containing file
    def get_directory(filename, dirs):
        res = ctx.find_file(filename, dirs)
        return res[: -len(filename) - 1]

    # OSX/Mac uses .dylib and GNU/Linux .so
    suffix = "dylib" if ctx.env["DEST_OS"] == "darwin" else "so"

    # Get required variable and set it to False if not define
    required = kwargs.get("required", False)

    # Get components and set it empty if not define
    components = kwargs.get("components", [])

    # Define INCLUDES
    if ctx.options.libcontrol:
        includes_check = [ctx.options.libcontrol, ctx.options.libcontrol + "/include"]
    else:
        includes_check = ["/usr/include", "/usr/local/include"]

    try:
        ctx.start_msg("Checking for libcontrol includes")
        ctx.env.INCLUDES_LIBCONTROL = [
            get_directory("libcontrol/ControlState.hpp", includes_check)
        ]
        ctx.end_msg("libcontrol include found in %s" % str(ctx.env.INCLUDES_LIBCONTROL))
    except:
        if required:
            ctx.fatal("Not found in %s" % str(includes_check))
        ctx.end_msg("Not found in %s" % str(includes_check), "YELLOW")

    # Define LIBS
    if ctx.options.libcontrol:
        lib_checks = [ctx.options.libcontrol, ctx.options.libcontrol + "/lib"]
    else:
        lib_checks = ["/usr/lib", "/usr/local/lib"]

    try:
        ctx.start_msg("Checking for libcontrol libs")

        ctx.env.LIBPATH_LIBCONTROL = get_directory("libControl." + suffix, lib_checks)
        ctx.end_msg("libcontrol libs found in %s" % str(ctx.env.LIBPATH_LIBCONTROL))
        ctx.env.LIB_LIBCONTROL = ["Control"]
    except:
        if required:
            ctx.fatal("libcontrol libs not found in %s" % str(lib_checks))
        ctx.end_msg("libcontrol libs not found in %s" % str(lib_checks), "YELLOW")


def configure(cfg):
    # Required package configuration
    cfg.load("eigen", tooldir="waf_tools")
    cfg.load("corrade", tooldir="waf_tools")

    # Configuration
    cfg.check_libcontrol(required=True)
