#! /usr/bin/env python
# encoding: utf-8

from waflib.Configure import conf


def options(opt):
    opt.add_option("--eigen", type="string", help="path to eigen", dest="eigen")
    # to-do: rename this to --eigen_lapacke
    opt.add_option(
        "--lapacke_blas",
        action="store_true",
        help="enable lapacke/blas if found (required Eigen>=3.3)",
        dest="lapacke_blas",
    )


@conf
def check_eigen(ctx, **kwargs):
    def get_directory(filename, dirs):
        res = ctx.find_file(filename, dirs)
        return res[: -len(filename) - 1]

    includes_check = [
        "/usr/include/eigen3",
        "/usr/local/include/eigen3",
        "/usr/include",
        "/usr/local/include",
    ]

    required = kwargs.get("required", False)

    # OSX/Mac uses .dylib and GNU/Linux .so
    suffix = "dylib" if ctx.env["DEST_OS"] == "darwin" else "so"

    if ctx.options.eigen:
        includes_check = [ctx.options.eigen]

    try:
        ctx.start_msg("Checking for Eigen")
        incl = get_directory("Eigen/Core", includes_check)
        ctx.env.INCLUDES_EIGEN = [incl]
        ctx.end_msg(incl)
        if ctx.options.lapacke_blas:
            ctx.start_msg("Checking for LAPACKE/BLAS (optional)")
            world_version = -1
            major_version = -1
            minor_version = -1

            config_file = ctx.find_file("Eigen/src/Core/util/Macros.h", includes_check)
            with open(config_file) as f:
                config_content = f.readlines()
            for line in config_content:
                world = line.find("#define EIGEN_WORLD_VERSION")
                major = line.find("#define EIGEN_MAJOR_VERSION")
                minor = line.find("#define EIGEN_MINOR_VERSION")
                if world > -1:
                    world_version = int(line.split(" ")[-1].strip())
                if major > -1:
                    major_version = int(line.split(" ")[-1].strip())
                if minor > -1:
                    minor_version = int(line.split(" ")[-1].strip())
                if world_version > 0 and major_version > 0 and minor_version > 0:
                    break

            if world_version == 3 and major_version >= 3:
                # Check for lapacke and blas
                extra_libs = [
                    "/usr/lib",
                    "/usr/local/lib",
                    "/usr/local/opt/openblas/lib",
                ]
                blas_libs = ["blas", "openblas"]
                blas_lib = ""
                blas_path = ""
                for b in blas_libs:
                    try:
                        blas_path = get_directory("lib" + b + "." + suffix, extra_libs)
                    except:
                        continue
                    blas_lib = b
                    break

                lapacke = False
                lapacke_path = ""
                try:
                    lapacke_path = get_directory("liblapacke." + suffix, extra_libs)
                    lapacke = True
                except:
                    lapacke = False

                if lapacke or blas_lib != "":
                    ctx.env.DEFINES_EIGEN = []
                    if lapacke_path != blas_path:
                        ctx.env.LIBPATH_EIGEN = [lapacke_path, blas_path]
                    else:
                        ctx.env.LIBPATH_EIGEN = [lapacke_path]
                    ctx.env.LIB_EIGEN = []
                    ctx.end_msg("LAPACKE: '%s', BLAS: '%s'" % (lapacke_path, blas_path))
                elif lapacke:
                    ctx.end_msg("Found only LAPACKE: %s" % lapacke_path, "YELLOW")
                elif blas_lib != "":
                    ctx.end_msg("Found only BLAS: %s" % blas_path, "YELLOW")
                else:
                    ctx.end_msg("Not found in %s" % str(extra_libs), "RED")
                if lapacke:
                    ctx.env.DEFINES_EIGEN.append("EIGEN_USE_LAPACKE")
                    ctx.env.LIB_EIGEN.append("lapacke")
                if blas_lib != "":
                    ctx.env.DEFINES_EIGEN.append("EIGEN_USE_BLAS")
                    ctx.env.LIB_EIGEN.append(blas_lib)
            else:
                ctx.end_msg(
                    "Found Eigen version %s: LAPACKE/BLAS can be used only with Eigen>=3.3"
                    % (
                        str(world_version)
                        + "."
                        + str(major_version)
                        + "."
                        + str(minor_version)
                    ),
                    "RED",
                )
    except:
        if required:
            ctx.fatal("Not found in %s" % str(includes_check))
        ctx.end_msg("Not found in %s" % str(includes_check), "RED")


def configure(cfg):
    # Configuration
    cfg.check_eigen(required=True)

